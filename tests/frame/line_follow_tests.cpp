#include "LineFollowApp.hpp"
#include "MotorSpeedProfiles.hpp"
#include "bsp_oled_bus.h"
#include "stm32f1xx_hal.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <limits>
#include <vector>

TestDwt test_dwt = {};
TestCoreDebug test_core_debug = {};
uint32_t test_tick = 0;
uint32_t test_primask = 0;

namespace
{
    bool bus_error = false;
    bool bus_present = true;
    uint32_t bus_ready_ms = 0;
    uint8_t pages = 0;
    uint8_t screen[512] = {};
    std::vector<uint32_t> completed_frames;

    GraySensor::Sample MakeSample(uint8_t white_mask, uint32_t now)
    {
        GraySensor::Sample sample = {};
        sample.channels = 8;
        sample.valid = true;
        sample.timestamp_ms = now;
        for (uint8_t index = 0; index < 8; ++index)
        {
            sample.line[index] = (white_mask & (1U << index)) ? 1.0f : 0.0f;
        }
        return sample;
    }

    void TestTrack()
    {
        Track track;
        Track::Config config;
        assert(track.Init(config));
        assert(!track.Update(MakeSample(0, 0), 0).valid);
        assert(track.GetCommand().state == Track::State::WaitingForLine);
        assert(!track.Update(MakeSample(0x18, 5), 5).valid);
        for (int repeat = 0; repeat < 10; ++repeat)
        {
            assert(!track.Update(MakeSample(0x18, 5), 5).valid);
        }
        assert(!track.Update(MakeSample(0x18, 10), 10).valid);
        const auto &center = track.Update(MakeSample(0x18, 15), 15);
        assert(center.valid && center.state == Track::State::Following);
        assert(fabsf(center.speed_diff_rpm) < 0.001f && center.base_rpm == 40);
        assert(track.Update(MakeSample(0x80, 20), 20).speed_diff_rpm > 0);
        const float last_turn = track.GetCommand().speed_diff_rpm;
        assert(track.Update(MakeSample(0, 25), 25).state == Track::State::GapHold);
        assert(track.GetCommand().speed_diff_rpm == last_turn);
        assert(track.Update(MakeSample(0, 141), 141).state == Track::State::Searching);
        assert(track.GetCommand().speed_diff_rpm > 0);
        assert(track.Update(MakeSample(0x18, 145), 145).state == Track::State::Searching);
        assert(track.Update(MakeSample(0x18, 150), 150).state == Track::State::Searching);
        const auto &recovered = track.Update(MakeSample(0x18, 155), 155);
        assert(recovered.state == Track::State::Following);
        assert(fabsf(recovered.speed_diff_rpm) < 0.001f); // No derivative kick on reacquisition.
        assert(track.Update(MakeSample(0xFF, 160), 160).state == Track::State::GapHold);
        assert(track.Update(MakeSample(0x81, 165), 165).state == Track::State::GapHold);
        assert(track.Update(MakeSample(0x18, 1355), 1355).state == Track::State::Lost);
        assert(!track.Update(MakeSample(0x18, 1360), 1360).valid); // Latched loss.

        track.Reset();
        assert(track.Update(MakeSample(0x18, 1400), 1451).state == Track::State::Invalid);
        track.Reset();
        auto bad = MakeSample(0x18, 0);
        bad.line[7] = std::numeric_limits<float>::quiet_NaN();
        assert(track.Update(bad, 0).state == Track::State::Invalid);
        track.Reset();
        bad = MakeSample(0x18, 0);
        bad.channels = FRAME_GRAY_MAX_CHANNELS + 1U;
        assert(track.Update(bad, 0).state == Track::State::Invalid);
        config.kp = std::numeric_limits<float>::infinity();
        assert(!track.Init(config));
        config = {};
        assert(track.Init(config));
        for (uint32_t now = 0xFFFFFFF0U; now != 0xFFFFFFFFU; now += 5U)
        {
            track.Update(MakeSample(0x01, now), now);
        }
        assert(track.GetCommand().state == Track::State::Following);
        assert(track.Update(MakeSample(0x01, 4), 4).speed_diff_rpm < 0);
        assert(track.Update(MakeSample(0, 9), 9).state == Track::State::GapHold);

        // Both polarities, all 256 GPIO combinations: finite and bounded commands.
        for (int polarity = 0; polarity < 2; ++polarity)
        {
            config.white_line = polarity == 0;
            assert(track.Init(config));
            for (unsigned mask = 0; mask < 256; ++mask)
            {
                track.Reset();
                const uint8_t raw_mask = config.white_line ? (uint8_t)mask : (uint8_t)~mask;
                track.Update(MakeSample(raw_mask, 0), 0);
                track.Update(MakeSample(raw_mask, 5), 5);
                const auto &command = track.Update(MakeSample(raw_mask, 10), 10);
                assert(isfinite(command.base_rpm) && isfinite(command.speed_diff_rpm));
                assert(command.base_rpm >= 0 && command.base_rpm <= config.base_rpm);
                assert(fabsf(command.speed_diff_rpm) <= config.maximum_diff_rpm);
                if (mask == 0 || mask == 255 || mask == 129)
                {
                    assert(!command.valid);
                }
            }
        }
    }

    void TestMixer()
    {
        SpeedMixer mixer;
        SpeedMixer::WheelSpeeds wheels;
        assert(mixer.Init());
        assert(!mixer.GetWheelSpeeds(0, wheels));
        assert(mixer.SetFollowOffset(20, 0));
        assert(!mixer.GetWheelSpeeds(0, wheels));
        mixer.ClearAll();
        assert(mixer.SetTrackSpeed(40, 10, 0));
        assert(mixer.GetWheelSpeeds(0, wheels));
        assert(wheels.rpm[0] == 50 && wheels.rpm[1] == 50);
        assert(wheels.rpm[2] == 30 && wheels.rpm[3] == 30);
        assert(mixer.SetFollowOffset(30, 0));
        assert(mixer.GetWheelSpeeds(0, wheels));
        assert(wheels.rpm[0] == 60 && wheels.rpm[2] == 45); // Ratio-preserving saturation.
        assert(mixer.SetDirectSpeed(SpeedMixer::Source::Navigation, 5, 6, 1));
        assert(mixer.GetWheelSpeeds(1, wheels) && wheels.rpm[0] == 5 && wheels.rpm[3] == 6);
        assert(!mixer.SetDirectSpeed(SpeedMixer::Source::Navigation, -1, 6, 2));
        assert(mixer.GetWheelSpeeds(2, wheels) && wheels.source == SpeedMixer::Source::Follow);
        assert(!mixer.GetWheelSpeeds(50, wheels));
        assert(mixer.SetTrackSpeed(40, -24, 0xFFFFFFF0U));
        assert(mixer.GetWheelSpeeds(4, wheels) && wheels.rpm[0] < wheels.rpm[2]);
        assert(!mixer.GetWheelSpeeds(34, wheels));
        assert(!mixer.SetTrackSpeed(40, std::numeric_limits<float>::quiet_NaN(), 35));
        assert(!mixer.GetWheelSpeeds(35, wheels));
        assert(!mixer.Init(161));
        assert(!mixer.Init(60, 101));
    }

    void TestFormatting()
    {
        char text[22];
        assert(MotorSpeedDisplay::FormatLine(1, true, 40.24f, text));
        assert(strcmp(text, "M1 040.2 RPM") == 0);
        assert(MotorSpeedDisplay::FormatLine(2, true, -3.26f, text));
        assert(strcmp(text, "M2 -003.3 RPM") == 0);
        assert(MotorSpeedDisplay::FormatLine(3, true, -0.001f, text));
        assert(strcmp(text, "M3 000.0 RPM") == 0);
        assert(MotorSpeedDisplay::FormatLine(4, false, 50, text));
        assert(strcmp(text, "M4 ---.- RPM") == 0);
        assert(
            MotorSpeedDisplay::FormatLine(4, true, std::numeric_limits<float>::infinity(), text));
        assert(strcmp(text, "M4 ---.- RPM") == 0);
        assert(MotorSpeedDisplay::FormatLine(4, true, 1e30f, text));
        assert(strcmp(text, "M4 RANGE RPM") == 0);
        assert(!MotorSpeedDisplay::FormatLine(5, true, 40, text));
    }

    struct Wheel
    {
        float rpm = 0;
        float duty = 0;
        uint32_t sampled_ms = 0;
        bool output_failure = false;
        bool feedback_lost = false;
    };

    bool SetDuty(void *context, float duty)
    {
        Wheel &wheel = *(Wheel *)context;
        wheel.duty = duty;
        return !wheel.output_failure;
    }

    void Stop(void *context)
    {
        ((Wheel *)context)->duty = 0;
    }

    bool ReadRpm(void *context, float *rpm)
    {
        Wheel &wheel = *(Wheel *)context;
        if (wheel.feedback_lost || (uint32_t)(test_tick - wheel.sampled_ms) < 10U)
        {
            return false;
        }
        wheel.sampled_ms = test_tick;
        *rpm = wheel.rpm;
        return true;
    }

    uint8_t white_mask = 0x18;
    bool gray_valid = true;
    bool gray_missing = false;

    bool ReadGray(void *, GraySensor::Sample *sample)
    {
        if (gray_missing)
        {
            return false;
        }
        *sample = MakeSample(white_mask, test_tick);
        sample->valid = gray_valid;
        return true;
    }

    void TestApplication()
    {
        Wheel wheels[4];
        DcMotor motors[4];
        GraySensor gray;
        for (uint8_t index = 0; index < 4; ++index)
        {
            DcMotor::Driver driver = {&wheels[index], SetDuty, Stop, ReadRpm};
            assert(motors[index].Init(driver));
            assert(MotorSpeedProfiles::Apply(motors[index], index + 1U, false));
        }
        gray.Bind(nullptr, ReadGray);
        LineFollowApp app(gray, motors[0], motors[1], motors[2], motors[3]);
        LineFollowApp::Config config;
        test_tick = 0;
        System.Init();
        assert(app.Init(config));
        assert(System.RegistApp(app));
        assert(!app.RequestStart());
        const auto tick = [&]()
        {
            ++test_tick;
            test_dwt.CYCCNT += 72000U;
            gray.Update();
            if (test_tick % 5U == 0U)
            {
                System.UpdateApplications();
                System.Run();
            }
            app.Control();
            DcMotor::ControlAllMotors();
            app.UpdateDisplay();
        };
        const auto run = [&](uint32_t duration)
        {
            for (uint32_t elapsed = 0; elapsed < duration; ++elapsed)
            {
                tick();
            }
        };
        const auto assert_stopped = [&]()
        {
            for (uint8_t index = 0; index < 4; ++index)
            {
                assert(!motors[index].IsEnabled() && wheels[index].duty == 0);
            }
        };
        const auto prepare = [&]()
        {
            for (uint8_t index = 0; index < 4; ++index)
            {
                motors[index].Disable();
                assert(motors[index].ClearFault());
                wheels[index].output_failure = wheels[index].feedback_lost = false;
            }
            gray_valid = true;
            gray_missing = false;
            white_mask = 0x18;
            System.Init();
            assert(app.Init(config));
            run(20);
            assert(System.GetState() == Systems::READY);
            assert(app.RequestStart());
            run(30);
            assert(System.GetState() == Systems::WORKING);
        };

        run(100);
        assert(System.GetState() == Systems::READY);
        assert_stopped(); // Healthy sensors do not authorize autonomous startup.
        white_mask = 0;
        assert(app.RequestStart());
        run(50);
        assert(System.GetState() == Systems::WORKING);
        assert_stopped();
        white_mask = 0x18;
        run(40);
        assert(motors[0].IsEnabled() && motors[3].IsEnabled());
        assert(motors[0].GetTargetSpeed() > 0 && motors[0].GetTargetSpeed() <= 4.3f);
        run(500);
        assert(fabsf(motors[0].GetTargetSpeed() - 40) < 0.001f);
        white_mask = 0x80;
        run(250);
        assert(motors[0].GetTargetSpeed() > motors[2].GetTargetSpeed());
        assert(motors[0].GetTargetSpeed() == motors[1].GetTargetSpeed());
        assert(motors[2].GetTargetSpeed() == motors[3].GetTargetSpeed());
        const float before = motors[0].GetTargetSpeed();
        run(5);
        assert(fabsf(motors[0].GetTargetSpeed() - before) <= 1.201f);
        white_mask = 0;
        run(160);
        assert(app.GetTrackCommand().state == Track::State::Searching);
        white_mask = 0x18;
        run(20);
        assert(app.GetTrackCommand().state == Track::State::Following);
        assert(System.GetState() == Systems::WORKING);

        // Real OLED state machine over a delayed, nonblocking bus mock.
        completed_frames.clear();
        wheels[1].rpm = -3.2f;
        run(1500);
        assert(completed_frames.size() >= 27);
        // M1 target is 40, feedback is 0: the tens digit must be the '0' glyph,
        // whose first column is 0x3E, not the '4' glyph's 0x18.
        assert(motors[0].GetTargetSpeed() == 40);
        assert(screen[4U * 6U] == 0x3EU);
        // Measured reverse motion must render a supported minus glyph on row M2.
        assert(screen[128U + 3U * 6U] == 0x08U);
        for (size_t index = 1; index < completed_frames.size(); ++index)
        {
            const uint32_t period = completed_frames[index] - completed_frames[index - 1U];
            assert(period >= 50 && period <= 60);
        }
        bus_error = true;
        run(1100);
        assert(System.GetState() == Systems::WORKING && motors[0].IsEnabled());
        bus_error = false;
        const size_t old_frames = completed_frames.size();
        run(1200);
        assert(completed_frames.size() > old_frames);
        white_mask = 0;
        run(1250);
        assert(System.GetState() == Systems::STOP);
        assert_stopped();
        white_mask = 0x18;
        run(30);
        assert_stopped(); // Reappearing line does not restart a stopped system.

        prepare();
        gray_valid = false;
        run(1);
        assert(System.GetState() == Systems::ERROR);
        assert_stopped();
        prepare();
        gray_missing = true;
        run(55);
        assert(System.GetState() == Systems::ERROR);
        assert_stopped();
        prepare();
        wheels[2].output_failure = true;
        run(20);
        assert(System.GetState() == Systems::ERROR);
        assert_stopped();
        prepare();
        wheels[1].feedback_lost = true;
        run(60);
        assert(System.GetState() == Systems::ERROR);
        assert_stopped();
        prepare();
        motors[3].Disable();
        run(5);
        assert(System.GetState() == Systems::ERROR);
        assert_stopped();
        prepare();
        test_tick += 60;
        gray.Update();
        DcMotor::ControlAllMotors();
        app.Control();
        assert(System.GetState() == Systems::ERROR);
        assert_stopped();
        System.Stop();
        bus_present = false;
        LineFollowApp missing_display(gray, motors[0], motors[1], motors[2], motors[3]);
        assert(missing_display.Init(config));
        assert_stopped();
        bus_present = true;
    }
} // namespace

extern "C" uint8_t BspOledBus_Init(void)
{
    return bus_present ? 1 : 0;
}

extern "C" uint8_t BspOledBus_Send(uint8_t control, const uint8_t *data, uint16_t size)
{
    assert(data && size > 0 && size <= 128);
    if (bus_error)
    {
        return 0;
    }
    bus_ready_ms = test_tick + 2U;
    if (control == 0x00 && size > 20U)
    {
        pages = 0;
    }
    if (control == 0x40)
    {
        assert(size == 128U && pages < 4U);
        memcpy(screen + pages * 128U, data, size);
        if (++pages == 4U)
        {
            completed_frames.push_back(test_tick);
            pages = 0;
        }
    }
    return 1;
}

extern "C" BspOledBus_Status BspOledBus_Poll(void)
{
    if (bus_error)
    {
        return BspOledBus_Error;
    }
    return (int32_t)(test_tick - bus_ready_ms) < 0 ? BspOledBus_Busy : BspOledBus_Idle;
}

int main()
{
    TestTrack();
    TestMixer();
    TestFormatting();
    TestApplication();
    puts("Line follow: centroid/PD, 512 masks, recovery/stop, four-wheel arbitration/slew, "
         "fault shutdown and measured-RPM OLED cadence/recovery passed.");
    return 0;
}
