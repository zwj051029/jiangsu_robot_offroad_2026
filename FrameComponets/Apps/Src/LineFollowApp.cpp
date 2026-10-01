#include "LineFollowApp.hpp"
#include "stm32f1xx_hal.h"
#include <math.h>

LineFollowApp::LineFollowApp(GraySensor &gray, DcMotor &m1, DcMotor &m2, DcMotor &m3, DcMotor &m4)
    : Application("LineFollow"), gray_(gray), motors_{&m1, &m2, &m3, &m4}, display_(m1, m2, m3, m4)
{
}

bool LineFollowApp::Init(const Config &config)
{
    StopMotors();
    initialized_ = false;
    if (!isfinite(config.acceleration_rpm_s) || config.acceleration_rpm_s <= 0 ||
        !isfinite(config.deceleration_rpm_s) || config.deceleration_rpm_s <= 0 ||
        !isfinite(config.maximum_rpm) || config.track.base_rpm > config.maximum_rpm ||
        !track_.Init(config.track) || !mixer_.Init(config.maximum_rpm))
    {
        return false;
    }
    for (uint8_t index = 0; index < 4; ++index)
    {
        if (!motors_[index]->IsReady() || motors_[index]->GetMode() != DcMotor::Mode::Speed)
        {
            return false;
        }
        for (uint8_t previous = 0; previous < index; ++previous)
        {
            if (motors_[index] == motors_[previous])
            {
                return false;
            }
        }
    }
    config_ = config;
    // Missing OLED is diagnostic-only; it cannot block acquisition or motor stop.
    display_.Init();
    initialized_ = true;
    return true;
}

bool LineFollowApp::WatchPoint()
{
    if (!initialized_ || !gray_.IsReady() ||
        !gray_.HasFreshSample(HAL_GetTick(), config_.track.sample_timeout_ms))
    {
        return false;
    }
    for (DcMotor *motor : motors_)
    {
        if (!motor->IsReady() || !motor->IsOnline() || motor->GetMode() != DcMotor::Mode::Speed)
        {
            return false;
        }
    }
    return true;
}

App::Status LineFollowApp::GetStatus()
{
    if (System.GetState() == Systems::WORKING && !WatchPoint())
    {
        return App::Error;
    }
    return WatchPoint() ? App::Normal : App::Warning;
}

bool LineFollowApp::RequestStart()
{
    if (System.GetState() != Systems::READY || !WatchPoint())
    {
        return false;
    }
    System.system_start_to_work_flag = true;
    return true;
}

void LineFollowApp::Start()
{
    StopMotors();
}

void LineFollowApp::Update()
{
    // GetStatus participates in the existing self-check/health lifecycle.
    // Control is explicitly called at the base tick, never from another task.
}

void LineFollowApp::StopMotors()
{
    for (uint8_t index = 0; index < 4; ++index)
    {
        motors_[index]->Disable();
        submitted_rpm_[index] = 0;
    }
    track_.Reset();
    mixer_.ClearAll();
    running_ = false;
}

bool LineFollowApp::ApplySpeeds(const SpeedMixer::WheelSpeeds &speeds, float dt)
{
    bool all_enabled = true;
    bool any_enabled = false;
    for (DcMotor *motor : motors_)
    {
        all_enabled = all_enabled && motor->IsEnabled();
        any_enabled = any_enabled || motor->IsEnabled();
    }
    // Unexpected partial disable is a fault, not a reason to silently restart one wheel.
    if (any_enabled && !all_enabled)
    {
        return false;
    }
    for (uint8_t index = 0; index < 4; ++index)
    {
        if (!all_enabled && !motors_[index]->Enable())
        {
            return false;
        }
        const float change = speeds.rpm[index] - submitted_rpm_[index];
        const float limit =
            (change >= 0 ? config_.acceleration_rpm_s : config_.deceleration_rpm_s) * dt;
        const float increment = change > limit ? limit : (change < -limit ? -limit : change);
        submitted_rpm_[index] += increment;
        if (!motors_[index]->SetSpeed(submitted_rpm_[index]))
        {
            return false;
        }
    }
    return true;
}

void LineFollowApp::Control()
{
    if (!initialized_)
    {
        return;
    }
    if (System.GetState() != Systems::WORKING)
    {
        if (running_)
        {
            StopMotors();
        }
        return;
    }
    const uint32_t now = HAL_GetTick();
    if (!WatchPoint())
    {
        StopMotors();
        System.Stop(true);
        return;
    }
    if (!running_)
    {
        track_.Reset();
        mixer_.ClearAll();
        running_ = true;
        last_control_ms_ = now - FRAME_SYSTEM_PERIOD_MS;
    }
    const uint32_t elapsed = now - last_control_ms_;
    if (elapsed < FRAME_SYSTEM_PERIOD_MS)
    {
        return;
    }
    // A scheduling stall must stop instead of applying a large slew step.
    if (elapsed >= 50U)
    {
        StopMotors();
        System.Stop(true);
        return;
    }
    last_control_ms_ = now;
    const Track::Command &command = track_.Update(gray_.GetSample(), now);
    if (command.state == Track::State::Lost || command.state == Track::State::Invalid)
    {
        const bool invalid = command.state == Track::State::Invalid;
        StopMotors();
        System.Stop(invalid);
        return;
    }
    if (!command.valid)
    {
        mixer_.ClearAll();
        for (uint8_t index = 0; index < 4; ++index)
        {
            motors_[index]->Disable();
            submitted_rpm_[index] = 0;
        }
        return;
    }
    SpeedMixer::WheelSpeeds speeds;
    if (!mixer_.SetTrackSpeed(command.base_rpm, command.speed_diff_rpm, now) ||
        !mixer_.GetWheelSpeeds(now, speeds) || !ApplySpeeds(speeds, (float)elapsed * 0.001f))
    {
        StopMotors();
        System.Stop(true);
    }
}

void LineFollowApp::UpdateDisplay()
{
    display_.Update();
}

const Track::Command &LineFollowApp::GetTrackCommand() const
{
    return track_.GetCommand();
}

SpeedMixer &LineFollowApp::GetSpeedMixer()
{
    return mixer_;
}
