#pragma once

#include "System.hpp"
#include "Track.hpp"
#include "SpeedMixer.hpp"
#include "MotorSpeedDisplay.hpp"

// Application lifecycle and all four motor writes remain owned by the framework task.
class LineFollowApp : public Application
{
public:
    struct Config
    {
        Track::Config track;
        float maximum_rpm = 60;
        float acceleration_rpm_s = 120;
        float deceleration_rpm_s = 240;
    };

    LineFollowApp(GraySensor &gray, DcMotor &m1, DcMotor &m2, DcMotor &m3, DcMotor &m4);
    bool Init(const Config &config);
    bool WatchPoint() override;
    App::Status GetStatus() override;
    bool RequestStart(); // Explicit request only; initialization never starts motion.
    void Control(); // Every 1 ms, computes and submits targets every 5 ms.
    void UpdateDisplay(); // Every 1 ms, measured RPM frames scheduled at 20 Hz.
    void StopMotors();
    const Track::Command &GetTrackCommand() const;
    SpeedMixer &GetSpeedMixer(); // Future APP policies share this single motor writer.

protected:
    void Start() override;
    void Update() override;

private:
    bool ApplySpeeds(const SpeedMixer::WheelSpeeds &speeds, float dt);
    GraySensor &gray_;
    DcMotor *motors_[4];
    Track track_;
    SpeedMixer mixer_;
    MotorSpeedDisplay display_;
    Config config_;
    float submitted_rpm_[4] = {};
    uint32_t last_control_ms_ = 0;
    bool initialized_ = false;
    bool running_ = false;
};
