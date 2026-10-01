#pragma once

#include <stdint.h>

// APP command arbitration. Forward RPM only; no GPIO, PID configuration or hardware writes.
class SpeedMixer
{
public:
    enum class Source : uint8_t
    {
        None,
        Track,
        Follow,
        TurnAround,
        Overtake,
        Navigation
    };

    struct WheelSpeeds
    {
        float rpm[4] = {}; // M1 left front, M2 left rear, M3 right front, M4 right rear.
        Source source = Source::None;
    };

    bool Init(float maximum_rpm = 60, uint32_t timeout_ms = 50);
    bool SetTrackSpeed(float base_rpm, float speed_diff_rpm, uint32_t now_ms);
    bool SetFollowOffset(float offset_rpm, uint32_t now_ms);
    bool SetDirectSpeed(Source source, float left_rpm, float right_rpm, uint32_t now_ms);
    void ClearSource(Source source);
    void ClearAll();
    bool GetWheelSpeeds(uint32_t now_ms, WheelSpeeds &speeds) const;

private:
    struct Request
    {
        float first = 0;
        float second = 0;
        uint32_t timestamp_ms = 0;
        bool valid = false;
    };

    bool Fresh(Source source, uint32_t now_ms) const;
    Request requests_[6];
    float maximum_rpm_ = 60;
    uint32_t timeout_ms_ = 50;
    bool initialized_ = false;
};
