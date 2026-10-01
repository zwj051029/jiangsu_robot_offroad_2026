#pragma once

#include "gray_sensor.hpp"
#include "pid.hpp"

// APP policy only: samples are calibrated WHITE=1, ordered vehicle left to right.
class Track
{
public:
    enum class State
    {
        Disabled,
        WaitingForLine,
        Following,
        GapHold,
        Searching,
        Lost,
        Invalid
    };

    struct Config
    {
        float base_rpm = 40;
        float turn_rpm = 28;
        float gap_rpm = 24;
        float search_rpm = 20;
        float search_diff_rpm = 18;
        float maximum_diff_rpm = 24;
        float kp = 28; // RPM per normalized line error.
        float kd = 0.25f; // RPM seconds per normalized line error; Ki stays zero.
        float error_filter_tau_s = 0.025f;
        bool white_line = true;
        uint8_t reacquire_samples = 3;
        uint32_t sample_timeout_ms = 50;
        uint32_t gap_hold_ms = 120;
        uint32_t loss_timeout_ms = 1200;
    };

    struct Command
    {
        float base_rpm = 0;
        float speed_diff_rpm = 0; // Positive: left faster, steer right.
        float line_error = 0; // Left negative, right positive, normalized [-1,1].
        State state = State::Disabled;
        bool valid = false;
    };

    bool Init(const Config &config);
    void Reset();
    const Command &Update(const GraySensor::Sample &sample, uint32_t now_ms);
    const Command &GetCommand() const;

private:
    bool FindLine(const GraySensor::Sample &sample, float &error) const;
    void HandleMissingLine(uint32_t now_ms);
    void Follow(float error, uint32_t sample_ms);

    Config config_;
    Command command_;
    PidGeneral pid_;
    uint32_t last_sample_ms_ = 0;
    uint32_t last_line_ms_ = 0;
    uint32_t filter_ms_ = 0;
    float filtered_error_ = 0;
    float last_diff_rpm_ = 0;
    float last_direction_ = 0;
    uint8_t reacquire_count_ = 0;
    bool initialized_ = false;
    bool sampled_ = false;
    bool seen_line_ = false;
    bool filter_valid_ = false;
};
