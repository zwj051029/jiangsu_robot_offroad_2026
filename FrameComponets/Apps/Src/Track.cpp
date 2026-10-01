#include "Track.hpp"
#include <math.h>

namespace
{
    bool Nonnegative(float value)
    {
        return isfinite(value) && value >= 0;
    }
} // namespace

bool Track::Init(const Config &config)
{
    initialized_ = false;
    Reset();
    if (!Nonnegative(config.base_rpm) || !Nonnegative(config.turn_rpm) ||
        !Nonnegative(config.gap_rpm) || !Nonnegative(config.search_rpm) ||
        !Nonnegative(config.search_diff_rpm) || !Nonnegative(config.maximum_diff_rpm) ||
        !Nonnegative(config.kp) || !Nonnegative(config.kd) ||
        !Nonnegative(config.error_filter_tau_s) || config.base_rpm <= 0 ||
        config.turn_rpm > config.base_rpm || config.gap_rpm > config.base_rpm ||
        config.search_rpm > config.base_rpm || config.search_diff_rpm > config.search_rpm ||
        config.maximum_diff_rpm > config.base_rpm || config.reacquire_samples == 0 ||
        config.sample_timeout_ms == 0 || config.sample_timeout_ms >= 0x80000000U ||
        config.gap_hold_ms >= config.loss_timeout_ms || config.loss_timeout_ms >= 0x80000000U)
    {
        return false;
    }
    config_ = config;
    pid_.Init(config.kp, 0, config.kd);
    pid_.SetLimit(0, config.maximum_diff_rpm, 0.2f);
    initialized_ = true;
    Reset();
    return true;
}

void Track::Reset()
{
    command_ = {};
    pid_.Reset();
    last_sample_ms_ = last_line_ms_ = filter_ms_ = 0;
    filtered_error_ = last_diff_rpm_ = last_direction_ = 0;
    reacquire_count_ = 0;
    sampled_ = seen_line_ = filter_valid_ = false;
}

bool Track::FindLine(const GraySensor::Sample &sample, float &error) const
{
    float weight = 0;
    float moment = 0;
    uint8_t active = 0;
    uint8_t groups = 0;
    bool previous = false;
    for (uint8_t index = 0; index < sample.channels; ++index)
    {
        const float value = config_.white_line ? sample.line[index] : 1.0f - sample.line[index];
        const bool on_line = value >= 0.5f;
        if (on_line)
        {
            ++active;
            if (!previous)
            {
                ++groups;
            }
            weight += value;
            moment += value * (2.0f * index / (sample.channels - 1U) - 1.0f);
        }
        previous = on_line;
    }
    // Wide marks and multiple separated lines require a route policy, not a fake centroid.
    if (active == 0 || active >= sample.channels - 1U || groups != 1)
    {
        return false;
    }
    error = moment / weight;
    return true;
}

void Track::HandleMissingLine(uint32_t now_ms)
{
    command_.valid = false;
    command_.base_rpm = command_.speed_diff_rpm = 0;
    if (!seen_line_)
    {
        command_.state = State::WaitingForLine;
        return;
    }
    const uint32_t missing_ms = now_ms - last_line_ms_;
    if (missing_ms >= config_.loss_timeout_ms)
    {
        command_.state = State::Lost;
        return;
    }
    command_.valid = true;
    if (missing_ms <= config_.gap_hold_ms)
    {
        command_.state = State::GapHold;
        command_.base_rpm = config_.gap_rpm;
        command_.speed_diff_rpm = last_diff_rpm_;
    }
    else
    {
        command_.state = State::Searching;
        command_.base_rpm = config_.search_rpm;
        // Alternate widening arcs if the last reliable line was centered.
        const bool alternate = ((missing_ms - config_.gap_hold_ms) / 250U) % 2U != 0U;
        const float direction = last_direction_ != 0 ? last_direction_ : (alternate ? -1.0f : 1.0f);
        command_.speed_diff_rpm = direction * config_.search_diff_rpm;
    }
}

void Track::Follow(float error, uint32_t sample_ms)
{
    const bool restarting = !filter_valid_ || command_.state != State::Following;
    const float dt = restarting ? 0.005f : (float)(sample_ms - filter_ms_) * 0.001f;
    if (restarting)
    {
        pid_.Reset();
        filtered_error_ = error;
    }
    else
    {
        const float alpha = dt / (config_.error_filter_tau_s + dt);
        filtered_error_ += alpha * (error - filtered_error_);
    }
    filter_ms_ = sample_ms;
    filter_valid_ = true;
    // Calc(error, 0) gives a positive difference for a line on the right.
    if (restarting)
    {
        pid_.kd_error = pid_.last_kd_error = filtered_error_;
    }
    pid_.ManualDt(dt);
    command_.speed_diff_rpm = pid_.Calc(filtered_error_, 0, config_.maximum_diff_rpm);
    command_.base_rpm =
        config_.base_rpm - (config_.base_rpm - config_.turn_rpm) * fabsf(filtered_error_);
    command_.line_error = filtered_error_;
    command_.state = State::Following;
    command_.valid = true;
    last_diff_rpm_ = command_.speed_diff_rpm;
    if (fabsf(filtered_error_) > 0.05f)
    {
        last_direction_ = filtered_error_ > 0 ? 1.0f : -1.0f;
    }
}

const Track::Command &Track::Update(const GraySensor::Sample &sample, uint32_t now_ms)
{
    if (!initialized_)
    {
        command_ = {};
        command_.state = State::Invalid;
        return command_;
    }
    // Lost is latched until Reset; an old command never restarts the vehicle.
    if (command_.state == State::Lost || command_.state == State::Invalid)
    {
        return command_;
    }
    bool valid = sample.valid && sample.channels >= 4 &&
                 sample.channels <= FRAME_GRAY_MAX_CHANNELS &&
                 (uint32_t)(now_ms - sample.timestamp_ms) <= config_.sample_timeout_ms;
    for (uint8_t index = 0; valid && index < sample.channels; ++index)
    {
        valid = isfinite(sample.line[index]) && sample.line[index] >= 0 && sample.line[index] <= 1;
    }
    if (!valid)
    {
        command_ = {};
        command_.state = State::Invalid;
        return command_;
    }
    if (sampled_ && sample.timestamp_ms == last_sample_ms_)
    {
        if (command_.state != State::Following)
        {
            HandleMissingLine(now_ms);
        }
        return command_;
    }
    sampled_ = true;
    last_sample_ms_ = sample.timestamp_ms;
    float error = 0;
    if (!FindLine(sample, error))
    {
        reacquire_count_ = 0;
        HandleMissingLine(now_ms);
        return command_;
    }
    // Check the deadline before accepting a late reacquisition.
    if (seen_line_ && (uint32_t)(now_ms - last_line_ms_) >= config_.loss_timeout_ms)
    {
        HandleMissingLine(now_ms);
        return command_;
    }
    if (command_.state != State::Following && ++reacquire_count_ < config_.reacquire_samples)
    {
        HandleMissingLine(now_ms);
        return command_;
    }
    seen_line_ = true;
    last_line_ms_ = sample.timestamp_ms;
    reacquire_count_ = 0;
    Follow(error, sample.timestamp_ms);
    return command_;
}

const Track::Command &Track::GetCommand() const
{
    return command_;
}
