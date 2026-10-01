#include "SpeedMixer.hpp"
#include <math.h>

bool SpeedMixer::Init(float maximum_rpm, uint32_t timeout_ms)
{
    ClearAll();
    initialized_ = isfinite(maximum_rpm) && maximum_rpm > 0 && maximum_rpm <= 160 &&
                   timeout_ms > 0 && timeout_ms <= 100;
    if (initialized_)
    {
        maximum_rpm_ = maximum_rpm;
        timeout_ms_ = timeout_ms;
    }
    return initialized_;
}

bool SpeedMixer::SetTrackSpeed(float base_rpm, float speed_diff_rpm, uint32_t now_ms)
{
    Request &request = requests_[(uint8_t)Source::Track];
    request.valid = initialized_ && isfinite(base_rpm) && isfinite(speed_diff_rpm) && base_rpm >= 0;
    if (request.valid)
    {
        request.first = base_rpm;
        request.second = speed_diff_rpm;
        request.timestamp_ms = now_ms;
    }
    return request.valid;
}

bool SpeedMixer::SetFollowOffset(float offset_rpm, uint32_t now_ms)
{
    Request &request = requests_[(uint8_t)Source::Follow];
    request.valid = initialized_ && isfinite(offset_rpm);
    if (request.valid)
    {
        request.first = offset_rpm;
        request.timestamp_ms = now_ms;
    }
    return request.valid;
}

bool SpeedMixer::SetDirectSpeed(Source source, float left_rpm, float right_rpm, uint32_t now_ms)
{
    if (source < Source::TurnAround || source > Source::Navigation)
    {
        return false;
    }
    Request &request = requests_[(uint8_t)source];
    // Reverse targets require an explicit disabled/stationary PI profile handover elsewhere.
    request.valid = initialized_ && isfinite(left_rpm) && isfinite(right_rpm) && left_rpm >= 0 &&
                    right_rpm >= 0;
    if (request.valid)
    {
        request.first = left_rpm;
        request.second = right_rpm;
        request.timestamp_ms = now_ms;
    }
    return request.valid;
}

void SpeedMixer::ClearSource(Source source)
{
    if (source > Source::None && source <= Source::Navigation)
    {
        requests_[(uint8_t)source] = {};
    }
}

void SpeedMixer::ClearAll()
{
    for (Request &request : requests_)
    {
        request = {};
    }
}

bool SpeedMixer::Fresh(Source source, uint32_t now_ms) const
{
    const Request &request = requests_[(uint8_t)source];
    return request.valid && (uint32_t)(now_ms - request.timestamp_ms) < timeout_ms_;
}

bool SpeedMixer::GetWheelSpeeds(uint32_t now_ms, WheelSpeeds &speeds) const
{
    speeds = {};
    if (!initialized_)
    {
        return false;
    }
    float left = 0;
    float right = 0;
    for (int index = (int)Source::Navigation; index >= (int)Source::TurnAround; --index)
    {
        const Source source = (Source)index;
        if (Fresh(source, now_ms))
        {
            speeds.source = source;
            left = requests_[index].first;
            right = requests_[index].second;
            break;
        }
    }
    if (speeds.source == Source::None)
    {
        // Follow alone must never revive a cleared/stale track request.
        if (!Fresh(Source::Track, now_ms))
        {
            return false;
        }
        const Request &track = requests_[(uint8_t)Source::Track];
        const float offset =
            Fresh(Source::Follow, now_ms) ? requests_[(uint8_t)Source::Follow].first : 0;
        left = track.first + track.second + offset;
        right = track.first - track.second + offset;
        speeds.source = Fresh(Source::Follow, now_ms) ? Source::Follow : Source::Track;
    }
    if (!isfinite(left) || !isfinite(right))
    {
        speeds = {};
        return false;
    }
    // Preserve the left/right ratio when saturating the fastest forward wheel.
    left = left < 0 ? 0 : left;
    right = right < 0 ? 0 : right;
    const float largest = left > right ? left : right;
    const float scale = largest > maximum_rpm_ ? maximum_rpm_ / largest : 1.0f;
    speeds.rpm[0] = speeds.rpm[1] = left * scale;
    speeds.rpm[2] = speeds.rpm[3] = right * scale;
    return true;
}
