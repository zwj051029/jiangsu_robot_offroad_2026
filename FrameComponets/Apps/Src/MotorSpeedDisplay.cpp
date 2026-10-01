#include "MotorSpeedDisplay.hpp"
#include "stm32f1xx_hal.h"
#include <math.h>
#include <stdio.h>

MotorSpeedDisplay::MotorSpeedDisplay(DcMotor &m1, DcMotor &m2, DcMotor &m3, DcMotor &m4)
    : motors_{&m1, &m2, &m3, &m4}
{
}

bool MotorSpeedDisplay::Init()
{
    initialized_ = display_.Init();
    refreshed_ = false;
    return initialized_;
}

bool MotorSpeedDisplay::FormatLine(uint8_t port, bool online, float rpm, char (&text)[22])
{
    if (port < 1 || port > 4)
    {
        text[0] = '\0';
        return false;
    }
    if (!online || !isfinite(rpm))
    {
        snprintf(text, sizeof(text), "M%u ---.- RPM", (unsigned)port);
        return true;
    }
    if (fabsf(rpm) > 9999.0f)
    {
        snprintf(text, sizeof(text), "M%u RANGE RPM", (unsigned)port);
        return true;
    }
    const unsigned tenths = (unsigned)(fabsf(rpm) * 10.0f + 0.5f);
    // The retained factory font supports minus but has no plus glyph.
    const char *sign = tenths == 0 || rpm >= 0 ? "" : "-";
    snprintf(text, sizeof(text), "M%u %s%03u.%u RPM", (unsigned)port, sign, tenths / 10U,
             tenths % 10U);
    return true;
}

void MotorSpeedDisplay::Update()
{
    if (!initialized_)
    {
        return;
    }
    display_.Update(); // IRQ-driven I2C; never waits for a transfer here.
    const uint32_t now = HAL_GetTick();
    if (display_.IsBusy() || (refreshed_ && (uint32_t)(now - last_refresh_ms_) < 50U))
    {
        return;
    }
    for (uint8_t index = 0; index < 4; ++index)
    {
        char text[22];
        FormatLine(index + 1U, motors_[index]->IsOnline(), motors_[index]->GetFilteredRpm(), text);
        if (!display_.SetLine(index, text))
        {
            return;
        }
    }
    if (display_.Refresh())
    {
        last_refresh_ms_ = now;
        refreshed_ = true;
    }
}
