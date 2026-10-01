#pragma once

#include "dc_motor.hpp"
#include "oled_factory.hpp"

// Own the factory OLED from the same task as the motor control loop.
class MotorSpeedDisplay
{
public:
    MotorSpeedDisplay(DcMotor &m1, DcMotor &m2, DcMotor &m3, DcMotor &m4);
    bool Init();
    void Update();
    // Formats measured (never target) RPM without requiring printf floating point support.
    static bool FormatLine(uint8_t port, bool online, float rpm, char (&text)[22]);

private:
    DcMotor *motors_[4];
    FactoryOled display_;
    uint32_t last_refresh_ms_ = 0;
    bool initialized_ = false;
    bool refreshed_ = false;
};
