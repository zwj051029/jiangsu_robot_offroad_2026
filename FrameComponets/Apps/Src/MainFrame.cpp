#include "MainFrame.hpp"
#include "bsp_motor_board.h"
#include "MotorSpeedProfiles.hpp"
#include "LineFollowBoard.hpp"

MotorPwmDriver Motor1Driver, Motor2Driver, Motor3Driver, Motor4Driver;
DcMotor Motor1, Motor2, Motor3, Motor4;
MotorPwmDriver &LeftMotorDriver = Motor1Driver;
MotorPwmDriver &RightMotorDriver = Motor3Driver;
DcMotor &LeftMotor = Motor1;
DcMotor &RightMotor = Motor3;
DcMotor &LeftFrontMotor = Motor1;
DcMotor &LeftRearMotor = Motor2;
DcMotor &RightFrontMotor = Motor3;
DcMotor &RightRearMotor = Motor4;
Ultrasonic RangeSensor;
GraySensor GrayArray;
OffroadApp Offroad(LeftMotor, RightMotor, RangeSensor, GrayArray);
Led StatusLed;

namespace
{
    DcMotor *const motors[] = {&Motor1, &Motor2, &Motor3, &Motor4};
    MotorPwmDriver *const drivers[] = {&Motor1Driver, &Motor2Driver, &Motor3Driver, &Motor4Driver};

    // M1 左前、M2 左后、M3 右前、M4 右后。
    // 对照官方 car_tracking：左侧 PWM 反向；编码器左侧取正增量、右侧取负增量。
    const bool output_reverse[] = {true, true, false, false};
    const bool encoder_reverse[] = {false, false, true, true};

    bool BindHardware()
    {
        if (BspMotorBoard_Init() != HAL_OK)
        {
            return false;
        }
        for (uint8_t index = 0; index < 4; ++index)
        {
            BspMotorBoard_Port port = {};
            if (BspMotorBoard_GetPort(index + 1U, &port) != HAL_OK)
            {
                return false;
            }
            MotorPwmDriver::Config config;
            config.pwm_timer = port.pwm;
            config.in1_channel = port.in1_channel;
            config.in2_channel = port.in2_channel;
            config.encoder_timer = port.encoder;
            config.output_reverse = output_reverse[index];
            // 此表直接针对 TIM 原始计数，已经包含 M4 的实际 A/B 布线，不再额外异或。
            config.encoder_reverse = encoder_reverse[index];
            // 实测 PI 基于实际 PWM，不能叠加旧的固定死区补偿。
            config.deadzone_duty = 0;
            config.maximum_duty = MotorSpeedProfiles::duty_limit;
            if (!drivers[index]->Init(config))
            {
                return false;
            }
            if (!motors[index]->Init(drivers[index]->GetDriver()) ||
                !MotorSpeedProfiles::Apply(*motors[index], index + 1U, false))
            {
                return false;
            }
        }
        if (LineFollowBoard::Enabled)
        {
            LineFollowApp::Config config;
            if (!LineFollowBoard::InitGray(GrayArray) || !LineFollow.Init(config))
            {
                return false;
            }
        }
        // 已实测方向；四轮配置前进 PI 后仍保持禁能，只有应用可显式使能。
        // 传感器负责人在这里补充各自驱动绑定。
        // RangeSensor.Bind(...);
        // GrayArray.Bind(...);
        // StatusLed.Init(LED_GPIO_Port, LED_Pin, Actuator::Trigger_High);
        // System.monitor.Init(LogSink);
        // TIM6 是 HAL 时间基准，不能用于这些模块。
        return true;
    }

    void StopAll()
    {
        for (auto motor : motors)
        {
            motor->Disable();
        }
    }

    void SetIndicator(bool on)
    {
        if (on)
        {
            StatusLed.On();
        }
        else
        {
            StatusLed.Off();
        }
    }
} // namespace

void MainFrameCpp()
{
    System.BindStopHandler(StopAll);
    System.BindIndicator(SetIndicator);

    if (!BindHardware())
    {
        System.Stop(true);
        return;
    }
    Application &app = LineFollowBoard::Enabled ? static_cast<Application &>(LineFollow)
                                                : static_cast<Application &>(Offroad);
    if (!System.RegistApp(app))
    {
        System.Stop(true);
    }
}
