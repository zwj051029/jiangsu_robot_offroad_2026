#include "RtosCpp.hpp"
#include "System.hpp"
#include "StateCore.hpp"
#include "Action.hpp"
#include "MainFrame.hpp"
#include "LineFollowBoard.hpp"
#include "bsp_dwt.h"
#include "cmsis_os.h"

void MainInitCpp(void)
{
    System.Init();
    MainFrameCpp();
}

void FrameTickCpp(void)
{
    static uint8_t prescaler = 0;
    BspDwt_CntUpdate();
    Offroad.SampleSensors();
    if (++prescaler >= FRAME_SYSTEM_PERIOD_MS / FRAME_CONTROL_PERIOD_MS)
    {
        prescaler = 0;
        System.UpdateApplications();
        System.Run();
        if (System.GetState() == Systems::WORKING)
        {
            StateCore::GetInstance().Run();
            Action.ExecutorRun();
        }
    }
    if (LineFollowBoard::Enabled)
    {
        LineFollow.Control();
    }
    else
    {
        Offroad.Control();
    }
    DcMotor::ControlAllMotors();
    if (LineFollowBoard::Enabled)
    {
        LineFollow.UpdateDisplay();
    }
}

void RobotSystemCpp(void)
{
    uint32_t wake = osKernelSysTick();
    for (;;)
    {
        FrameTickCpp();
        osDelayUntil(&wake, FRAME_CONTROL_PERIOD_MS);
    }
}
