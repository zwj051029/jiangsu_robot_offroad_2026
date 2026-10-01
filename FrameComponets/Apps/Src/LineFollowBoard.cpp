#include "LineFollowBoard.hpp"
#include "MainFrame.hpp"
#include "gray_yahboom_8lp.hpp"
#include "stm32f1xx_hal.h"

LineFollowApp LineFollow(GrayArray, Motor1, Motor2, Motor3, Motor4);

namespace
{
    GrayYahboom8Lp gray_driver;
} // namespace

bool LineFollowBoard::InitGray(GraySensor &gray)
{
    // Confirmed wiring: x1 is vehicle left, x8 right; BLACK=low, WHITE=high.
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {};
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &gpio);
    const GrayYahboom8Lp::Channel channels[8] = {
        {GPIOC, GPIO_PIN_0, GPIO_PIN_RESET}, {GPIOC, GPIO_PIN_1, GPIO_PIN_RESET},
        {GPIOC, GPIO_PIN_2, GPIO_PIN_RESET}, {GPIOC, GPIO_PIN_3, GPIO_PIN_RESET},
        {GPIOA, GPIO_PIN_4, GPIO_PIN_RESET}, {GPIOA, GPIO_PIN_5, GPIO_PIN_RESET},
        {GPIOB, GPIO_PIN_0, GPIO_PIN_RESET}, {GPIOB, GPIO_PIN_1, GPIO_PIN_RESET}};
    GrayYahboom8Lp::Config config;
    if (!gray_driver.Init(channels, config))
    {
        return false;
    }
    gray.Bind(&gray_driver, GrayYahboom8Lp::Read);
    // No automatic confirmation: wait for warmup and the operator's KEY1 calibration.
    return true;
}

void LineFollowBoard::ConfirmCalibration(bool calibrated)
{
    gray_driver.ConfirmCalibration(calibrated);
}

void LineFollowBoard::NotifyGrayPowerOn()
{
    gray_driver.NotifyPowerOn();
}
