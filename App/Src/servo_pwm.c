/**
 * @file    servo_pwm.c
 * @brief   Servo PWM on PB15 (TIM12_CH2), 400 Hz, 1000~2000 us pulse
 *
 * Timer config (APB1 timer clock = 240 MHz):
 *   PSC  = 239  -> 1 MHz tick (1 us resolution)
 *   ARR  = 2499 -> period = 2500 us = 400 Hz
 *   CCR  = 1000..2000 -> pulse 1000..2000 us
 */
#include "servo_pwm.h"
#include "stm32h7xx_hal.h"

static TIM_HandleTypeDef htim12;

void Servo_Init(void)
{
    /* ---- Enable clocks ---- */
    __HAL_RCC_TIM12_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* ---- PB15 as TIM12_CH2 (AF2) ---- */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_15;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF2_TIM12;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* ---- TIM12 base ---- */
    htim12.Instance               = TIM12;
    htim12.Init.Prescaler         = 239;       /* 240 MHz / 240 = 1 MHz */
    htim12.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim12.Init.Period            = 2499;      /* 1 MHz / 2500 = 400 Hz */
    htim12.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim12);

    /* ---- PWM channel 2 ---- */
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 1500;     /* center: 1500 us */
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim12, &oc, TIM_CHANNEL_2);

    /* ---- Start PWM ---- */
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
}

void Servo_SetAngle(uint16_t deg)
{
    if (deg > 359) deg = 359;
    /* 0 deg -> 1000 us,  359 deg -> 2000 us */
    uint16_t pulse = (uint16_t)(1000U + (uint32_t)deg * 1000U / 359U);
    Servo_SetPulseUs(pulse);
}

void Servo_SetPulseUs(uint16_t us)
{
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, us);
}
