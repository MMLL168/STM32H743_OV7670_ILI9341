/**
 * @file    servo_pwm.h
 * @brief   Servo PWM output on PB15 (TIM12_CH2) — 400 Hz, 1000~2000 us
 *
 * Maps a value 0~359 (degrees) to pulse width 1000~2000 us.
 * Used to relay red-object X position to B-G431B-ESC1 for motor angle control.
 */
#ifndef SERVO_PWM_H
#define SERVO_PWM_H

#include <stdint.h>

void Servo_Init(void);          /* Init TIM12 CH2 PWM on PB15, start output */
void Servo_SetAngle(uint16_t deg);  /* 0~359 -> pulse 1000~2000 us */
void Servo_SetPulseUs(uint16_t us); /* Direct pulse width in us (1000~2000) */

#endif /* SERVO_PWM_H */
