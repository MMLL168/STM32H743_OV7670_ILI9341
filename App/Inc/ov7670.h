/**
 * @file    ov7670.h
 * @brief   OV7670 camera driver — SCCB (I2C) + DCMI/DMA on STM32H7
 */
#ifndef OV7670_H
#define OV7670_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ---- I2C address (8-bit format used by HAL) ---- */
#define OV7670_ADDR         0x42

/* ---- Image parameters ---- */
#define CAM_WIDTH           320
#define CAM_HEIGHT          240
#define CAM_BPP             2           /* RGB565 = 2 bytes/pixel */
#define CAM_FRAME_SIZE      (CAM_WIDTH * CAM_HEIGHT * CAM_BPP)

/* ---- Key register addresses ---- */
#define OV7670_REG_PID      0x0A        /* Product ID (expect 0x76) */
#define OV7670_REG_VER      0x0B        /* Version    (expect 0x73) */

/* ---- GPIO helpers ---- */
#define OV7670_RST_LOW()    HAL_GPIO_WritePin(OV7670_RESET_GPIO_Port, OV7670_RESET_Pin, GPIO_PIN_RESET)
#define OV7670_RST_HIGH()   HAL_GPIO_WritePin(OV7670_RESET_GPIO_Port, OV7670_RESET_Pin, GPIO_PIN_SET)
#define OV7670_PWDN_ON()    HAL_GPIO_WritePin(OV7670_PWDN_GPIO_Port,  OV7670_PWDN_Pin,  GPIO_PIN_SET)
#define OV7670_PWDN_OFF()   HAL_GPIO_WritePin(OV7670_PWDN_GPIO_Port,  OV7670_PWDN_Pin,  GPIO_PIN_RESET)

/* ---- Public API ---- */
void     OV7670_Init(I2C_HandleTypeDef *hi2c, DCMI_HandleTypeDef *hdcmi);
bool     OV7670_ReadID(uint8_t *pid, uint8_t *ver);
void     OV7670_HardwareReset(void);
bool     OV7670_StartCapture(uint32_t *dst_buf);
void     OV7670_StopCapture(void);
bool     OV7670_SnapshotCapture(uint32_t *dst_buf);
bool     OV7670_SetColorBar(bool enable);
bool     OV7670_SetOutputByteSwap(bool enable);
bool     OV7670_SetPixelShift(uint8_t shift);
bool     OV7670_SetHorizontalWindowShift(int8_t steps);
bool     OV7670_SetSharpness(bool manual, uint8_t edge);
bool     OV7670_SetDenoise(bool manual, uint8_t threshold);

/* Low-level SCCB (I2C with retry) */
HAL_StatusTypeDef OV7670_WriteReg(uint8_t reg, uint8_t val);
HAL_StatusTypeDef OV7670_ReadReg(uint8_t reg, uint8_t *val);

#endif /* OV7670_H */
