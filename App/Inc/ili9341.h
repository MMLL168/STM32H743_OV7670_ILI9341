/**
 * @file    ili9341.h
 * @brief   ILI9341 LCD driver for SPI + DMA on STM32H7
 */
#ifndef ILI9341_H
#define ILI9341_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ---- Display geometry (landscape) ---- */
#define LCD_WIDTH   320
#define LCD_HEIGHT  240

/* ---- Common RGB565 colours ---- */
#define COLOR_BLACK   0x0000
#define COLOR_WHITE   0xFFFF
#define COLOR_RED     0xF800
#define COLOR_GREEN   0x07E0
#define COLOR_BLUE    0x001F
#define COLOR_YELLOW  0xFFE0
#define COLOR_CYAN    0x07FF
#define COLOR_MAGENTA 0xF81F
#define COLOR_GRAY    0x7BEF
#define COLOR_DGRAY   0x39E7

/* ---- GPIO helpers (directly use CubeMX defines) ---- */
#define LCD_CS_LOW()   HAL_GPIO_WritePin(LCD_CS_GPIO_Port,  LCD_CS_Pin,  GPIO_PIN_RESET)
#define LCD_CS_HIGH()  HAL_GPIO_WritePin(LCD_CS_GPIO_Port,  LCD_CS_Pin,  GPIO_PIN_SET)
#define LCD_DC_CMD()   HAL_GPIO_WritePin(LCD_DC_GPIO_Port,  LCD_DC_Pin,  GPIO_PIN_RESET)
#define LCD_DC_DATA()  HAL_GPIO_WritePin(LCD_DC_GPIO_Port,  LCD_DC_Pin,  GPIO_PIN_SET)
#define LCD_RST_LOW()  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET)
#define LCD_RST_HIGH() HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET)

/* ---- Public API ---- */
void     LCD_Init(SPI_HandleTypeDef *hspi);
void     LCD_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void     LCD_FillColor(uint16_t color);
void     LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void     LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color);

/* DMA-accelerated bulk transfer (non-blocking) */
void     LCD_SendData_DMA(const uint8_t *data, uint32_t len);
void     LCD_WaitDMA(void);
bool     LCD_IsDMABusy(void);

/* Blocking bulk transfer */
void     LCD_SendData(const uint8_t *data, uint32_t len);

/* Callback — call from HAL_SPI_TxCpltCallback */
void     LCD_DMA_TxCpltCallback(void);

#endif /* ILI9341_H */
