/**
 * @file    gui.c
 * @brief   On-screen GUI overlay — text, status bar, shapes
 */
#include "gui.h"
#include "ili9341.h"
#include "font5x7.h"
#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/*  Text rendering                                                     */
/* ------------------------------------------------------------------ */
/* Scale factor: each font pixel drawn as SCALE x SCALE */
#define FONT_SCALE      2
#define CHAR_W          (FONT_CELL_W * FONT_SCALE)  /* 12 */
#define CHAR_H          (FONT_CELL_H * FONT_SCALE)  /* 16 */

void GUI_DrawChar(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg)
{
    if (c < FONT_FIRST || c > FONT_LAST) c = ' ';
    const uint8_t *glyph = font5x7[c - FONT_FIRST];

    for (uint8_t col = 0; col < FONT_W; col++) {
        uint8_t line = glyph[col];
        for (uint8_t row = 0; row < FONT_H; row++) {
            uint16_t color = (line & (1 << row)) ? fg : bg;
            LCD_FillRect(x + col * FONT_SCALE, y + row * FONT_SCALE,
                         FONT_SCALE, FONT_SCALE, color);
        }
    }
    /* gap column */
    LCD_FillRect(x + FONT_W * FONT_SCALE, y, FONT_SCALE, FONT_H * FONT_SCALE, bg);
}

void GUI_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t fg, uint16_t bg)
{
    while (*str) {
        if (x + CHAR_W > LCD_WIDTH) break;
        GUI_DrawChar(x, y, *str++, fg, bg);
        x += CHAR_W;
    }
}

/* ------------------------------------------------------------------ */
/*  Status bar (top of screen, 12 pixels high)                         */
/* ------------------------------------------------------------------ */
#define BAR_H  20
#define BAR_BG COLOR_DGRAY

void GUI_DrawStatusBar(const char *milestone, float fps)
{
    /* Background bar */
    LCD_FillRect(0, 0, LCD_WIDTH, BAR_H, BAR_BG);

    /* Milestone name (left) */
    GUI_DrawString(4, 2, milestone, COLOR_WHITE, BAR_BG);

    /* FPS (right-aligned) */
    char fps_str[16];
    uint32_t fps10 = (fps <= 0.0f) ? 0U : (uint32_t)(fps * 10.0f + 0.5f);
    snprintf(fps_str, sizeof(fps_str), "%lu.%lu FPS",
             (unsigned long)(fps10 / 10U),
             (unsigned long)(fps10 % 10U));
    uint16_t len = (uint16_t)strlen(fps_str);
    uint16_t fx = LCD_WIDTH - (len * CHAR_W) - 4;
    GUI_DrawString(fx, 2, fps_str, COLOR_GREEN, BAR_BG);
}

/* ------------------------------------------------------------------ */
/*  Info box (bottom area)                                             */
/* ------------------------------------------------------------------ */
#define INFOBOX_H   40

void GUI_DrawInfoBox(uint16_t y, const char *line1, const char *line2)
{
    LCD_FillRect(0, y, LCD_WIDTH, INFOBOX_H, COLOR_BLACK);
    if (line1) GUI_DrawString(4, y + 2, line1, COLOR_CYAN, COLOR_BLACK);
    if (line2) GUI_DrawString(4, y + 20, line2, COLOR_YELLOW, COLOR_BLACK);
}

/* ------------------------------------------------------------------ */
/*  Splash screen                                                      */
/* ------------------------------------------------------------------ */
void GUI_ShowSplash(void)
{
    LCD_FillColor(COLOR_BLACK);

    GUI_DrawString(20,  40,  "STM32H743+OV7670", COLOR_CYAN,   COLOR_BLACK);
    GUI_DrawString(76,  65,  "ILI9341 LCD",      COLOR_WHITE,  COLOR_BLACK);
    GUI_DrawString(40, 100,  "Camera Preview",   COLOR_GREEN,  COLOR_BLACK);
    GUI_DrawString(44, 130,  "Initializing...",  COLOR_YELLOW, COLOR_BLACK);
    GUI_DrawString(52, 180,  "QVGA 320x240",    COLOR_GRAY,   COLOR_BLACK);
    GUI_DrawString(100, 205, "RGB565",           COLOR_GRAY,   COLOR_BLACK);
}

/* ------------------------------------------------------------------ */
/*  Test result display                                                */
/* ------------------------------------------------------------------ */
void GUI_ShowTestResult(const char *title, const char *result, bool pass)
{
    uint16_t rc = pass ? COLOR_GREEN : COLOR_RED;

    LCD_FillRect(0, 80, LCD_WIDTH, 80, COLOR_BLACK);
    GUI_DrawString(4, 85, title, COLOR_WHITE, COLOR_BLACK);
    GUI_DrawString(4, 110, result, rc, COLOR_BLACK);
    GUI_DrawString(4, 135, pass ? "[PASS]" : "[FAIL]", rc, COLOR_BLACK);
}

/* ------------------------------------------------------------------ */
/*  Helpers                                                            */
/* ------------------------------------------------------------------ */
void GUI_ClearScreen(uint16_t color)
{
    LCD_FillColor(color);
}

uint16_t GUI_RGB565(uint8_t r, uint8_t g, uint8_t b)
{
    return ((uint16_t)(r & 0xF8) << 8) |
           ((uint16_t)(g & 0xFC) << 3) |
           ((uint16_t)(b >> 3));
}
