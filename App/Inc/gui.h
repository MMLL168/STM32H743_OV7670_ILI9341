/**
 * @file    gui.h
 * @brief   On-screen GUI overlay for ILI9341 — status bar, text, shapes
 */
#ifndef GUI_H
#define GUI_H

#include <stdint.h>
#include <stdbool.h>

/* ---- Text rendering ---- */
void GUI_DrawChar(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg);
void GUI_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t fg, uint16_t bg);

/* ---- Overlay components ---- */
void GUI_DrawStatusBar(const char *milestone, float fps);
void GUI_DrawInfoBox(uint16_t y, const char *line1, const char *line2);
void GUI_ShowSplash(void);
void GUI_ShowTestResult(const char *title, const char *result, bool pass);
void GUI_ClearScreen(uint16_t color);

/* ---- Colour helpers ---- */
uint16_t GUI_RGB565(uint8_t r, uint8_t g, uint8_t b);

#endif /* GUI_H */
