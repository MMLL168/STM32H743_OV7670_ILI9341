/**
 * @file    app.h
 * @brief   Application controller — milestone state machine, frame pipeline
 */
#ifndef APP_H
#define APP_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ---- Milestone states ---- */
typedef enum {
    APP_STATE_SPLASH = 0,
    APP_STATE_F0_LIVE_PREVIEW,
    APP_STATE_D0_SYNC_DEBUG,
    APP_STATE_D1_COLORBAR_DEBUG,
    APP_STATE_M0_COLOR_TEST,
    APP_STATE_M1_SCCB_TEST,
    APP_STATE_M2_SINGLE_FRAME,
    APP_STATE_M3_LIVE_PREVIEW,
    APP_STATE_M4_DOUBLE_BUF,
    APP_STATE_M5_GRAYSCALE,
    APP_STATE_M6_RED_TRACKING,
} AppState_t;

/* ---- Display mode for M5 ---- */
typedef enum {
    DISPLAY_MODE_RGB = 0,
    DISPLAY_MODE_GRAYSCALE,
    DISPLAY_MODE_EDGE_DETECT,
    DISPLAY_MODE_COUNT
} DisplayMode_t;

/* ---- Runtime info (read-only from outside) ---- */
typedef struct {
    AppState_t    state;
    DisplayMode_t disp_mode;
    float         fps;
    uint32_t      frame_count;
    uint8_t       cam_pid;
    uint8_t       cam_ver;
    bool          cam_ok;
    bool          track_found;
    uint16_t      track_x;
    uint16_t      track_y;
    uint32_t      track_pixels;
} AppInfo_t;

/* ---- Public API ---- */
void            App_Init(void);
void            App_Run(void);          /* call in main while(1) */
const AppInfo_t *App_GetInfo(void);

/* ---- Callbacks (call from IT context) ---- */
void            App_DCMI_FrameCallback(void);
void            App_SPI_TxCpltCallback(void);
void            App_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif /* APP_H */
