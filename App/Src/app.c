/**
 * @file    app.c
 * @brief   Application controller — milestone state machine, frame pipeline
 *
 *  Frame pipeline (M3/M4/M5):
 *    1. DCMI DMA captures QQVGA 160x120 RGB565 into ping-pong frame buffer
 *    2. HAL_DCMI_FrameEventCallback sets frame_ready flag, swaps buffer
 *    3. Main loop: 2x nearest-neighbour scale → SPI DMA line-by-line → LCD
 *    4. GUI status bar overlay drawn on top
 */
#include "app.h"
#include "ili9341.h"
#include "ov7670.h"
#include "gui.h"
#include "servo_pwm.h"
#include "dcmi.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* ================================================================== */
/*  Frame buffers — placed in AXI SRAM via linker section              */
/* ================================================================== */
#define FRAME_BUF_SIZE  CAM_FRAME_SIZE
#define LINE_BUF_SIZE   (LCD_WIDTH * 2)
#define APP_DEBUG_BOOT          1
#define DEBUG_SYNC_TIMEOUT_MS   5000U
#define DEBUG_SYNC_MIN_FRAMES   3U
#define PREVIEW_OVERLAY_PERIOD_MS 1000U
#define RED_TRACK_MIN_PIXELS_DEFAULT  60U
#define RED_TRACK_BOX_COLOR   COLOR_YELLOW
#define RED_TRACK_CROSS_COLOR COLOR_CYAN
#define PREVIEW_HSHIFT_DEFAULT 0
#define PREVIEW_HSHIFT_MAX     4

__attribute__((section(".axi_sram"), aligned(32)))
static uint8_t frame_buf[2][FRAME_BUF_SIZE];

__attribute__((section(".axi_sram"), aligned(32)))
static uint8_t line_buf[2][LINE_BUF_SIZE];

/* ================================================================== */
/*  Private state                                                      */
/* ================================================================== */
static AppInfo_t info;

static volatile bool     frame_ready;
static volatile uint8_t  write_buf_idx;       /* buffer being written by DCMI */
static volatile uint32_t frame_counter;
static volatile uint32_t last_frame_callback_tick;
static uint8_t           uart3_rx_byte;
static volatile uint8_t  uart_rx_ring[64];
static volatile uint16_t uart_rx_head;
static volatile uint16_t uart_rx_tail;
static uint32_t          state_generation;
static bool              display_byte_swap;
static bool              sensor_byte_swap;
static uint8_t           sensor_pixel_shift;
static int8_t            sensor_hshift_steps;
static uint32_t          lcd_spi_prescaler;
static bool              sensor_manual_sharpness;
static uint8_t           sensor_sharpness;
static bool              sensor_manual_denoise;
static uint8_t           sensor_denoise;

/* Red tracking thresholds (runtime-adjustable) */
static uint8_t           red_r_min   = 16U;
static uint8_t           red_g_max   = 12U;
static uint8_t           red_b_max   = 12U;
static uint16_t          red_min_px  = RED_TRACK_MIN_PIXELS_DEFAULT;

/* Deferred OV7670 register write (avoid SPI/I2C conflict) */
static volatile bool     ov_write_pending;
static uint8_t           ov_write_reg;
static uint8_t           ov_write_val;

/* FPS calculation */
static uint32_t fps_tick_start;
static uint32_t fps_frame_count;
static uint32_t perf_tick_start;
static uint32_t perf_transfer_accum_ms;
static uint32_t perf_overlay_accum_ms;
static uint32_t perf_frames_accum;
static uint32_t perf_transfer_avg_ms;
static uint32_t perf_overlay_avg_ms;
static uint32_t perf_capture_last_tick;
static uint32_t perf_capture_accum_ms;
static uint32_t perf_capture_samples;
static uint32_t perf_capture_avg_ms;

/* State machine timing */
static uint32_t state_entry_tick;

typedef struct {
    uint32_t pclk_toggles;
    uint32_t vsync_toggles;
    uint32_t href_toggles;
    uint32_t href_high_samples;
    uint32_t pclk_rising_while_href;
    uint32_t data_samples;
    uint32_t data_transitions;
    uint8_t  data_or;
    uint8_t  data_and;
    uint8_t  first_data[16];
    uint8_t  first_count;
} CamSignalDiag_t;

typedef struct {
    uint32_t pck_polarity;
    uint32_t vs_polarity;
    uint32_t hs_polarity;
    uint32_t sr_start;
    uint32_t sr_end;
    uint32_t cr_end;
    uint32_t ris_end;
    uint32_t mis_end;
    uint32_t ier_end;
    uint32_t ndtr_start;
    uint32_t ndtr_end;
    uint32_t error_code;
    uint32_t frame_count_delta;
    uint32_t nonzero_bytes;
    bool     ndtr_moved;
    bool     fifo_seen;
} DcmiProbeResult_t;

typedef struct {
    bool     found;
    uint32_t count;
    uint32_t sum_x;
    uint32_t sum_y;
    uint16_t cx;
    uint16_t cy;
    uint16_t min_x;
    uint16_t min_y;
    uint16_t max_x;
    uint16_t max_y;
} RedTrack_t;

/* ================================================================== */
/*  Forward declarations                                               */
/* ================================================================== */
static void State_Splash(void);
static void State_F0_LivePreview(void);
static void State_D0_SyncDebug(void);
static void State_D1_ColorBarDebug(void);
static void State_M0_ColorTest(void);
static void State_M1_SCCBTest(void);
static void State_M2_SingleFrame(void);
static void State_M3_LivePreview(void);
static void State_M4_DoubleBuf(void);
static void State_M5_Grayscale(void);
static void State_M6_RedTracking(void);
static void TransferFrameToLCD(const uint8_t *fb);
static void TransferFrameToLCD_Grayscale(const uint8_t *fb);
static void TransferFrameToLCD_Edge(const uint8_t *fb);
static void LCD_SendFrameChunkedDMA(const uint8_t *data, uint32_t len);
static void ChangeState(AppState_t new_state);
static void UpdateFPS(void);
static void PerfReset(void);
static void PerfRecordFrame(uint32_t transfer_ms, uint32_t overlay_ms);
static bool ApplyLcdSpiPrescaler(uint32_t prescaler);
static void DumpFrameToUART(const uint8_t *fb);
static void ApplyOv7670SyncDefaults(void);
static void ApplyDcmiSync(uint32_t pck_polarity, uint32_t vs_polarity, uint32_t hs_polarity);
static void ConfigureDcmiCrop(void);
static void ApplyCameraTuning(void);
static void UART_Log(const char *fmt, ...);
static void PrintFieldControlHelp(void);
static void PollFieldControlCommand(void);
static void ProcessDeferredOvWrite(void);
static void LogWindowAndFreezeDiag(void);
static const char *AppStateName(AppState_t state);
static void StartUart3RxIT(void);
static bool StateEntered(uint32_t *stamp);
static uint8_t SampleCameraDataBus(void);
static void ProbeCameraSignals(uint32_t duration_ms, CamSignalDiag_t *diag);
static void LogCameraSignalDiag(const CamSignalDiag_t *diag);
static uint32_t GetDcmiNdtr(void);
static bool ProbeDcmiCapture(uint32_t pck_polarity, uint32_t vs_polarity,
                             uint32_t hs_polarity, uint32_t wait_ms,
                             DcmiProbeResult_t *result);
static void LogDcmiProbeResult(const char *label, const DcmiProbeResult_t *result);
static bool IsRedPixel565(uint16_t px);
static RedTrack_t TrackRedObject(const uint8_t *fb);
static void DrawTrackingRect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
                             uint16_t color);
static void DrawTrackingCrosshair(uint16_t x, uint16_t y, uint16_t color);
static void DrawTrackingOverlay(const RedTrack_t *track);
static uint32_t CountNonZeroBytes(const uint8_t *fb);

/* ================================================================== */
/*  Initialisation                                                     */
/* ================================================================== */
void App_Init(void)
{
    memset(&info, 0, sizeof(info));
    frame_ready   = false;
    write_buf_idx = 0;
    frame_counter = 0;
    last_frame_callback_tick = 0;
    uart3_rx_byte = 0;
    uart_rx_head = 0;
    uart_rx_tail = 0;
    state_generation = 0;
    display_byte_swap = false;
    sensor_byte_swap = false;
    sensor_pixel_shift = 0;
    sensor_hshift_steps = PREVIEW_HSHIFT_DEFAULT;
    lcd_spi_prescaler = hspi1.Init.BaudRatePrescaler;
    sensor_manual_sharpness = false;
    sensor_sharpness = 8U;
    sensor_manual_denoise = false;
    sensor_denoise = 8U;
    ApplyOv7670SyncDefaults();

    /* UART test message — verify VCP connection */
    const char *hello = "\r\n=== STM32H743 OV7670 UART OK (921600) ===\r\n";
    HAL_UART_Transmit(&huart3, (const uint8_t *)hello, strlen(hello), 100);

    char dbg[80];
    snprintf(dbg, sizeof(dbg), "DCMI SR=0x%08lX  PCLK=%s  VS=%s  HS=%s\r\n",
             (unsigned long)DCMI->SR,
             (hdcmi.Init.PCKPolarity == DCMI_PCKPOLARITY_RISING) ? "RISE" : "FALL",
             (hdcmi.Init.VSPolarity == DCMI_VSPOLARITY_LOW) ? "LOW" : "HIGH",
             (hdcmi.Init.HSPolarity == DCMI_HSPOLARITY_LOW) ? "LOW" : "HIGH");
    HAL_UART_Transmit(&huart3, (uint8_t *)dbg, strlen(dbg), 100);

    /* Enable DCMI NVIC (not generated by CubeMX) */
    HAL_NVIC_SetPriority(DCMI_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DCMI_IRQn);
    HAL_NVIC_SetPriority(USART3_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    StartUart3RxIT();

    /* Reconfigure DMA priorities per proposal */
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 1, 0);  /* DCMI DMA — highest */
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 3, 0);  /* SPI TX DMA */

    /* ---- LCD ---- */
    LCD_Init(&hspi1);

    /* ---- Servo PWM on PB15 (TIM12 CH2, 400 Hz) ---- */
    Servo_Init();

    /* Start from field-debug flow or normal milestone demo */
#if APP_DEBUG_BOOT
    PrintFieldControlHelp();
    ChangeState(APP_STATE_F0_LIVE_PREVIEW);
#else
    ChangeState(APP_STATE_SPLASH);
#endif
}

/* ================================================================== */
/*  Main loop                                                          */
/* ================================================================== */
void App_Run(void)
{
#if APP_DEBUG_BOOT
    PollFieldControlCommand();
#endif

    switch (info.state) {
        case APP_STATE_SPLASH:        State_Splash();       break;
        case APP_STATE_F0_LIVE_PREVIEW: State_F0_LivePreview(); break;
        case APP_STATE_D0_SYNC_DEBUG: State_D0_SyncDebug(); break;
        case APP_STATE_D1_COLORBAR_DEBUG: State_D1_ColorBarDebug(); break;
        case APP_STATE_M0_COLOR_TEST: State_M0_ColorTest(); break;
        case APP_STATE_M1_SCCB_TEST:  State_M1_SCCBTest();  break;
        case APP_STATE_M2_SINGLE_FRAME: State_M2_SingleFrame(); break;
        case APP_STATE_M3_LIVE_PREVIEW: State_M3_LivePreview(); break;
        case APP_STATE_M4_DOUBLE_BUF:   State_M4_DoubleBuf();   break;
        case APP_STATE_M5_GRAYSCALE:    State_M5_Grayscale();   break;
        case APP_STATE_M6_RED_TRACKING: State_M6_RedTracking(); break;
    }
}

/* ================================================================== */
/*  State implementations                                              */
/* ================================================================== */

static void State_F0_LivePreview(void)
{
    static uint32_t entry_stamp = 0;
    static bool freeze_warned = false;
    static uint32_t overlay_last_draw_tick = 0;
    static bool overlay_dirty = false;

    if (StateEntered(&entry_stamp)) {
        LCD_FillColor(COLOR_BLACK);
        GUI_DrawStatusBar("F0: Live Preview", 0.0f);
        GUI_DrawInfoBox(200,
                        "UART: p/c/t/d w/x",
                        "r/f b j/l q/e a/s");

        OV7670_StopCapture();
        OV7670_Init(&hi2c2, &hdcmi);
        info.cam_ok = OV7670_ReadID(&info.cam_pid, &info.cam_ver);
        ApplyCameraTuning();
        (void)OV7670_SetColorBar(false);

        frame_ready = false;
        write_buf_idx = 0;
        fps_tick_start = HAL_GetTick();
        fps_frame_count = 0;
        info.track_found = false;
        info.track_x = 0;
        info.track_y = 0;
        info.track_pixels = 0;
        freeze_warned = false;
        overlay_last_draw_tick = HAL_GetTick();
        overlay_dirty = false;
        PerfReset();
        OV7670_StartCapture((uint32_t *)frame_buf[write_buf_idx]);
        UART_Log("\r\n[F0] Live preview mode\r\n");
    }

    if (frame_ready) {
        frame_ready = false;
        bool had_freeze_warning = freeze_warned;
        uint32_t transfer_ms = 0U;
        uint32_t overlay_ms = 0U;
        uint32_t now;

        {
            uint8_t disp_idx = write_buf_idx ^ 1U;
            uint32_t t0 = HAL_GetTick();
            TransferFrameToLCD(frame_buf[disp_idx]);
            transfer_ms = HAL_GetTick() - t0;
        }

        fps_frame_count++;
        UpdateFPS();
        freeze_warned = false;
        now = HAL_GetTick();
        if (had_freeze_warning) {
            overlay_dirty = true;
        }
        if (now - overlay_last_draw_tick >= PREVIEW_OVERLAY_PERIOD_MS) {
            uint32_t t1 = HAL_GetTick();
            GUI_DrawStatusBar("F0: Live Preview", info.fps);
            overlay_ms = HAL_GetTick() - t1;
            overlay_last_draw_tick = HAL_GetTick();
        }
        if (overlay_dirty) {
            uint32_t t2 = HAL_GetTick();
            GUI_DrawInfoBox(200,
                            "UART: p/c/t/d w/x",
                            "r/f b j/l q/e a/s");
            overlay_ms += HAL_GetTick() - t2;
            overlay_dirty = false;
        }
        PerfRecordFrame(transfer_ms, overlay_ms);

        /* Process deferred OV7670 register write after all LCD SPI is done */
        ProcessDeferredOvWrite();
        info.frame_count++;
    }

    if (!freeze_warned &&
        frame_counter != 0U &&
        last_frame_callback_tick != 0U &&
        (HAL_GetTick() - last_frame_callback_tick > 1500U)) {
        char line2[48];

        snprintf(line2, sizeof(line2), "No callback %lums",
                 (unsigned long)(HAL_GetTick() - last_frame_callback_tick));
        GUI_DrawInfoBox(200,
                        "Preview freeze? press GUI w",
                        line2);
        UART_Log("[F0] WARN: no DCMI frame callback for %lu ms; preview may be frozen\r\n",
                 (unsigned long)(HAL_GetTick() - last_frame_callback_tick));
        freeze_warned = true;
        overlay_dirty = true;
    }
}

static void State_D0_SyncDebug(void)
{
    static uint8_t step = 0;
    static uint8_t frames_received = 0;
    static CamSignalDiag_t sig_diag;
    static uint32_t last_nonzero = 0;
    static uint32_t capture_start_tick = 0;
    static uint32_t entry_stamp = 0;

    if (StateEntered(&entry_stamp)) {
        step = 0;
        frames_received = 0;
        last_nonzero = 0;
        capture_start_tick = 0;
    }

    if (step == 0) {
        static const struct {
            const char *label;
            uint32_t pck;
            uint32_t vs;
            uint32_t hs;
        } candidates[] = {
            {"RISE/HIGH/HIGH", DCMI_PCKPOLARITY_RISING,  DCMI_VSPOLARITY_HIGH, DCMI_HSPOLARITY_HIGH},
            {"RISE/HIGH/LOW",  DCMI_PCKPOLARITY_RISING,  DCMI_VSPOLARITY_HIGH, DCMI_HSPOLARITY_LOW},
            {"RISE/LOW/HIGH",  DCMI_PCKPOLARITY_RISING,  DCMI_VSPOLARITY_LOW,  DCMI_HSPOLARITY_HIGH},
            {"RISE/LOW/LOW",   DCMI_PCKPOLARITY_RISING,  DCMI_VSPOLARITY_LOW,  DCMI_HSPOLARITY_LOW},
            {"FALL/HIGH/HIGH", DCMI_PCKPOLARITY_FALLING, DCMI_VSPOLARITY_HIGH, DCMI_HSPOLARITY_HIGH},
            {"FALL/HIGH/LOW",  DCMI_PCKPOLARITY_FALLING, DCMI_VSPOLARITY_HIGH, DCMI_HSPOLARITY_LOW},
            {"FALL/LOW/HIGH",  DCMI_PCKPOLARITY_FALLING, DCMI_VSPOLARITY_LOW,  DCMI_HSPOLARITY_HIGH},
            {"FALL/LOW/LOW",   DCMI_PCKPOLARITY_FALLING, DCMI_VSPOLARITY_LOW,  DCMI_HSPOLARITY_LOW},
        };
        DcmiProbeResult_t probe;
        const uint32_t orig_pck = hdcmi.Init.PCKPolarity;
        const uint32_t orig_vs  = hdcmi.Init.VSPolarity;
        const uint32_t orig_hs  = hdcmi.Init.HSPolarity;
        uint32_t best_nonzero = 0;
        int32_t best_idx = -1;
        char id_line[48];
        char sig_line[48];
        char verdict_line[48];

        LCD_FillColor(COLOR_BLACK);
        GUI_DrawStatusBar("DBG0: Sync Probe", 0.0f);
        GUI_DrawString(4, 30, "Boot: camera debug mode", COLOR_CYAN, COLOR_BLACK);
        GUI_DrawString(4, 52, "Init OV7670 + capture probe", COLOR_WHITE, COLOR_BLACK);

        OV7670_Init(&hi2c2, &hdcmi);
        info.cam_ok = OV7670_ReadID(&info.cam_pid, &info.cam_ver);
        ApplyCameraTuning();
        (void)OV7670_SetColorBar(false);

        ProbeCameraSignals(200, &sig_diag);
        LogCameraSignalDiag(&sig_diag);

        if (info.cam_ok && sig_diag.pclk_rising_while_href != 0U) {
            UART_Log("[DBG0] Sweeping DCMI sync polarities...\r\n");
            for (uint32_t i = 0; i < (sizeof(candidates) / sizeof(candidates[0])); i++) {
                if (ProbeDcmiCapture(candidates[i].pck, candidates[i].vs, candidates[i].hs, 120, &probe)) {
                    LogDcmiProbeResult(candidates[i].label, &probe);
                } else {
                    LogDcmiProbeResult(candidates[i].label, &probe);
                }

                if (probe.nonzero_bytes > best_nonzero) {
                    best_nonzero = probe.nonzero_bytes;
                    best_idx = (int32_t)i;
                }
            }

            HAL_DCMI_Stop(&hdcmi);
            HAL_DCMI_DeInit(&hdcmi);
            if (best_idx >= 0 && best_nonzero != 0U) {
                hdcmi.Init.PCKPolarity = candidates[best_idx].pck;
                hdcmi.Init.VSPolarity  = candidates[best_idx].vs;
                hdcmi.Init.HSPolarity  = candidates[best_idx].hs;
                UART_Log("[DBG0] Best DCMI polarity = %s (NZ=%lu)\r\n",
                         candidates[best_idx].label, (unsigned long)best_nonzero);
            } else {
                hdcmi.Init.PCKPolarity = orig_pck;
                hdcmi.Init.VSPolarity  = orig_vs;
                hdcmi.Init.HSPolarity  = orig_hs;
            }
            HAL_DCMI_Init(&hdcmi);
            ConfigureDcmiCrop();
        }

        snprintf(id_line, sizeof(id_line), "PID=%02X VER=%02X CAM=%s",
                 info.cam_pid, info.cam_ver, info.cam_ok ? "OK" : "FAIL");
        snprintf(sig_line, sizeof(sig_line), "PCLK=%lu VS=%lu HREF=%lu",
                 (unsigned long)sig_diag.pclk_toggles,
                 (unsigned long)sig_diag.vsync_toggles,
                 (unsigned long)sig_diag.href_toggles);

        if (!info.cam_ok) {
            snprintf(verdict_line, sizeof(verdict_line), "No OV7670 ID response");
        } else if (sig_diag.pclk_toggles == 0U) {
            snprintf(verdict_line, sizeof(verdict_line), "No PCLK on PA6");
        } else if (sig_diag.pclk_rising_while_href == 0U) {
            snprintf(verdict_line, sizeof(verdict_line), "No active HREF on PA4");
        } else if (best_nonzero == 0U) {
            snprintf(verdict_line, sizeof(verdict_line), "Try DCMI polarity sweep");
        } else {
            snprintf(verdict_line, sizeof(verdict_line), "Best NZ=%lu", (unsigned long)best_nonzero);
        }

        GUI_DrawString(4, 78, id_line, COLOR_GREEN, COLOR_BLACK);
        GUI_DrawString(4, 100, sig_line, COLOR_YELLOW, COLOR_BLACK);
        GUI_DrawInfoBox(200, verdict_line, "Auto -> OV7670 color bar");

        UART_Log("\r\n[DBG0] Booting field debug flow\r\n");
        UART_Log("[DBG0] %s\r\n", id_line);
        UART_Log("[DBG0] %s\r\n", sig_line);
        UART_Log("[DBG0] %s\r\n", verdict_line);

        frame_ready = false;
        write_buf_idx = 0;
        frames_received = 0;
        last_nonzero = 0;
        OV7670_StartCapture((uint32_t *)frame_buf[0]);
        capture_start_tick = HAL_GetTick();
        step = 1;
    }

    if (step == 1 && frame_ready) {
        char frame_line[48];
        uint8_t disp_idx;

        frame_ready = false;
        frames_received++;
        disp_idx = write_buf_idx ^ 1U;
        last_nonzero = CountNonZeroBytes(frame_buf[disp_idx]);

        snprintf(frame_line, sizeof(frame_line), "Frames=%u NonZero=%lu",
                 frames_received, (unsigned long)last_nonzero);
        LCD_FillRect(0, 150, LCD_WIDTH, 20, COLOR_BLACK);
        GUI_DrawString(4, 150, frame_line, COLOR_WHITE, COLOR_BLACK);
        GUI_DrawInfoBox(200, "Sync probe complete", frame_line);

        UART_Log("[DBG0] Frame #%u nonzero=%lu\r\n",
                 frames_received, (unsigned long)last_nonzero);

        if (frames_received >= DEBUG_SYNC_MIN_FRAMES) {
            OV7670_StopCapture();
            step = 0;
            ChangeState(APP_STATE_D1_COLORBAR_DEBUG);
        }
    }

    if (step == 1 && (HAL_GetTick() - capture_start_tick > DEBUG_SYNC_TIMEOUT_MS)) {
        char frame_line[48];

        OV7670_StopCapture();
        snprintf(frame_line, sizeof(frame_line), "Frames=%u NonZero=%lu",
                 frames_received, (unsigned long)last_nonzero);
        GUI_DrawInfoBox(200, "Sync timeout -> color bar", frame_line);
        UART_Log("[DBG0] Timeout. %s\r\n", frame_line);
        step = 0;
        ChangeState(APP_STATE_D1_COLORBAR_DEBUG);
    }
}

static void State_D1_ColorBarDebug(void)
{
    static uint32_t entry_stamp = 0;
    static uint32_t last_frame_tick = 0;
    static bool timeout_shown = false;

    if (StateEntered(&entry_stamp)) {
        LCD_FillColor(COLOR_BLACK);
        GUI_DrawStatusBar("DBG1: Color Bar", 0.0f);
        GUI_DrawInfoBox(200,
                        "Stable bars: sensor/bus path OK",
                        "UART p/c/t/d r/f b");

        OV7670_Init(&hi2c2, &hdcmi);
        info.cam_ok = OV7670_ReadID(&info.cam_pid, &info.cam_ver);
        ApplyCameraTuning();
        OV7670_StopCapture();
        if (OV7670_SetColorBar(true)) {
            UART_Log("\r\n[DBG1] OV7670 color bar enabled\r\n");
        } else {
            UART_Log("\r\n[DBG1] Failed to enable OV7670 color bar\r\n");
        }

        frame_ready = false;
        write_buf_idx = 0;
        fps_tick_start = HAL_GetTick();
        fps_frame_count = 0;
        last_frame_tick = HAL_GetTick();
        timeout_shown = false;
        OV7670_StartCapture((uint32_t *)frame_buf[write_buf_idx]);
    }

    if (frame_ready) {
        frame_ready = false;

        {
            uint8_t disp_idx = write_buf_idx ^ 1U;
            uint32_t nonzero = CountNonZeroBytes(frame_buf[disp_idx]);
            char line2[48];

            TransferFrameToLCD(frame_buf[disp_idx]);
            snprintf(line2, sizeof(line2), "FrameOK NZ=%lu", (unsigned long)nonzero);
            GUI_DrawInfoBox(200,
                            "Stable bars: sensor/bus path OK",
                            line2);
            UART_Log("[DBG1] Frame nonzero=%lu\r\n", (unsigned long)nonzero);
        }

        GUI_DrawStatusBar("DBG1: Color Bar", info.fps);
        last_frame_tick = HAL_GetTick();
        timeout_shown = false;

        fps_frame_count++;
        UpdateFPS();
        info.frame_count++;
    }

    if (!timeout_shown && (HAL_GetTick() - last_frame_tick > 1500U)) {
        GUI_DrawInfoBox(200,
                        "No frame in DBG1",
                        "Check VSYNC/HREF/DCMI callback");
        UART_Log("[DBG1] No frame callback for 1500 ms\r\n");
        timeout_shown = true;
    }
}

/* ---- Splash: 2 seconds ---- */
static void State_Splash(void)
{
    static bool shown = false;
    if (!shown) {
        GUI_ShowSplash();
        shown = true;
    }
    if (HAL_GetTick() - state_entry_tick > 2000) {
        shown = false;
        ChangeState(APP_STATE_M0_COLOR_TEST);
    }
}

/* ---- M0: LCD colour test — Red/Green/Blue 1.5s each, then White ---- */
static void State_M0_ColorTest(void)
{
    uint32_t elapsed = HAL_GetTick() - state_entry_tick;
    static uint8_t prev_phase = 0xFF;

    uint8_t phase;
    if      (elapsed < 1500) phase = 0;
    else if (elapsed < 3000) phase = 1;
    else if (elapsed < 4500) phase = 2;
    else if (elapsed < 6000) phase = 3;
    else {
        prev_phase = 0xFF;
        ChangeState(APP_STATE_M1_SCCB_TEST);
        return;
    }

    if (phase != prev_phase) {
        const uint16_t colors[] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_WHITE};
        const char *names[]     = {"RED", "GREEN", "BLUE", "WHITE"};
        LCD_FillColor(colors[phase]);
        GUI_DrawStatusBar("M0: Color Test", 0.0f);
        GUI_DrawString(120, 120, names[phase],
                       phase == 3 ? COLOR_BLACK : COLOR_WHITE,
                       colors[phase]);
        prev_phase = phase;
    }
}

/* ---- M1: SCCB communication — read OV7670 PID ---- */
static void State_M1_SCCBTest(void)
{
    static bool tested = false;
    if (!tested) {
        LCD_FillColor(COLOR_BLACK);
        GUI_DrawStatusBar("M1: SCCB Test", 0.0f);
        GUI_DrawString(4, 30, "Reading OV7670 ID...", COLOR_WHITE, COLOR_BLACK);

        OV7670_Init(&hi2c2, &hdcmi);
        info.cam_ok = OV7670_ReadID(&info.cam_pid, &info.cam_ver);

        char buf[40];
        snprintf(buf, sizeof(buf), "PID=0x%02X  VER=0x%02X", info.cam_pid, info.cam_ver);
        GUI_ShowTestResult("OV7670 SCCB", buf, info.cam_ok);

        if (info.cam_ok) {
            GUI_DrawString(4, 160, "Camera detected!", COLOR_GREEN, COLOR_BLACK);
        } else {
            GUI_DrawString(4, 160, "No camera or wrong ID", COLOR_RED, COLOR_BLACK);
        }

        /* Show register readback verification */
        extern volatile struct {
            uint8_t pid, ver, id_ok, init_done, reg_verify_ok, reg_verify_fail;
            uint8_t clkrc_rb, com7_rb, com15_rb, com3_rb, com14_rb, com10_rb;
            uint32_t sccb_errors, sccb_writes;
        } cam_dbg;

        char rbuf[40];
        snprintf(rbuf, sizeof(rbuf), "RegChk:%s Err:%lu/%lu",
                 cam_dbg.reg_verify_ok ? "OK" : "FAIL",
                 (unsigned long)cam_dbg.sccb_errors,
                 (unsigned long)cam_dbg.sccb_writes);
        GUI_DrawString(4, 185,  rbuf,
                       cam_dbg.reg_verify_ok ? COLOR_GREEN : COLOR_RED,
                       COLOR_BLACK);
        tested = true;
    }

    if (HAL_GetTick() - state_entry_tick > 3000) {
        tested = false;
        if (info.cam_ok) {
            ChangeState(APP_STATE_M2_SINGLE_FRAME);
        } else {
            /* Retry M1 */
            ChangeState(APP_STATE_M1_SCCB_TEST);
        }
    }
}

/* ---- M2: Single frame via continuous mode + UART dump ---- */
static void State_M2_SingleFrame(void)
{
    static uint8_t step = 0;
    static uint8_t frames_received = 0;

    if (step == 0) {
        static const struct {
            const char *label;
            uint32_t pck;
            uint32_t vs;
            uint32_t hs;
        } candidates[] = {
            {"RISE/HIGH/HIGH", DCMI_PCKPOLARITY_RISING,  DCMI_VSPOLARITY_HIGH, DCMI_HSPOLARITY_HIGH},
            {"RISE/HIGH/LOW",  DCMI_PCKPOLARITY_RISING,  DCMI_VSPOLARITY_HIGH, DCMI_HSPOLARITY_LOW},
            {"RISE/LOW/HIGH",  DCMI_PCKPOLARITY_RISING,  DCMI_VSPOLARITY_LOW,  DCMI_HSPOLARITY_HIGH},
            {"RISE/LOW/LOW",   DCMI_PCKPOLARITY_RISING,  DCMI_VSPOLARITY_LOW,  DCMI_HSPOLARITY_LOW},
            {"FALL/HIGH/HIGH", DCMI_PCKPOLARITY_FALLING, DCMI_VSPOLARITY_HIGH, DCMI_HSPOLARITY_HIGH},
            {"FALL/HIGH/LOW",  DCMI_PCKPOLARITY_FALLING, DCMI_VSPOLARITY_HIGH, DCMI_HSPOLARITY_LOW},
            {"FALL/LOW/HIGH",  DCMI_PCKPOLARITY_FALLING, DCMI_VSPOLARITY_LOW,  DCMI_HSPOLARITY_HIGH},
            {"FALL/LOW/LOW",   DCMI_PCKPOLARITY_FALLING, DCMI_VSPOLARITY_LOW,  DCMI_HSPOLARITY_LOW},
        };
        CamSignalDiag_t sig_diag;
        DcmiProbeResult_t probe;
        const uint32_t orig_pck = hdcmi.Init.PCKPolarity;
        const uint32_t orig_vs  = hdcmi.Init.VSPolarity;
        const uint32_t orig_hs  = hdcmi.Init.HSPolarity;
        bool found_probe = false;

        LCD_FillColor(COLOR_BLACK);
        GUI_DrawStatusBar("M2: Single Frame", 0.0f);
        GUI_DrawString(4, 30, "Starting capture...", COLOR_WHITE, COLOR_BLACK);

        UART_Log("\r\n[M2] Continuous capture, waiting for frames...\r\n");

        /* Check DCMI SR before start */
        UART_Log("[M2] DCMI SR=0x%08lX  CR=0x%08lX\r\n",
                 (unsigned long)DCMI->SR, (unsigned long)DCMI->CR);

        ProbeCameraSignals(200, &sig_diag);
        LogCameraSignalDiag(&sig_diag);

        if (sig_diag.pclk_toggles == 0) {
            UART_Log("[M2] Verdict: no PCLK toggles seen on PA6. This points to hardware/XCLK/PCLK wiring, not pixel data formatting.\r\n");
        } else if (sig_diag.pclk_rising_while_href == 0) {
            UART_Log("[M2] Verdict: PCLK exists, but PA4 never sees active HREF.\r\n");
            UART_Log("[M2]         Check the HREF wire/pinout first, then OV7670 sync-register setup.\r\n");
            UART_Log("[M2]         COM10=0x00 keeps HREF output mode, so PA4 should still show line activity.\r\n");
        } else {
            UART_Log("[M2] Raw bus looks alive. Sweeping DCMI polarities to separate FW config from wiring.\r\n");
            for (uint32_t i = 0; i < (sizeof(candidates) / sizeof(candidates[0])); i++) {
                if (ProbeDcmiCapture(candidates[i].pck, candidates[i].vs, candidates[i].hs, 150, &probe)) {
                    LogDcmiProbeResult(candidates[i].label, &probe);
                    UART_Log("[M2] Verdict: camera bus is active and DCMI starts capturing with %s.\r\n",
                             candidates[i].label);
                    found_probe = true;
                    break;
                }
                LogDcmiProbeResult(candidates[i].label, &probe);
            }

            if (!found_probe) {
                HAL_DCMI_Stop(&hdcmi);
                HAL_DCMI_DeInit(&hdcmi);
                hdcmi.Init.PCKPolarity = orig_pck;
                hdcmi.Init.VSPolarity  = orig_vs;
                hdcmi.Init.HSPolarity  = orig_hs;
                HAL_DCMI_Init(&hdcmi);
                ConfigureDcmiCrop();
                UART_Log("[M2] Verdict: raw sync/data activity exists, but none of the tried DCMI polarities captured bytes.\r\n");
                UART_Log("[M2]         That leans toward AF/pin mapping/electrical issues more than OV7670 register setup.\r\n");
            }
        }

        frame_ready = false;
        write_buf_idx = 0;
        frames_received = 0;
        OV7670_StartCapture((uint32_t *)frame_buf[0]);

        /* Check after start */
        UART_Log("[M2] After start: SR=0x%08lX  CR=0x%08lX\r\n",
                 (unsigned long)DCMI->SR, (unsigned long)DCMI->CR);

        /* Check DMA config */
        UART_Log("[M2] DMA NDTR=%lu  PAR=0x%08lX  M0AR=0x%08lX\r\n",
                 (unsigned long)((DMA_Stream_TypeDef *)hdcmi.DMA_Handle->Instance)->NDTR,
                 (unsigned long)((DMA_Stream_TypeDef *)hdcmi.DMA_Handle->Instance)->PAR,
                 (unsigned long)((DMA_Stream_TypeDef *)hdcmi.DMA_Handle->Instance)->M0AR);

        step = 1;
    }

    if (step == 1) {
        if (frame_ready) {
            frame_ready = false;
            frames_received++;

            char dbg[80];
            snprintf(dbg, sizeof(dbg), "[M2] Frame #%d! NDTR=%lu SR=0x%08lX\r\n",
                     frames_received,
                     (unsigned long)((DMA_Stream_TypeDef *)hdcmi.DMA_Handle->Instance)->NDTR,
                     (unsigned long)DCMI->SR);
            HAL_UART_Transmit(&huart3, (uint8_t *)dbg, strlen(dbg), 100);

            /* Read buffer that was just completed (the one NOT being written to) */
            uint8_t disp_idx = write_buf_idx ^ 1;

            /* Print first 32 bytes */
            snprintf(dbg, sizeof(dbg), "[M2] FB[%d][0..31]: ", disp_idx);
            HAL_UART_Transmit(&huart3, (uint8_t *)dbg, strlen(dbg), 100);
            for (int i = 0; i < 32; i++) {
                snprintf(dbg, sizeof(dbg), "%02X ", frame_buf[disp_idx][i]);
                HAL_UART_Transmit(&huart3, (uint8_t *)dbg, strlen(dbg), 50);
            }
            snprintf(dbg, sizeof(dbg), "\r\n");
            HAL_UART_Transmit(&huart3, (uint8_t *)dbg, strlen(dbg), 50);

            /* Count non-zero bytes in buffer */
            uint32_t nonzero = 0;
            for (uint32_t i = 0; i < FRAME_BUF_SIZE; i++) {
                if (frame_buf[disp_idx][i] != 0) nonzero++;
            }
            snprintf(dbg, sizeof(dbg), "[M2] Non-zero bytes: %lu / %lu\r\n",
                     (unsigned long)nonzero, (unsigned long)FRAME_BUF_SIZE);
            HAL_UART_Transmit(&huart3, (uint8_t *)dbg, strlen(dbg), 100);

            /* After 3rd frame, display + dump */
            if (frames_received >= 3) {
                OV7670_StopCapture();

                GUI_DrawString(4, 50, "Frame captured!", COLOR_GREEN, COLOR_BLACK);
                TransferFrameToLCD(frame_buf[disp_idx]);
                GUI_DrawStatusBar("M2: Captured!", 0.0f);

                snprintf(dbg, sizeof(dbg), "[M2] Dumping frame %d to UART...\r\n", disp_idx);
                HAL_UART_Transmit(&huart3, (uint8_t *)dbg, strlen(dbg), 100);
                DumpFrameToUART(frame_buf[disp_idx]);

                step = 2;
            }
        }

        /* Timeout after 10s */
        if (HAL_GetTick() - state_entry_tick > 10000 && frames_received == 0) {
            char dbg[80];
            snprintf(dbg, sizeof(dbg), "[M2] TIMEOUT! SR=0x%08lX  NDTR=%lu\r\n",
                     (unsigned long)DCMI->SR,
                     (unsigned long)((DMA_Stream_TypeDef *)hdcmi.DMA_Handle->Instance)->NDTR);
            HAL_UART_Transmit(&huart3, (uint8_t *)dbg, strlen(dbg), 100);

            OV7670_StopCapture();
            GUI_DrawString(4, 50, "Capture timeout!", COLOR_RED, COLOR_BLACK);
            step = 2;
        }
    }

    if (step == 2 && HAL_GetTick() - state_entry_tick > 20000) {
        step = 0;
        ChangeState(APP_STATE_M3_LIVE_PREVIEW);
    }
}

/* ---- M3: Continuous live preview ---- */
static void State_M3_LivePreview(void)
{
    static bool started = false;

    if (!started) {
        frame_ready = false;
        write_buf_idx = 0;
        fps_tick_start = HAL_GetTick();
        fps_frame_count = 0;
        OV7670_StartCapture((uint32_t *)frame_buf[write_buf_idx]);
        started = true;
    }

    if (frame_ready) {
        frame_ready = false;
        uint8_t disp_idx = write_buf_idx ^ 1;

        TransferFrameToLCD(frame_buf[disp_idx]);
        GUI_DrawStatusBar("M3: Live Preview", info.fps);

        fps_frame_count++;
        UpdateFPS();
        info.frame_count++;
    }

    /* Auto-advance to M4 after 10 seconds */
    if (HAL_GetTick() - state_entry_tick > 10000) {
        OV7670_StopCapture();
        started = false;
        ChangeState(APP_STATE_M4_DOUBLE_BUF);
    }
}

/* ---- M4: Double buffer optimized ---- */
static void State_M4_DoubleBuf(void)
{
    static bool started = false;

    if (!started) {
        frame_ready = false;
        write_buf_idx = 0;
        fps_tick_start = HAL_GetTick();
        fps_frame_count = 0;
        OV7670_StartCapture((uint32_t *)frame_buf[write_buf_idx]);
        started = true;
    }

    if (frame_ready) {
        frame_ready = false;
        uint8_t disp_idx = write_buf_idx ^ 1;

        TransferFrameToLCD(frame_buf[disp_idx]);

        char buf[48];
        snprintf(buf, sizeof(buf), "M4: DblBuf  Frm:%lu", (unsigned long)info.frame_count);
        GUI_DrawStatusBar(buf, info.fps);

        fps_frame_count++;
        UpdateFPS();
        info.frame_count++;
    }

    /* Auto-advance to M5 after 10 seconds */
    if (HAL_GetTick() - state_entry_tick > 10000) {
        OV7670_StopCapture();
        started = false;
        ChangeState(APP_STATE_M5_GRAYSCALE);
    }
}

/* ---- M5: Advanced — grayscale / edge detect (cycles every 8s) ---- */
static void State_M5_Grayscale(void)
{
    static bool started = false;

    if (!started) {
        frame_ready = false;
        write_buf_idx = 0;
        fps_tick_start = HAL_GetTick();
        fps_frame_count = 0;
        info.disp_mode = DISPLAY_MODE_RGB;
        OV7670_StartCapture((uint32_t *)frame_buf[write_buf_idx]);
        started = true;
    }

    /* Cycle display mode every 8s */
    uint32_t elapsed = HAL_GetTick() - state_entry_tick;
    info.disp_mode = (DisplayMode_t)((elapsed / 8000) % DISPLAY_MODE_COUNT);

    if (frame_ready) {
        frame_ready = false;
        uint8_t disp_idx = write_buf_idx ^ 1;

        switch (info.disp_mode) {
            case DISPLAY_MODE_GRAYSCALE:
                TransferFrameToLCD_Grayscale(frame_buf[disp_idx]);
                break;
            case DISPLAY_MODE_EDGE_DETECT:
                TransferFrameToLCD_Edge(frame_buf[disp_idx]);
                break;
            default:
                TransferFrameToLCD(frame_buf[disp_idx]);
                break;
        }

        const char *mode_names[] = {"RGB", "Grayscale", "Edge"};
        char buf[48];
        snprintf(buf, sizeof(buf), "M5: %s", mode_names[info.disp_mode]);
        GUI_DrawStatusBar(buf, info.fps);

        fps_frame_count++;
        UpdateFPS();
        info.frame_count++;

        ProcessDeferredOvWrite();
    }

    if (HAL_GetTick() - state_entry_tick > 24000) {
        OV7670_StopCapture();
        started = false;
        ChangeState(APP_STATE_M6_RED_TRACKING);
    }
}

/* ---- M6: RGB preview + red blob tracking overlay ---- */
static void State_M6_RedTracking(void)
{
    static uint32_t entry_stamp = 0;

    if (StateEntered(&entry_stamp)) {
        frame_ready = false;
        write_buf_idx = 0;
        fps_tick_start = HAL_GetTick();
        fps_frame_count = 0;
        info.track_found = false;
        info.track_x = 0;
        info.track_y = 0;
        info.track_pixels = 0;
        OV7670_Init(&hi2c2, &hdcmi);
        info.cam_ok = OV7670_ReadID(&info.cam_pid, &info.cam_ver);
        ApplyCameraTuning();
        (void)OV7670_SetColorBar(false);
        OV7670_StartCapture((uint32_t *)frame_buf[write_buf_idx]);
        UART_Log("\r\n[M6] Red tracking mode\r\n");
    }

    if (frame_ready) {
        RedTrack_t track;
        uint8_t disp_idx;
        char status[48];

        frame_ready = false;
        disp_idx = write_buf_idx ^ 1;
        track = TrackRedObject(frame_buf[disp_idx]);

        TransferFrameToLCD(frame_buf[disp_idx]);
        DrawTrackingOverlay(&track);

        info.track_found = track.found;
        info.track_x = track.cx;
        info.track_y = track.cy;
        info.track_pixels = track.count;

        if (track.found) {
            /* Map X (0..CAM_WIDTH-1) to angle (0..359) for servo */
            uint16_t angle = (uint16_t)((uint32_t)track.cx * 359U / (CAM_WIDTH - 1U));
            uint16_t pulse = (uint16_t)(1000U + (uint32_t)angle * 1000U / 359U);
            Servo_SetAngle(angle);

            snprintf(status, sizeof(status), "M6: (%u,%u) %udeg",
                     track.cx, track.cy, angle);
            UART_Log("[TRACK] %u %u %lu %u %u %u %u ang=%u pwm=%u\r\n",
                     track.cx, track.cy, (unsigned long)track.count,
                     track.min_x, track.min_y, track.max_x, track.max_y,
                     angle, pulse);
        } else {
            snprintf(status, sizeof(status), "M6: Red not found");
            UART_Log("[TRACK] none\r\n");
        }
        GUI_DrawStatusBar(status, info.fps);
        GUI_DrawInfoBox(200,
                        "UART: p/c/t/d w/x",
                        "r/f b j/l q/e a/s");

        fps_frame_count++;
        UpdateFPS();
        info.frame_count++;

        ProcessDeferredOvWrite();
    }
}

/* ================================================================== */
/*  Frame transfer: 2x nearest-neighbour scale → LCD                   */
/* ================================================================== */
static void TransferFrameToLCD(const uint8_t *fb)
{
    const uint32_t x_scale = LCD_WIDTH / CAM_WIDTH;
    const uint32_t y_scale = LCD_HEIGHT / CAM_HEIGHT;

    if (x_scale == 1U && y_scale == 1U && !display_byte_swap) {
        LCD_SetWindow(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);
        LCD_SendFrameChunkedDMA(fb, CAM_FRAME_SIZE);
        return;
    }

    LCD_SetWindow(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);

    for (uint32_t y = 0; y < CAM_HEIGHT; y++) {
        const uint16_t *src = (const uint16_t *)(fb + y * CAM_WIDTH * 2U);
        uint16_t *dst = (uint16_t *)line_buf[0];

        for (uint32_t x = 0; x < CAM_WIDTH; x++) {
            uint16_t px = src[x];

            if (display_byte_swap) {
                px = (uint16_t)((px << 8) | (px >> 8));
            }
            for (uint32_t i = 0; i < x_scale; i++) {
                dst[x * x_scale + i] = px;
            }
        }
        for (uint32_t i = 0; i < y_scale; i++) {
            LCD_SendData_DMA(line_buf[0], LINE_BUF_SIZE);
            LCD_WaitDMA();
        }
    }
}

/* ---- Grayscale conversion ---- */
static void TransferFrameToLCD_Grayscale(const uint8_t *fb)
{
    const uint32_t x_scale = LCD_WIDTH / CAM_WIDTH;
    const uint32_t y_scale = LCD_HEIGHT / CAM_HEIGHT;

    LCD_SetWindow(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);

    for (uint32_t y = 0; y < CAM_HEIGHT; y++) {
        const uint16_t *src = (const uint16_t *)(fb + y * CAM_WIDTH * 2U);
        uint16_t *dst = (uint16_t *)line_buf[0];

        for (uint32_t x = 0; x < CAM_WIDTH; x++) {
            uint16_t px = src[x];
            uint8_t r = (uint8_t)((px >> 11) & 0x1F);
            uint8_t g = (uint8_t)((px >> 5)  & 0x3F);
            uint8_t b = (uint8_t)(px & 0x1F);
            uint8_t gray5 = (uint8_t)((r * 9U + g * 9U + b * 3U) / 21U);
            uint8_t gray6 = (uint8_t)(gray5 << 1);
            uint16_t gpx = ((uint16_t)gray5 << 11) | ((uint16_t)gray6 << 5) | gray5;

            for (uint32_t i = 0; i < x_scale; i++) {
                dst[x * x_scale + i] = gpx;
            }
        }
        for (uint32_t i = 0; i < y_scale; i++) {
            LCD_SendData_DMA(line_buf[0], LINE_BUF_SIZE);
            LCD_WaitDMA();
        }
    }
}

/* ---- Simple Sobel edge detection ---- */
static void TransferFrameToLCD_Edge(const uint8_t *fb)
{
    const uint32_t x_scale = LCD_WIDTH / CAM_WIDTH;
    const uint32_t y_scale = LCD_HEIGHT / CAM_HEIGHT;

    LCD_SetWindow(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);

    for (uint32_t y = 0; y < CAM_HEIGHT; y++) {
        uint16_t *dst = (uint16_t *)line_buf[0];

        for (uint32_t x = 0; x < CAM_WIDTH; x++) {
            if (x == 0U || x == CAM_WIDTH - 1U || y == 0U || y == CAM_HEIGHT - 1U) {
                for (uint32_t i = 0; i < x_scale; i++) {
                    dst[x * x_scale + i] = COLOR_BLACK;
                }
                continue;
            }

            int32_t gx = 0;
            int32_t gy = 0;

            for (int dy = -1; dy <= 1; dy++) {
                uint32_t row_y = (uint32_t)((int32_t)y + dy);
                const uint16_t *row = (const uint16_t *)(fb + row_y * CAM_WIDTH * 2U);
                for (int dx = -1; dx <= 1; dx++) {
                    uint32_t col_x = (uint32_t)((int32_t)x + dx);
                    uint16_t px = row[col_x];
                    uint8_t r = (uint8_t)((px >> 11) & 0x1F);
                    uint8_t g = (uint8_t)((px >> 5)  & 0x3F);
                    uint8_t b = (uint8_t)(px & 0x1F);
                    int32_t lum = (int32_t)r * 2 + (int32_t)g + (int32_t)b * 2;
                    static const int8_t sx[3][3] = {{-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1}};
                    static const int8_t sy[3][3] = {{-1, -2, -1}, {0, 0, 0}, {1, 2, 1}};

                    gx += lum * sx[dy + 1][dx + 1];
                    gy += lum * sy[dy + 1][dx + 1];
                }
            }

            int32_t mag = (gx < 0 ? -gx : gx) + (gy < 0 ? -gy : gy);
            if (mag > 255) {
                mag = 255;
            }
            {
                uint8_t e5 = (uint8_t)(mag >> 3);
                uint8_t e6 = (uint8_t)(mag >> 2);
                uint16_t epx = ((uint16_t)e5 << 11) | ((uint16_t)e6 << 5) | e5;

                for (uint32_t i = 0; i < x_scale; i++) {
                    dst[x * x_scale + i] = epx;
                }
            }
        }
        for (uint32_t i = 0; i < y_scale; i++) {
            LCD_SendData_DMA(line_buf[0], LINE_BUF_SIZE);
            LCD_WaitDMA();
        }
    }
}

static void LCD_SendFrameChunkedDMA(const uint8_t *data, uint32_t len)
{
    const uint32_t max_chunk = 60000U;
    uint32_t remaining = len;
    const uint8_t *ptr = data;

    while (remaining > 0U) {
        uint32_t chunk = (remaining > max_chunk) ? max_chunk : remaining;
        LCD_SendData_DMA(ptr, chunk);
        LCD_WaitDMA();
        ptr += chunk;
        remaining -= chunk;
    }
}

static uint32_t CountNonZeroBytes(const uint8_t *fb)
{
    uint32_t nonzero = 0;

    for (uint32_t i = 0; i < FRAME_BUF_SIZE; i++) {
        if (fb[i] != 0U) {
            nonzero++;
        }
    }

    return nonzero;
}

/* ================================================================== */
/*  Helpers                                                            */
/* ================================================================== */
static void ApplyOv7670SyncDefaults(void)
{
    ApplyDcmiSync(DCMI_PCKPOLARITY_RISING,
                  DCMI_VSPOLARITY_HIGH,
                  DCMI_HSPOLARITY_LOW);
}

static void ConfigureDcmiCrop(void)
{
    /* Capture exactly the frame dimensions the firmware expects.  This
     * keeps RGB565 rows aligned even if the sensor emits guard pixels. */
    HAL_DCMI_ConfigCrop(&hdcmi, 0, 0,
                        (CAM_WIDTH * 2) - 1,   /* CAPCNT: bytes/line - 1 */
                        CAM_HEIGHT - 1);        /* VLINE:  lines - 1     */
    HAL_DCMI_EnableCrop(&hdcmi);
}

static void ApplyDcmiSync(uint32_t pck_polarity, uint32_t vs_polarity, uint32_t hs_polarity)
{
    HAL_DCMI_Stop(&hdcmi);
    HAL_DCMI_DeInit(&hdcmi);

    hdcmi.Init.PCKPolarity = pck_polarity;
    hdcmi.Init.VSPolarity  = vs_polarity;
    hdcmi.Init.HSPolarity  = hs_polarity;

    if (HAL_DCMI_Init(&hdcmi) != HAL_OK) {
        Error_Handler();
    }
    ConfigureDcmiCrop();
}

static void ApplyCameraTuning(void)
{
    if (!OV7670_SetOutputByteSwap(sensor_byte_swap)) {
        UART_Log("[CTRL] WARN: failed to set COM3[6] swap=%u\r\n", sensor_byte_swap ? 1U : 0U);
    }
    if (!OV7670_SetPixelShift(sensor_pixel_shift)) {
        UART_Log("[CTRL] WARN: failed to set PSHFT=%u\r\n", sensor_pixel_shift);
    }
    if (!OV7670_SetHorizontalWindowShift(sensor_hshift_steps)) {
        UART_Log("[CTRL] WARN: failed to set HSHIFT=%d\r\n", (int)sensor_hshift_steps);
    }
    if (!OV7670_SetSharpness(sensor_manual_sharpness, sensor_sharpness)) {
        UART_Log("[CTRL] WARN: failed to set sharpness manual=%u edge=%u\r\n",
                 sensor_manual_sharpness ? 1U : 0U, (unsigned)sensor_sharpness);
    }
    if (!OV7670_SetDenoise(sensor_manual_denoise, sensor_denoise)) {
        UART_Log("[CTRL] WARN: failed to set denoise manual=%u thr=%u\r\n",
                 sensor_manual_denoise ? 1U : 0U, (unsigned)sensor_denoise);
    }
}

static bool ApplyLcdSpiPrescaler(uint32_t prescaler)
{
    LCD_WaitDMA();
    HAL_SPI_DeInit(&hspi1);
    hspi1.Init.BaudRatePrescaler = prescaler;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        return false;
    }
    lcd_spi_prescaler = prescaler;
    return true;
}

static void UART_Log(const char *fmt, ...)
{
    char buf[192];
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n <= 0) {
        return;
    }

    if (n >= (int)sizeof(buf)) {
        n = (int)sizeof(buf) - 1;
    }

    HAL_UART_Transmit(&huart3, (uint8_t *)buf, (uint16_t)n, 1000);
}

static void PrintFieldControlHelp(void)
{
    UART_Log("\r\n[CTRL] Field controls over USART3 RX:\r\n");
    UART_Log("[CTRL]   p = live preview\r\n");
    UART_Log("[CTRL]   c = OV7670 color bar\r\n");
    UART_Log("[CTRL]   t = red tracking\r\n");
    UART_Log("[CTRL]   d = sync diagnostic\r\n");
    UART_Log("[CTRL]   w = window/freeze snapshot\r\n");
    UART_Log("[CTRL]   x = reset preview baseline\r\n");
    UART_Log("[CTRL]   r = PCLK rising edge\r\n");
    UART_Log("[CTRL]   f = PCLK falling edge\r\n");
    UART_Log("[CTRL]   b = toggle LCD byte swap\r\n");
    UART_Log("[CTRL]   j = horizontal window left -8 px\r\n");
    UART_Log("[CTRL]   l = horizontal window right +8 px\r\n");
    UART_Log("[CTRL]   q/e = sharpness -/+ (manual)\r\n");
    UART_Log("[CTRL]   a/s = denoise -/+ (manual)\r\n");
    UART_Log("[CTRL]   8 = LCD SPI prescaler /8\r\n");
    UART_Log("[CTRL]   4 = LCD SPI prescaler /4\r\n");
    UART_Log("[CTRL]   m = toggle OV7670 COM3[6] byte swap\r\n");
    UART_Log("[CTRL]   z = sweep baseline (COM3[6]=ON, PSHFT=0)\r\n");
    UART_Log("[CTRL]   [ = PSHFT -1\r\n");
    UART_Log("[CTRL]   ] = PSHFT +1\r\n");
    UART_Log("[CTRL]   u = dump current frame to UART\r\n");
    UART_Log("[CTRL]   h/? = help\r\n");
}

static void PollFieldControlCommand(void)
{
    uint8_t ch = 0;

    while (uart_rx_head != uart_rx_tail) {
        ch = uart_rx_ring[uart_rx_tail];
        uart_rx_tail = (uint16_t)((uart_rx_tail + 1U) % (uint16_t)sizeof(uart_rx_ring));

        if (ch >= 'A' && ch <= 'Z') {
            ch = (uint8_t)(ch - 'A' + 'a');
        }

        switch (ch) {
            case 'p':
                if (info.state != APP_STATE_F0_LIVE_PREVIEW) {
                    OV7670_StopCapture();
                    UART_Log("[CTRL] -> live preview\r\n");
                    ChangeState(APP_STATE_F0_LIVE_PREVIEW);
                }
                break;

            case 'c':
                if (info.state != APP_STATE_D1_COLORBAR_DEBUG) {
                    OV7670_StopCapture();
                    UART_Log("[CTRL] -> color bar\r\n");
                    ChangeState(APP_STATE_D1_COLORBAR_DEBUG);
                }
                break;

            case 't':
                if (info.state != APP_STATE_M6_RED_TRACKING) {
                    OV7670_StopCapture();
                    UART_Log("[CTRL] -> red tracking\r\n");
                    ChangeState(APP_STATE_M6_RED_TRACKING);
                }
                break;

            case 'd':
                OV7670_StopCapture();
                UART_Log("[CTRL] -> sync diagnostic\r\n");
                ChangeState(APP_STATE_D0_SYNC_DEBUG);
                break;

            case 'w':
                UART_Log("[CTRL] -> window/freeze snapshot\r\n");
                LogWindowAndFreezeDiag();
                break;

            case 'x':
                display_byte_swap = false;
                sensor_byte_swap = false;
                sensor_pixel_shift = 0U;
                sensor_hshift_steps = PREVIEW_HSHIFT_DEFAULT;
                sensor_manual_sharpness = false;
                sensor_manual_denoise = false;
                OV7670_StopCapture();
                ApplyOv7670SyncDefaults();
                UART_Log("[CTRL] -> reset preview baseline (disp_swap=OFF sensor_swap=OFF PSHFT=0 HSHIFT=%d sharp=auto denoise=auto PCLK=RISE)\r\n",
                         PREVIEW_HSHIFT_DEFAULT);
                ChangeState(info.state);
                break;

            case 'h':
            case '?':
                PrintFieldControlHelp();
                break;

            case 'r':
                OV7670_StopCapture();
                ApplyDcmiSync(DCMI_PCKPOLARITY_RISING,
                              hdcmi.Init.VSPolarity,
                              hdcmi.Init.HSPolarity);
                UART_Log("[CTRL] -> PCLK rising\r\n");
                ChangeState(info.state);
                break;

            case 'f':
                OV7670_StopCapture();
                ApplyDcmiSync(DCMI_PCKPOLARITY_FALLING,
                              hdcmi.Init.VSPolarity,
                              hdcmi.Init.HSPolarity);
                UART_Log("[CTRL] -> PCLK falling\r\n");
                ChangeState(info.state);
                break;

            case 'b':
                display_byte_swap = !display_byte_swap;
                UART_Log("[CTRL] -> byte swap %s\r\n",
                         display_byte_swap ? "ON" : "OFF");
                break;

            case 'j':
                if (sensor_hshift_steps > -16) {
                    sensor_hshift_steps--;
                }
                OV7670_StopCapture();
                UART_Log("[CTRL] -> HSHIFT=%d\r\n", (int)sensor_hshift_steps);
                ChangeState(info.state);
                break;

            case 'l':
                if (sensor_hshift_steps < PREVIEW_HSHIFT_MAX) {
                    sensor_hshift_steps++;
                }
                OV7670_StopCapture();
                UART_Log("[CTRL] -> HSHIFT=%d\r\n", (int)sensor_hshift_steps);
                ChangeState(info.state);
                break;

            case 'q':
                sensor_manual_sharpness = true;
                if (sensor_sharpness > 0U) {
                    sensor_sharpness--;
                }
                OV7670_StopCapture();
                UART_Log("[CTRL] -> sharpness=%u (manual)\r\n", (unsigned)sensor_sharpness);
                ChangeState(info.state);
                break;

            case 'e':
                sensor_manual_sharpness = true;
                if (sensor_sharpness < 31U) {
                    sensor_sharpness++;
                }
                OV7670_StopCapture();
                UART_Log("[CTRL] -> sharpness=%u (manual)\r\n", (unsigned)sensor_sharpness);
                ChangeState(info.state);
                break;

            case 'a':
                sensor_manual_denoise = true;
                if (sensor_denoise > 0U) {
                    sensor_denoise--;
                }
                OV7670_StopCapture();
                UART_Log("[CTRL] -> denoise=%u (manual)\r\n", (unsigned)sensor_denoise);
                ChangeState(info.state);
                break;

            case 's':
                sensor_manual_denoise = true;
                if (sensor_denoise < 255U) {
                    sensor_denoise++;
                }
                OV7670_StopCapture();
                UART_Log("[CTRL] -> denoise=%u (manual)\r\n", (unsigned)sensor_denoise);
                ChangeState(info.state);
                break;

            case '8':
                OV7670_StopCapture();
                if (ApplyLcdSpiPrescaler(SPI_BAUDRATEPRESCALER_8)) {
                    UART_Log("[CTRL] -> LCD SPI /8\r\n");
                    ChangeState(info.state);
                } else {
                    UART_Log("[CTRL] WARN: failed to set LCD SPI /8\r\n");
                }
                break;

            case '4':
                OV7670_StopCapture();
                if (ApplyLcdSpiPrescaler(SPI_BAUDRATEPRESCALER_4)) {
                    UART_Log("[CTRL] -> LCD SPI /4\r\n");
                    ChangeState(info.state);
                } else {
                    UART_Log("[CTRL] WARN: failed to set LCD SPI /4\r\n");
                }
                break;

            case 'm':
                sensor_byte_swap = !sensor_byte_swap;
                OV7670_StopCapture();
                UART_Log("[CTRL] -> OV7670 COM3[6] swap %s\r\n",
                         sensor_byte_swap ? "ON" : "OFF");
                ChangeState(info.state);
                break;

            case 'z':
                /* NOTE: COM3[6] byte swap does not work on many OV7670 clones.
                 * The raw byte order (big-endian) is already correct for
                 * ILI9341 SPI and for the Python viewer (BE mode). */
                sensor_byte_swap = false;
                sensor_pixel_shift = 0U;
                sensor_hshift_steps = PREVIEW_HSHIFT_DEFAULT;
                sensor_manual_sharpness = false;
                sensor_manual_denoise = false;
                OV7670_StopCapture();
                UART_Log("[CTRL] -> reset baseline COM3[6]=OFF PSHFT=0 HSHIFT=%d\r\n",
                         PREVIEW_HSHIFT_DEFAULT);
                ChangeState(info.state);
                break;

            case '[':
                if (sensor_pixel_shift > 0U) {
                    sensor_pixel_shift--;
                }
                OV7670_StopCapture();
                UART_Log("[CTRL] -> PSHFT=%u\r\n", sensor_pixel_shift);
                ChangeState(info.state);
                break;

            case ']':
                if (sensor_pixel_shift < 255U) {
                    sensor_pixel_shift++;
                }
                OV7670_StopCapture();
                UART_Log("[CTRL] -> PSHFT=%u\r\n", sensor_pixel_shift);
                ChangeState(info.state);
                break;

            case 'u':
                if (frame_counter == 0U) {
                    UART_Log("[CTRL] No frame captured yet\r\n");
                } else {
                    uint8_t disp_idx = write_buf_idx ^ 1U;
                    OV7670_StopCapture();
                    UART_Log("[CTRL] -> dumping frame buffer %u\r\n", disp_idx);
                    DumpFrameToUART(frame_buf[disp_idx]);
                    ChangeState(info.state);
                }
                break;

            case 'T': {
                /* Parse "T<r_min>,<g_max>,<b_max>,<min_px>\n" from ring buffer */
                char tbuf[32];
                uint8_t ti = 0;
                while (uart_rx_tail != uart_rx_head && ti < sizeof(tbuf) - 1U) {
                    char tc = (char)uart_rx_ring[uart_rx_tail];
                    uart_rx_tail = (uint16_t)((uart_rx_tail + 1U) % sizeof(uart_rx_ring));
                    if (tc == '\n') break;
                    tbuf[ti++] = tc;
                }
                tbuf[ti] = '\0';
                unsigned r_v, g_v, b_v, n_v;
                if (sscanf(tbuf, "%u,%u,%u,%u", &r_v, &g_v, &b_v, &n_v) == 4) {
                    red_r_min  = (uint8_t)(r_v > 31 ? 31 : r_v);
                    red_g_max  = (uint8_t)(g_v > 31 ? 31 : g_v);
                    red_b_max  = (uint8_t)(b_v > 31 ? 31 : b_v);
                    red_min_px = (uint16_t)(n_v > 9999 ? 9999 : n_v);
                    UART_Log("[CTRL] Track thresholds: R>=%u G<=%u B<=%u min_px=%u\r\n",
                             red_r_min, red_g_max, red_b_max, red_min_px);
                } else {
                    UART_Log("[CTRL] Bad threshold format (expect T<r>,<g>,<b>,<n>)\r\n");
                }
                break;
            }

            case 'W': {
                /* Generic OV7670 register write: "W<reg_hex>,<val_hex>\n"
                 * Deferred to frame gap to avoid SPI/I2C bus conflict. */
                char wbuf[32];
                uint8_t wi = 0;
                while (uart_rx_tail != uart_rx_head && wi < sizeof(wbuf) - 1U) {
                    char wc = (char)uart_rx_ring[uart_rx_tail];
                    uart_rx_tail = (uint16_t)((uart_rx_tail + 1U) % sizeof(uart_rx_ring));
                    if (wc == '\n') break;
                    wbuf[wi++] = wc;
                }
                wbuf[wi] = '\0';
                unsigned reg_v, val_v;
                if (sscanf(wbuf, "%x,%x", &reg_v, &val_v) == 2 && reg_v <= 0xFF && val_v <= 0xFF) {
                    ov_write_reg = (uint8_t)reg_v;
                    ov_write_val = (uint8_t)val_v;
                    ov_write_pending = true;
                } else {
                    UART_Log("[CTRL] Bad format (expect W<reg_hex>,<val_hex>)\r\n");
                }
                break;
            }

            case 'D':
                /* Pixel dump: sample 9 points in the frame, print both byte orders */
                if (frame_counter == 0U) {
                    UART_Log("[CTRL] No frame captured yet\r\n");
                } else {
                    uint8_t di = write_buf_idx ^ 1U;
                    const uint8_t *fb = frame_buf[di];
                    static const uint32_t probe_x[] = {  80, 160, 240, 80, 160, 240, 80, 160, 240 };
                    static const uint32_t probe_y[] = {  40,  40,  40, 60,  60,  60, 80,  80,  80 };
                    /* Also count red with current thresholds for full frame */
                    uint32_t red_cnt = 0;
                    UART_Log("[PIXDUMP] Thresholds: R>=%u G5<=%u B<=%u min_px=%u\r\n",
                             red_r_min, red_g_max, red_b_max, red_min_px);
                    for (uint32_t pi = 0; pi < 9; pi++) {
                        uint32_t px = probe_x[pi], py = probe_y[pi];
                        if (px >= CAM_WIDTH || py >= CAM_HEIGHT) continue;
                        uint32_t off = (py * CAM_WIDTH + px) * 2U;
                        uint8_t b0 = fb[off], b1 = fb[off + 1];
                        uint16_t be = (uint16_t)((b0 << 8) | b1);
                        uint16_t le = (uint16_t)((b1 << 8) | b0);
                        UART_Log("[PIXDUMP] (%lu,%lu) raw=[%02X,%02X] "
                                 "BE:0x%04X R=%u G6=%u B=%u | "
                                 "LE:0x%04X R=%u G6=%u B=%u\r\n",
                                 (unsigned long)px, (unsigned long)py,
                                 b0, b1,
                                 be, (be >> 11) & 0x1F, (be >> 5) & 0x3F, be & 0x1F,
                                 le, (le >> 11) & 0x1F, (le >> 5) & 0x3F, le & 0x1F);
                    }
                    /* Count red pixels in full frame */
                    for (uint32_t i = 0; i < CAM_WIDTH * CAM_HEIGHT; i++) {
                        uint16_t pxv = ((const uint16_t *)fb)[i];
                        pxv = (uint16_t)((pxv << 8) | (pxv >> 8));
                        if (IsRedPixel565(pxv)) red_cnt++;
                    }
                    UART_Log("[PIXDUMP] Red pixels (BE swap): %lu / %lu\r\n",
                             (unsigned long)red_cnt, (unsigned long)(CAM_WIDTH * CAM_HEIGHT));
                    /* Count without swap */
                    red_cnt = 0;
                    for (uint32_t i = 0; i < CAM_WIDTH * CAM_HEIGHT; i++) {
                        uint16_t pxv = ((const uint16_t *)fb)[i];
                        if (IsRedPixel565(pxv)) red_cnt++;
                    }
                    UART_Log("[PIXDUMP] Red pixels (no swap): %lu / %lu\r\n",
                             (unsigned long)red_cnt, (unsigned long)(CAM_WIDTH * CAM_HEIGHT));
                }
                break;

            default:
                UART_Log("[CTRL] Unknown key '%c' (use h for help)\r\n", (char)ch);
                break;
        }
    }
}

static bool StateEntered(uint32_t *stamp)
{
    if (*stamp != state_generation) {
        *stamp = state_generation;
        return true;
    }

    return false;
}

static const char *AppStateName(AppState_t state)
{
    switch (state) {
        case APP_STATE_SPLASH:          return "SPLASH";
        case APP_STATE_F0_LIVE_PREVIEW: return "F0";
        case APP_STATE_D0_SYNC_DEBUG:   return "DBG0";
        case APP_STATE_D1_COLORBAR_DEBUG:return "DBG1";
        case APP_STATE_M0_COLOR_TEST:   return "M0";
        case APP_STATE_M1_SCCB_TEST:    return "M1";
        case APP_STATE_M2_SINGLE_FRAME: return "M2";
        case APP_STATE_M3_LIVE_PREVIEW: return "M3";
        case APP_STATE_M4_DOUBLE_BUF:   return "M4";
        case APP_STATE_M5_GRAYSCALE:    return "M5";
        case APP_STATE_M6_RED_TRACKING: return "M6";
        default:                        return "?";
    }
}

static void StartUart3RxIT(void)
{
    (void)HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
}

static void ProcessDeferredOvWrite(void)
{
    static uint32_t last_write_tick = 0;

    if (!ov_write_pending) return;

    /* Rate-limit: at least 200ms between writes */
    if (HAL_GetTick() - last_write_tick < 200U) return;

    /* Don't write if SPI is busy (LCD DMA in progress) */
    if (hspi1.State != HAL_SPI_STATE_READY) return;

    /* Also check I2C is ready */
    if (hi2c2.State != HAL_I2C_STATE_READY) return;

    ov_write_pending = false;
    last_write_tick = HAL_GetTick();

    if (OV7670_WriteReg(ov_write_reg, ov_write_val) == HAL_OK) {
        uint8_t rb = 0;
        OV7670_ReadReg(ov_write_reg, &rb);
        UART_Log("[CTRL] OV7670 reg 0x%02X = 0x%02X (rb=0x%02X)\r\n",
                 ov_write_reg, ov_write_val, rb);
    } else {
        UART_Log("[CTRL] OV7670 write reg 0x%02X failed\r\n", ov_write_reg);
    }
}

static void LogWindowAndFreezeDiag(void)
{
    uint8_t com10 = 0;
    uint8_t tslb = 0;
    uint8_t com3 = 0;
    uint8_t com14 = 0;
    uint8_t mvfp = 0;
    uint8_t pshift = 0;
    uint8_t hstart = 0;
    uint8_t hstop = 0;
    uint8_t href = 0;
    uint8_t vstart = 0;
    uint8_t vstop = 0;
    uint8_t vref = 0;
    uint16_t hstart_px;
    uint16_t hstop_px;
    uint16_t vstart_ln;
    uint16_t vstop_ln;
    uint8_t href_edge;
    HAL_StatusTypeDef status = HAL_OK;

    status |= OV7670_ReadReg(0x15, &com10);
    status |= OV7670_ReadReg(0x3A, &tslb);
    status |= OV7670_ReadReg(0x0C, &com3);
    status |= OV7670_ReadReg(0x3E, &com14);
    status |= OV7670_ReadReg(0x1E, &mvfp);
    status |= OV7670_ReadReg(0x1B, &pshift);
    status |= OV7670_ReadReg(0x17, &hstart);
    status |= OV7670_ReadReg(0x18, &hstop);
    status |= OV7670_ReadReg(0x32, &href);
    status |= OV7670_ReadReg(0x19, &vstart);
    status |= OV7670_ReadReg(0x1A, &vstop);
    status |= OV7670_ReadReg(0x03, &vref);

    if (status != HAL_OK) {
        UART_Log("[WIN] ERROR: SCCB readback failed while collecting window snapshot\r\n");
        return;
    }

    hstart_px = (uint16_t)(((uint16_t)hstart << 3) | (href & 0x07U));
    hstop_px  = (uint16_t)(((uint16_t)hstop << 3) | ((href >> 3) & 0x07U));
    vstart_ln = (uint16_t)(((uint16_t)vstart << 2) | (vref & 0x03U));
    vstop_ln  = (uint16_t)(((uint16_t)vstop << 2) | ((vref >> 2) & 0x03U));
    href_edge = (uint8_t)((href >> 6) & 0x03U);

    if (last_frame_callback_tick == 0U) {
        UART_Log("[WIN] state=%s disp_frames=%lu cb_frames=%lu last_cb=never ready=%u wr=%u disp_swap=%u sensor_swap=%u pshift=%u hshift=%d sharp=%s:%u denoise=%s:%u\r\n",
                 AppStateName(info.state),
                 (unsigned long)info.frame_count,
                 (unsigned long)frame_counter,
                 frame_ready ? 1U : 0U,
                 (unsigned)write_buf_idx,
                 display_byte_swap ? 1U : 0U,
                 sensor_byte_swap ? 1U : 0U,
                 (unsigned)sensor_pixel_shift,
                 (int)sensor_hshift_steps,
                 sensor_manual_sharpness ? "man" : "auto",
                 (unsigned)sensor_sharpness,
                 sensor_manual_denoise ? "man" : "auto",
                 (unsigned)sensor_denoise);
    } else {
        UART_Log("[WIN] state=%s disp_frames=%lu cb_frames=%lu last_cb_ms=%lu ready=%u wr=%u disp_swap=%u sensor_swap=%u pshift=%u hshift=%d sharp=%s:%u denoise=%s:%u\r\n",
                 AppStateName(info.state),
                 (unsigned long)info.frame_count,
                 (unsigned long)frame_counter,
                 (unsigned long)(HAL_GetTick() - last_frame_callback_tick),
                 frame_ready ? 1U : 0U,
                 (unsigned)write_buf_idx,
                 display_byte_swap ? 1U : 0U,
                 sensor_byte_swap ? 1U : 0U,
                 (unsigned)sensor_pixel_shift,
                 (int)sensor_hshift_steps,
                 sensor_manual_sharpness ? "man" : "auto",
                 (unsigned)sensor_sharpness,
                 sensor_manual_denoise ? "man" : "auto",
                 (unsigned)sensor_denoise);
    }

    UART_Log("[WIN] dcmi PCK=%s VS=%s HS=%s SR=0x%08lX CR=0x%08lX NDTR=%lu Err=0x%08lX\r\n",
             (hdcmi.Init.PCKPolarity == DCMI_PCKPOLARITY_RISING) ? "RISE" : "FALL",
             (hdcmi.Init.VSPolarity == DCMI_VSPOLARITY_LOW) ? "LOW" : "HIGH",
             (hdcmi.Init.HSPolarity == DCMI_HSPOLARITY_LOW) ? "LOW" : "HIGH",
             (unsigned long)DCMI->SR,
             (unsigned long)DCMI->CR,
             (unsigned long)GetDcmiNdtr(),
             (unsigned long)hdcmi.ErrorCode);
    UART_Log("[WIN] lcd spi=%s\r\n",
             (lcd_spi_prescaler == SPI_BAUDRATEPRESCALER_4) ? "/4" :
             (lcd_spi_prescaler == SPI_BAUDRATEPRESCALER_8) ? "/8" : "?");
    UART_Log("[WIN] reg COM10=%02X TSLB=%02X COM3=%02X COM14=%02X MVFP=%02X PSHFT=%02X\r\n",
             com10, tslb, com3, com14, mvfp, pshift);
    UART_Log("[WIN] hwin HSTART=%02X HSTOP=%02X HREF=%02X -> start=%u stop=%u edge=%u\r\n",
             hstart, hstop, href,
             (unsigned)hstart_px,
             (unsigned)hstop_px,
             (unsigned)href_edge);
    UART_Log("[WIN] vwin VSTART=%02X VSTOP=%02X VREF=%02X -> start=%u stop=%u\r\n",
             vstart, vstop, vref,
             (unsigned)vstart_ln,
             (unsigned)vstop_ln);

    if (hstop_px < hstart_px) {
        UART_Log("[WIN] note horizontal window wraps (stop < start), verify OV7670 timing/window setup\r\n");
    }
    if (last_frame_callback_tick != 0U &&
        (HAL_GetTick() - last_frame_callback_tick > 1500U)) {
        UART_Log("[WIN] warn no recent DCMI frame callback; preview can freeze on last frame\r\n");
    }
    {
        uint32_t fps10 = (info.fps <= 0.0f) ? 0U : (uint32_t)(info.fps * 10.0f + 0.5f);
        UART_Log("[WIN] perf fps=%lu.%lu xfer_avg_ms=%lu overlay_avg_ms=%lu capture_avg_ms=%lu\r\n",
                 (unsigned long)(fps10 / 10U),
                 (unsigned long)(fps10 % 10U),
                 (unsigned long)perf_transfer_avg_ms,
                 (unsigned long)perf_overlay_avg_ms,
                 (unsigned long)perf_capture_avg_ms);
    }
}

static inline uint32_t ReadPinFast(GPIO_TypeDef *port, uint16_t pin)
{
    return ((port->IDR & pin) != 0U) ? 1U : 0U;
}

static uint8_t SampleCameraDataBus(void)
{
    uint8_t bus = 0;

    if (ReadPinFast(GPIOC, GPIO_PIN_6))  bus |= (1U << 0);
    if (ReadPinFast(GPIOC, GPIO_PIN_7))  bus |= (1U << 1);
    if (ReadPinFast(GPIOG, GPIO_PIN_10)) bus |= (1U << 2);
    if (ReadPinFast(GPIOG, GPIO_PIN_11)) bus |= (1U << 3);
    if (ReadPinFast(GPIOC, GPIO_PIN_11)) bus |= (1U << 4);
    if (ReadPinFast(GPIOB, GPIO_PIN_6))  bus |= (1U << 5);
    if (ReadPinFast(GPIOB, GPIO_PIN_8))  bus |= (1U << 6);
    if (ReadPinFast(GPIOB, GPIO_PIN_9))  bus |= (1U << 7);

    return bus;
}

static void ProbeCameraSignals(uint32_t duration_ms, CamSignalDiag_t *diag)
{
    uint32_t start_tick;
    uint32_t last_pclk;
    uint32_t last_vsync;
    uint32_t last_href;
    uint8_t last_data = 0;
    bool have_last_data = false;

    memset(diag, 0, sizeof(*diag));
    diag->data_and = 0xFFU;

    last_pclk  = ReadPinFast(GPIOA, GPIO_PIN_6);
    last_vsync = ReadPinFast(GPIOB, GPIO_PIN_7);
    last_href  = ReadPinFast(GPIOA, GPIO_PIN_4);
    start_tick = HAL_GetTick();

    while ((HAL_GetTick() - start_tick) < duration_ms) {
        uint32_t pclk  = ReadPinFast(GPIOA, GPIO_PIN_6);
        uint32_t vsync = ReadPinFast(GPIOB, GPIO_PIN_7);
        uint32_t href  = ReadPinFast(GPIOA, GPIO_PIN_4);

        if (pclk != last_pclk) {
            diag->pclk_toggles++;
        }
        if (vsync != last_vsync) {
            diag->vsync_toggles++;
        }
        if (href != last_href) {
            diag->href_toggles++;
        }
        if (href != 0U) {
            diag->href_high_samples++;
        }

        if ((last_pclk == 0U) && (pclk != 0U) && (href != 0U)) {
            uint8_t data = SampleCameraDataBus();

            diag->pclk_rising_while_href++;
            diag->data_or |= data;
            diag->data_and &= data;

            if (diag->first_count < sizeof(diag->first_data)) {
                diag->first_data[diag->first_count++] = data;
            }

            if (have_last_data && (data != last_data)) {
                diag->data_transitions++;
            }

            last_data = data;
            have_last_data = true;
            diag->data_samples++;
        }

        last_pclk = pclk;
        last_vsync = vsync;
        last_href = href;
    }

    if (diag->data_samples == 0U) {
        diag->data_and = 0x00U;
    }
}

static void LogCameraSignalDiag(const CamSignalDiag_t *diag)
{
    UART_Log("[M2] RawProbe: PCLKtog=%lu VSYNCtog=%lu HREFtog=%lu HREF_hi=%lu PCLK@HREF=%lu\r\n",
             (unsigned long)diag->pclk_toggles,
             (unsigned long)diag->vsync_toggles,
             (unsigned long)diag->href_toggles,
             (unsigned long)diag->href_high_samples,
             (unsigned long)diag->pclk_rising_while_href);
    UART_Log("[M2] RawProbe: samples=%lu transitions=%lu data_or=0x%02X data_and=0x%02X\r\n",
             (unsigned long)diag->data_samples,
             (unsigned long)diag->data_transitions,
             diag->data_or,
             diag->data_and);

    if (diag->first_count > 0U) {
        UART_Log("[M2] RawProbe: first data bytes:");
        for (uint32_t i = 0; i < diag->first_count; i++) {
            UART_Log(" %02X", diag->first_data[i]);
        }
        UART_Log("\r\n");
    }
}

static uint32_t GetDcmiNdtr(void)
{
    DMA_Stream_TypeDef *stream = (DMA_Stream_TypeDef *)hdcmi.DMA_Handle->Instance;
    return stream->NDTR;
}

static bool ProbeDcmiCapture(uint32_t pck_polarity, uint32_t vs_polarity,
                             uint32_t hs_polarity, uint32_t wait_ms,
                             DcmiProbeResult_t *result)
{
    uint32_t start_tick;
    uint32_t frame_count_start;

    memset(result, 0, sizeof(*result));
    result->pck_polarity = pck_polarity;
    result->vs_polarity  = vs_polarity;
    result->hs_polarity  = hs_polarity;

    HAL_DCMI_Stop(&hdcmi);
    HAL_DCMI_DeInit(&hdcmi);

    hdcmi.Init.PCKPolarity = pck_polarity;
    hdcmi.Init.VSPolarity  = vs_polarity;
    hdcmi.Init.HSPolarity  = hs_polarity;

    if (HAL_DCMI_Init(&hdcmi) != HAL_OK) {
        result->error_code = 0xFFFFFFFFU;
        return false;
    }
    ConfigureDcmiCrop();

    __HAL_DCMI_CLEAR_FLAG(&hdcmi,
                          DCMI_FLAG_FRAMERI |
                          DCMI_FLAG_OVRRI |
                          DCMI_FLAG_ERRRI |
                          DCMI_FLAG_VSYNCRI |
                          DCMI_FLAG_LINERI);
    __HAL_DCMI_ENABLE_IT(&hdcmi,
                         DCMI_IT_FRAME |
                         DCMI_IT_OVR |
                         DCMI_IT_ERR |
                         DCMI_IT_VSYNC |
                         DCMI_IT_LINE);

    frame_ready = false;
    frame_count_start = frame_counter;
    hdcmi.ErrorCode = HAL_DCMI_ERROR_NONE;
    memset(frame_buf[0], 0, FRAME_BUF_SIZE);

    if (HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS,
                           (uint32_t)frame_buf[0],
                           CAM_FRAME_SIZE / 4U) != HAL_OK) {
        result->error_code = hdcmi.ErrorCode;
        return false;
    }

    result->sr_start = DCMI->SR;
    result->ndtr_start = GetDcmiNdtr();

    start_tick = HAL_GetTick();
    while ((HAL_GetTick() - start_tick) < wait_ms) {
        uint32_t ndtr_now = GetDcmiNdtr();

        if (ndtr_now != result->ndtr_start) {
            result->ndtr_moved = true;
            break;
        }
        if ((DCMI->SR & DCMI_SR_FNE) != 0U) {
            result->fifo_seen = true;
            break;
        }
        if (frame_ready) {
            break;
        }
    }

    result->ndtr_end = GetDcmiNdtr();
    result->sr_end   = DCMI->SR;
    result->cr_end   = DCMI->CR;
    result->ris_end  = DCMI->RISR;
    result->mis_end  = DCMI->MISR;
    result->ier_end  = DCMI->IER;
    result->error_code = hdcmi.ErrorCode;
    result->frame_count_delta = frame_counter - frame_count_start;
    result->fifo_seen = result->fifo_seen || ((result->sr_end & DCMI_SR_FNE) != 0U);

    HAL_DCMI_Stop(&hdcmi);
    result->nonzero_bytes = CountNonZeroBytes(frame_buf[0]);

    return (result->nonzero_bytes != 0U) ||
           result->ndtr_moved ||
           result->fifo_seen ||
           (result->frame_count_delta != 0U);
}

static void LogDcmiProbeResult(const char *label, const DcmiProbeResult_t *result)
{
    UART_Log("[M2] Probe %-13s NDTR=%lu->%lu NZ=%lu PCK=%s VS=%s HS=%s FIFO=%u Frame=%lu Err=0x%08lX\r\n",
             label,
             (unsigned long)result->ndtr_start,
             (unsigned long)result->ndtr_end,
             (unsigned long)result->nonzero_bytes,
             (result->pck_polarity == DCMI_PCKPOLARITY_RISING) ? "RISE" : "FALL",
             (result->vs_polarity  == DCMI_VSPOLARITY_LOW) ? "LOW" : "HIGH",
             (result->hs_polarity  == DCMI_HSPOLARITY_LOW) ? "LOW" : "HIGH",
             result->fifo_seen ? 1U : 0U,
             (unsigned long)result->frame_count_delta,
             (unsigned long)result->error_code);
    UART_Log("[M2]       SR=0x%08lX->0x%08lX CR=0x%08lX RIS=0x%08lX MIS=0x%08lX IER=0x%08lX\r\n",
             (unsigned long)result->sr_start,
             (unsigned long)result->sr_end,
             (unsigned long)result->cr_end,
             (unsigned long)result->ris_end,
             (unsigned long)result->mis_end,
             (unsigned long)result->ier_end);
}

static bool IsRedPixel565(uint16_t px)
{
    uint8_t r = (uint8_t)((px >> 11) & 0x1FU);  /* 0-31 */
    uint8_t g = (uint8_t)((px >> 5) & 0x3FU);   /* 0-63 */
    uint8_t b = (uint8_t)(px & 0x1FU);           /* 0-31 */
    uint8_t g5 = (uint8_t)(g >> 1);              /* 0-31 */

    return (r >= red_r_min) &&
           (g5 <= red_g_max) &&
           (b <= red_b_max) &&
           (r > (uint8_t)(g5 * 2U)) &&
           (r > (uint8_t)(b * 2U));
}

static RedTrack_t TrackRedObject(const uint8_t *fb)
{
    RedTrack_t track;

    memset(&track, 0, sizeof(track));
    track.min_x = CAM_WIDTH - 1U;
    track.min_y = CAM_HEIGHT - 1U;

    for (uint32_t y = 0; y < CAM_HEIGHT; y++) {
        const uint16_t *row = (const uint16_t *)(fb + y * CAM_WIDTH * 2U);

        for (uint32_t x = 0; x < CAM_WIDTH; x++) {
            uint16_t px = row[x];
            /* DCMI stores big-endian bytes; ARM reads little-endian uint16 → swap */
            px = (uint16_t)((px << 8) | (px >> 8));

            if (!IsRedPixel565(px)) {
                continue;
            }

            track.count++;
            track.sum_x += x;
            track.sum_y += y;

            if (x < track.min_x) track.min_x = (uint16_t)x;
            if (x > track.max_x) track.max_x = (uint16_t)x;
            if (y < track.min_y) track.min_y = (uint16_t)y;
            if (y > track.max_y) track.max_y = (uint16_t)y;
        }
    }

    if (track.count >= red_min_px) {
        track.found = true;
        track.cx = (uint16_t)(track.sum_x / track.count);
        track.cy = (uint16_t)(track.sum_y / track.count);
    } else {
        track.count = 0;
        track.min_x = 0;
        track.min_y = 0;
        track.max_x = 0;
        track.max_y = 0;
    }

    return track;
}

static void DrawTrackingRect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
                             uint16_t color)
{
    uint16_t w;
    uint16_t h;

    if (x0 >= LCD_WIDTH)  x0 = LCD_WIDTH - 1U;
    if (x1 >= LCD_WIDTH)  x1 = LCD_WIDTH - 1U;
    if (y0 >= LCD_HEIGHT) y0 = LCD_HEIGHT - 1U;
    if (y1 >= LCD_HEIGHT) y1 = LCD_HEIGHT - 1U;

    if (x1 < x0) {
        uint16_t tmp = x0;
        x0 = x1;
        x1 = tmp;
    }
    if (y1 < y0) {
        uint16_t tmp = y0;
        y0 = y1;
        y1 = tmp;
    }

    w = (uint16_t)(x1 - x0 + 1U);
    h = (uint16_t)(y1 - y0 + 1U);

    LCD_FillRect(x0, y0, w, 1U, color);
    LCD_FillRect(x0, y1, w, 1U, color);
    LCD_FillRect(x0, y0, 1U, h, color);
    LCD_FillRect(x1, y0, 1U, h, color);
}

static void DrawTrackingCrosshair(uint16_t x, uint16_t y, uint16_t color)
{
    uint16_t left = (x > 6U) ? (uint16_t)(x - 6U) : 0U;
    uint16_t top = (y > 6U) ? (uint16_t)(y - 6U) : 0U;
    uint16_t right = (x + 6U < LCD_WIDTH) ? (uint16_t)(x + 6U) : (LCD_WIDTH - 1U);
    uint16_t bottom = (y + 6U < LCD_HEIGHT) ? (uint16_t)(y + 6U) : (LCD_HEIGHT - 1U);

    LCD_FillRect(left, y, (uint16_t)(right - left + 1U), 1U, color);
    LCD_FillRect(x, top, 1U, (uint16_t)(bottom - top + 1U), color);
}

static void DrawTrackingOverlay(const RedTrack_t *track)
{
    uint16_t x0;
    uint16_t y0;
    uint16_t x1;
    uint16_t y1;
    uint16_t cx;
    uint16_t cy;

    if (!track->found) {
        return;
    }

    x0 = (uint16_t)(((uint32_t)track->min_x * LCD_WIDTH) / CAM_WIDTH);
    y0 = (uint16_t)(((uint32_t)track->min_y * LCD_HEIGHT) / CAM_HEIGHT);
    x1 = (uint16_t)((((uint32_t)track->max_x + 1U) * LCD_WIDTH) / CAM_WIDTH);
    y1 = (uint16_t)((((uint32_t)track->max_y + 1U) * LCD_HEIGHT) / CAM_HEIGHT);
    cx = (uint16_t)(((uint32_t)track->cx * LCD_WIDTH) / CAM_WIDTH);
    cy = (uint16_t)(((uint32_t)track->cy * LCD_HEIGHT) / CAM_HEIGHT);

    if (x1 > 0U) {
        x1--;
    }
    if (y1 > 0U) {
        y1--;
    }

    DrawTrackingRect(x0, y0, x1, y1, RED_TRACK_BOX_COLOR);
    DrawTrackingCrosshair(cx, cy, RED_TRACK_CROSS_COLOR);
}

static void ChangeState(AppState_t new_state)
{
    info.state = new_state;
    state_entry_tick = HAL_GetTick();
    state_generation++;
}

static void UpdateFPS(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t dt = now - fps_tick_start;
    if (dt >= 1000) {
        info.fps = (float)fps_frame_count * 1000.0f / (float)dt;
        fps_frame_count = 0;
        fps_tick_start = now;
    }
}

static void PerfReset(void)
{
    perf_tick_start = HAL_GetTick();
    perf_transfer_accum_ms = 0U;
    perf_overlay_accum_ms = 0U;
    perf_frames_accum = 0U;
    perf_transfer_avg_ms = 0U;
    perf_overlay_avg_ms = 0U;
    perf_capture_last_tick = 0U;
    perf_capture_accum_ms = 0U;
    perf_capture_samples = 0U;
    perf_capture_avg_ms = 0U;
}

static void PerfRecordFrame(uint32_t transfer_ms, uint32_t overlay_ms)
{
    uint32_t now = HAL_GetTick();
    uint32_t dt = now - perf_tick_start;

    perf_transfer_accum_ms += transfer_ms;
    perf_overlay_accum_ms += overlay_ms;
    perf_frames_accum++;

    if (dt >= 1000U && perf_frames_accum != 0U) {
        perf_transfer_avg_ms = perf_transfer_accum_ms / perf_frames_accum;
        perf_overlay_avg_ms = perf_overlay_accum_ms / perf_frames_accum;
        if (perf_capture_samples != 0U) {
            perf_capture_avg_ms = perf_capture_accum_ms / perf_capture_samples;
        }
        perf_transfer_accum_ms = 0U;
        perf_overlay_accum_ms = 0U;
        perf_frames_accum = 0U;
        perf_capture_accum_ms = 0U;
        perf_capture_samples = 0U;
        perf_tick_start = now;
    }
}

const AppInfo_t *App_GetInfo(void)
{
    return &info;
}

/* ================================================================== */
/*  UART frame dump — send raw RGB565 for PC analysis                  */
/* ================================================================== */
static void DumpFrameToUART(const uint8_t *fb)
{
    /* Header: "FRAME" + width(2) + height(2) + bpp(1) = 9 bytes */
    const uint8_t hdr[] = {
        'F','R','A','M','E',
        (uint8_t)(CAM_WIDTH >> 8), (uint8_t)(CAM_WIDTH & 0xFF),
        (uint8_t)(CAM_HEIGHT >> 8), (uint8_t)(CAM_HEIGHT & 0xFF),
        CAM_BPP
    };
    HAL_UART_Transmit(&huart3, (uint8_t *)hdr, sizeof(hdr), 100);

    /* Send raw pixel data in 1024-byte chunks */
    uint32_t remaining = CAM_FRAME_SIZE;
    const uint8_t *p = fb;
    while (remaining > 0) {
        uint16_t chunk = (remaining > 1024) ? 1024 : (uint16_t)remaining;
        HAL_UART_Transmit(&huart3, (uint8_t *)p, chunk, 500);
        p += chunk;
        remaining -= chunk;
    }

    /* Tail marker */
    const uint8_t tail[] = {'E','N','D','\n'};
    HAL_UART_Transmit(&huart3, (uint8_t *)tail, sizeof(tail), 100);
}

/* ================================================================== */
/*  Callbacks (called from ISR)                                        */
/* ================================================================== */
void App_DCMI_FrameCallback(void)
{
    uint32_t now = HAL_GetTick();

    if (perf_capture_last_tick != 0U) {
        perf_capture_accum_ms += (now - perf_capture_last_tick);
        perf_capture_samples++;
    }
    perf_capture_last_tick = now;
    frame_counter++;
    frame_ready = true;
    last_frame_callback_tick = now;

    /* Swap ping-pong buffer for next capture */
    write_buf_idx ^= 1;

    /* Redirect DCMI DMA to the new write buffer */
    HAL_DCMI_Stop(&hdcmi);
    HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS,
                        (uint32_t)frame_buf[write_buf_idx],
                        CAM_FRAME_SIZE / 4);
}

void App_SPI_TxCpltCallback(void)
{
    LCD_DMA_TxCpltCallback();
}

void App_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3) {
        uint16_t next_head = (uint16_t)((uart_rx_head + 1U) % (uint16_t)sizeof(uart_rx_ring));

        if (next_head != uart_rx_tail) {
            uart_rx_ring[uart_rx_head] = uart3_rx_byte;
            uart_rx_head = next_head;
        }

        StartUart3RxIT();
    }
}
