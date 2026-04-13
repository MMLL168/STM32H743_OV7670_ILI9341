# STM32H743ZI2 + OV7670 + ILI9341 即時影像預覽系統

使用 NUCLEO-H743ZI2 開發板，透過 DCMI 介面擷取 OV7670 攝影機的 QVGA (320x240) RGB565 影像，以 SPI 驅動 ILI9341 LCD (320x240) 1:1 即時顯示，並搭配桌面 `debug_gui.py` 工具進行影像調參、frame dump 與現場除錯。

---

## 硬體需求

| 元件 | 規格 |
|------|------|
| MCU Board | NUCLEO-H743ZI2 (STM32H743ZITx, LQFP144) |
| Camera | OV7670 (無 FIFO 版本) |
| LCD | ILI9341 2.4" SPI TFT (320x240, RGB565) |
| 電源 | 3.3V (由 NUCLEO 板供電) |

---

## 接線

```
OV7670          STM32H743ZI2        ILI9341
──────          ────────────        ───────
D0   ────────── PC6
D1   ────────── PC7
D2   ────────── PG10
D3   ────────── PG11
D4   ────────── PC11
D5   ────────── PB6
D6   ────────── PB8
D7   ────────── PB9
VSYNC ───────── PB7
HSYNC ───────── PA4
PCLK ─────────  PA6
XCLK ─────────  PA8 (MCO1, 8MHz)
SDA  ─────────  PF0 (I2C2)
SCL  ─────────  PF1 (I2C2)
RESET ────────  PG0
PWDN ─────────  PG1
                 PA5  ──────────── SCK
                 PA7  ──────────── MOSI (SDA)
                 PE3  ──────────── CS
                 PE4  ──────────── DC (RS)
                 PE5  ──────────── RST
                 3.3V ──────────── LED (背光)
                 GND  ──────────── GND
```

---

## 專案結構

```
NUCLEO-H743ZI2_OV7670_ILI9341/
├── App/                              ← 應用程式碼（不受 CubeMX 覆蓋）
│   ├── Inc/
│   │   ├── ili9341.h                 ← ILI9341 LCD 驅動 API
│   │   ├── ov7670.h                  ← OV7670 攝影機驅動 API
│   │   ├── gui.h                     ← GUI 覆蓋層 API
│   │   ├── app.h                     ← 應用狀態機 API
│   │   └── font5x7.h                ← 5x7 點陣字型 (ASCII 32-127)
│   └── Src/
│       ├── ili9341.c                 ← SPI + DMA 驅動實作
│       ├── ov7670.c                  ← SCCB (I2C) + DCMI 驅動實作
│       ├── gui.c                     ← 文字渲染、狀態列、啟動畫面
│       └── app.c                     ← 狀態機、雙緩衝、縮放、影像處理
├── Core/                             ← CubeMX 生成（USER CODE 區塊內有修改）
│   ├── Inc/
│   │   ├── main.h
│   │   ├── dcmi.h / dma.h / i2c.h / spi.h / gpio.h
│   │   └── stm32h7xx_it.h
│   └── Src/
│       ├── main.c                    ← USER CODE: App_Init() + App_Run()
│       ├── stm32h7xx_it.c            ← USER CODE: DCMI_IRQ + HAL callbacks
│       └── dcmi.c / dma.c / i2c.c / spi.c / gpio.c
├── Drivers/                          ← STM32 HAL/CMSIS（CubeMX 管理）
├── CMakeLists.txt                    ← 已加入 App/ sources + includes
├── STM32H743XX_FLASH.ld              ← 已加入 .axi_sram section
├── proposal.md                       ← 原始規格書
├── devlog.md                         ← 開發日誌
└── NUCLEO-H743ZI2_OV7670_ILI9341_template.ioc
```

---

## CubeMX 相容性

所有對 CubeMX 生成檔案的修改皆在 `/* USER CODE BEGIN */` ... `/* USER CODE END */` 區塊內，重新生成程式碼時不會被覆蓋。

自訂程式碼完全放在 `App/` 目錄，CubeMX 不會觸碰此目錄。

修改清單：

| 檔案 | USER CODE 區塊 | 內容 |
|------|----------------|------|
| `Core/Src/main.c` | `Includes` | `#include "app.h"` |
| `Core/Src/main.c` | `2` | `App_Init();` |
| `Core/Src/main.c` | `3` | `App_Run();` |
| `Core/Src/stm32h7xx_it.c` | `Includes` | `#include "app.h"` / `#include "dcmi.h"` |
| `Core/Src/stm32h7xx_it.c` | `1` | `DCMI_IRQHandler` + HAL callbacks |
| `STM32H743XX_FLASH.ld` | (非 CubeMX) | `.axi_sram` section → AXI SRAM |
| `CMakeLists.txt` | (非 CubeMX) | App sources + include path |

---

## 系統時鐘

| 項目 | 值 |
|------|-----|
| HSE | 8 MHz |
| SYSCLK | 480 MHz |
| HCLK (AHB) | 240 MHz |
| APB1 / APB2 | 120 MHz |
| MCO1 → OV7670 XCLK | 8 MHz |
| SPI1 Baud Rate | 24 MHz (PLL1Q 192MHz / 8) |
| I2C2 (SCCB) | 100 kHz |

---

## 記憶體配置

| 區域 | 地址 | 用途 |
|------|------|------|
| DTCM (128KB) | 0x20000000 | Stack + Heap + .data + .bss |
| AXI SRAM (512KB) | 0x24000000 | Frame Buffer x2 (300KB) + Line Buffer x2 (1.3KB) |

Frame buffer 透過 `__attribute__((section(".axi_sram"), aligned(32)))` 放置，確保 DMA 可存取且 D-Cache 對齊。

---

## 開機流程（目前 Debug Boot）

目前韌體預設啟用 `APP_DEBUG_BOOT=1`，上電後直接進入 `F0: Live Preview`，方便現場先看畫面、再用 UART 或 GUI 切換模式。

可用模式：

```
F0: Live Preview
 ├─ p = 回一般預覽
 ├─ c = OV7670 color bar
 ├─ d = sync / polarity 診斷
 ├─ t = red tracking
 └─ w = window / freeze snapshot
```

傳統 `M0~M6` 里程碑狀態機仍保留在韌體內，方便個別測試，但目前現場工作流程以 `F0 + debug controls` 為主。

### 里程碑詳細

| 里程碑 | 目標 | 驗證方式 |
|--------|------|---------|
| **M0** | LCD 驅動驗證 | 紅→綠→藍→白 全螢幕依序顯示 |
| **M1** | OV7670 SCCB 通訊 | 讀取 PID=0x76，畫面顯示 PASS/FAIL |
| **M2** | DCMI 單幀擷取 | 快照模式擷取一幀並驗證 DCMI / DMA / crop |
| **M3** | 連續即時預覽 | DCMI 連續模式 + 狀態列顯示即時 FPS |
| **M4** | 雙緩衝優化 | Ping-Pong buffer，DCMI 寫入一塊、LCD 讀取另一塊 |
| **M5** | 進階影像處理 | RGB → 灰階 → Sobel 邊緣偵測 |
| **M6** | 紅色物體追蹤 | UART/GUI 協助調整閾值、LCD 疊加框線與準星 |

---

## GUI 介面

系統內建 LCD GUI 覆蓋層，直接在 ILI9341 上繪製：

- **狀態列** (頂部 12px)：左側顯示當前里程碑名稱，右側顯示即時 FPS (綠色)
- **啟動畫面**：專案名稱 + 參數資訊
- **測試結果**：PASS (綠色) / FAIL (紅色) 明確標示
- **資訊框**：底部顯示擷取參數

字型使用內建 5x7 bitmap font，支援 ASCII 32-127。

### 桌面 Debug GUI

`tools/debug_gui.py` 提供 PC 端除錯介面，可透過 ST-Link VCP 進行：

- 模式切換：`Preview / Color Bar / Tracking / Diag`
- 手動影像調整：`H- / H+ / Sharp- / Sharp+ / Noise- / Noise+ / Shift- / Shift+`
- `Window/Freeze` snapshot
- `Dump Frame`
- `Start Live Grab` 週期性抓圖
- 四種 decode 變體檢視與主預覽切換：
  - `Normal (BE)`
  - `Byte Swapped (LE)`
  - `Word Swapped`
  - `Word+Byte Swapped`

---

## 關鍵技術設計

### D-Cache 一致性
STM32H7 啟用 D-Cache 後，DMA 與 CPU 之間存在資料不一致問題：
- DCMI DMA 寫入 frame buffer 後 → `SCB_InvalidateDCache_by_Addr()` 確保 CPU 讀到最新資料
- SPI DMA 傳輸前 → `SCB_CleanDCache_by_Addr()` 確保 DMA 讀到 CPU 寫入的資料
- Frame buffer 32-byte 對齊，符合 cache line 大小

### SCCB 容錯
OV7670 使用 SCCB 協議，ACK 行為與標準 I2C 不同：
- 每次 I2C 操作最多 retry 5 次
- 自動清除 `I2C_FLAG_AF` 錯誤旗標
- 每次 retry 間隔 1ms

### QVGA 1:1 直出
- OV7670 目前以 `320x240 RGB565` 輸出
- LCD 直接以 `320x240` 顯示，不再依賴 `160x120 → 320x240` 的 2x 放大
- 仍使用 line buffer 逐行傳送，避免額外配置完整 LCD shadow buffer

### Ping-Pong 雙緩衝
- Buffer A/B 交替：DCMI 寫入 A 時，CPU 讀取 B 送往 LCD
- 幀完成中斷 (`HAL_DCMI_FrameEventCallback`) 觸發 buffer 切換
- 避免撕裂現象 (tearing)

---

## 編譯

```bash
# 使用 CMake + arm-none-eabi-gcc
cmake --preset Debug
cmake --build --preset Debug
```

或使用 STM32CubeIDE 匯入 `.ioc` 檔案後編譯。

---

## 燒錄

```bash
# 使用 ST-Link (OpenOCD)
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
  -c "program build/Debug/NUCLEO-H743ZI2_OV7670_ILI9341_template.elf verify reset exit"

# 或使用 STM32CubeProgrammer
STM32_Programmer_CLI -c port=SWD -w build/Debug/NUCLEO-H743ZI2_OV7670_ILI9341_template.elf
```

---

## 注意事項

1. **SCCB ≠ I2C**：OV7670 ACK 行為不規範，韌體已內建 retry 機制
2. **D-Cache**：已處理 DMA/CPU 資料一致性，不需手動設定 MPU
3. **PC11 (DCMI_D4)**：確認 CubeMX 中 AF13 正確配對
4. **PF0/PF1 (I2C2)**：確認 NUCLEO Morpho connector 上這兩支 pin 沒有被 jumper 佔用
5. **Sync 極性**：目前實際穩定工作點為 `PCLK=Rising / VSYNC=High / HSYNC=Low`，韌體會在 debug flow 中套用此組合
6. **CubeMX 重新生成**：重新生成後只需確認 `CMakeLists.txt` 的 App source/include 仍存在
