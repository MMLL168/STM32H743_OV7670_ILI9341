# STM32H743ZI2 + OV7670 + ILI9341 專案提案

## 專案概述

使用 NUCLEO-H743ZI2 開發板，透過 DCMI 介面擷取 OV7670 攝影機的 QVGA (320×240) RGB565 影像，並以 SPI 驅動 ILI9341 LCD 進行 1:1 即時顯示，同時保留桌面 GUI 除錯與影像調參能力。

---

## 1. MCU 與時鐘樹

| 項目 | 設定 |
|------|------|
| MCU | STM32H743ZITx (LQFP144) |
| Board | NUCLEO-H743ZI2 |
| Toolchain | CMake |
| HSE | 8 MHz |
| PLL Source | HSE |
| DIVM1 / DIVN1 / DIVP1 | /1 × 120 /2 |
| SYSCLK | **480 MHz** |
| HCLK (AHB) | **240 MHz** (HPRE /2) |
| APB1 / APB2 | **120 MHz** (D2PPREx /2) |
| APB3 / APB4 | **120 MHz** |
| MCO1 (PA8) | HSE /1 = **8 MHz** → OV7670 XCLK |
| I2C1,2,3 Clock | PCLK1 = 120 MHz |
| SPI1,2,3 Clock | PLL1Q = 192 MHz |
| Cortex-M7 | I-Cache ✅ / D-Cache ✅ |
| Stack Size | 0x2000 (8 KB) |
| Heap Size | 0x10000 (64 KB) |

---

## 2. DCMI — OV7670 攝影機介面

### 2.1 Mode & Parameter

| 參數 | 設定 |
|------|------|
| Mode | Slave 8 bits External Synchro |
| Pixel Clock Polarity | **Active on Rising Edge** |
| Vertical Sync Polarity | Active High |
| Horizontal Sync Polarity | Active Low |
| Capture Rate | All frames are captured |
| JPEG Mode | Disabled |

### 2.2 DCMI 腳位分配

| OV7670 信號 | STM32 Pin | Alternate Function |
|-------------|-----------|-------------------|
| D0 | PC6 | DCMI_D0 (AF13) |
| D1 | PC7 | DCMI_D1 (AF13) |
| D2 | PG10 | DCMI_D2 (AF13) |
| D3 | PG11 | DCMI_D3 (AF13) |
| D4 | PC11 | DCMI_D4 (AF13) |
| D5 | PB6 | DCMI_D5 (AF13) |
| D6 | PB8 | DCMI_D6 (AF13) |
| D7 | PB9 | DCMI_D7 (AF13) |
| VSYNC | PB7 | DCMI_VSYNC (AF13) |
| HSYNC | PA4 | DCMI_HSYNC (AF13) |
| PCLK | PA6 | DCMI_PIXCLK (AF13) |
| XCLK | PA8 | RCC_MCO_1 (8 MHz) |

> 所有 DCMI 腳位 GPIO Speed 設為 Very High。

### 2.3 DCMI DMA (DMA1 Stream 0)

| 參數 | 設定 |
|------|------|
| Direction | Peripheral to Memory |
| Mode | **Circular** |
| Priority | **High** |
| Peripheral Increment | ✗ |
| Memory Increment | ✓ |
| Peripheral Data Width | Word (32-bit) |
| Memory Data Width | Word (32-bit) |
| FIFO | **Enabled / Threshold Full** |
| Peripheral Burst | Single |
| Memory Burst | **Incremental Burst of 4 beats** |

---

## 3. I2C2 — OV7670 SCCB 通訊

| 參數 | 設定 |
|------|------|
| SDA | PF0 (AF4, 內部上拉) |
| SCL | PF1 (AF4, 內部上拉) |
| Speed Mode | Standard Mode |
| Frequency | 100 kHz |
| Timing Register | **0x307075B1** (H7 專用, APB1=120MHz) |
| Addressing | 7-bit |
| Analog Filter | Enabled |
| Clock No Stretch | Disabled |

> ⚠️ OV7670 使用 SCCB 協議，ACK 行為與標準 I2C 不同。韌體端需加入 Retry 機制，忽略 HAL_I2C_ERROR_AF 錯誤碼。

---

## 4. SPI1 — ILI9341 LCD 顯示

### 4.1 SPI 參數

| 參數 | 設定 |
|------|------|
| Mode | Transmit Only Master |
| Hardware NSS | Disable |
| Frame Format | Motorola |
| Data Size | 8 Bits |
| First Bit | MSB First |
| Prescaler | **8** |
| Baud Rate | PLL1Q(192MHz) / 8 = **24 MHz** |
| CPOL | Low |
| CPHA | 1 Edge (SPI Mode 0) |
| NSS Signal Type | Software |
| NSSP Mode | **Disabled** |
| CRC | Disabled |

### 4.2 SPI1 TX DMA (DMA1 Stream 1)

| 參數 | 設定 |
|------|------|
| Direction | Memory to Peripheral |
| Mode | Normal |
| Priority | **Medium** |
| Peripheral Increment | ✗ |
| Memory Increment | ✓ |
| Peripheral Data Width | Byte |
| Memory Data Width | Byte |
| FIFO | Disabled |

### 4.3 LCD 控制腳位

| 功能 | Pin | 初始電平 | Speed | 說明 |
|------|-----|---------|-------|------|
| LCD_CS | PE3 | **High** | Very High | 預設不選中 |
| LCD_DC | PE4 | **Low** | Very High | 預設命令模式 |
| LCD_RST | PE5 | **High** | Low | 預設不復位 |
| SPI1_SCK | PA5 | — | — | AF5 |
| SPI1_MOSI | PA7 | — | — | AF5 |

---

## 5. OV7670 控制 GPIO

| 功能 | Pin | 初始電平 | 說明 |
|------|-----|---------|------|
| OV7670_RESET | PG0 | **High** | Low = 復位, High = 正常 |
| OV7670_PWDN | PG1 | **Low** | Low = 正常工作, High = 省電 |

---

## 6. NVIC 中斷優先級

| 中斷 | Preempt Priority | 說明 |
|------|------------------|------|
| DMA1_Stream0 (DCMI) | **1** | 最高 — 影像資料不可丟失 |
| DCMI | **2** | 幀完成通知 |
| DMA1_Stream1 (SPI TX) | **3** | LCD 刷屏 |
| SysTick / TIM6 | **15** | HAL 時基 |

> Priority Group = **NVIC_PRIORITYGROUP_4** (4-bit preemption, 0-bit sub)

---

## 7. Project Manager 設定

| 項目 | 設定 |
|------|------|
| Toolchain | **CMake** |
| Stack Size | **0x2000** (8 KB) |
| Heap Size | **0x10000** (64 KB) |
| Code Generator | Generate peripheral initialization as a pair of .c/.h files ✅ |

---

## 8. 記憶體規劃 (QVGA RGB565)

| 用途 | 計算 | 大小 |
|------|------|------|
| Frame Buffer × 1 | 320 × 240 × 2 bytes | 150 KB |
| Frame Buffer × 2 (Ping-Pong) | 150 KB × 2 | **300 KB** |
| AXI SRAM 可用 | — | 512 KB |
| D1 DTCM | — | 128 KB |

> Frame Buffer 建議放在 AXI SRAM (0x24000000)，須 4-byte 對齊。
> DTCM 不可被 DMA 存取，不適合放 Frame Buffer。

---

## 9. 開發里程碑

| 里程碑 | 目標 | 驗證方式 |
|--------|------|---------|
| **M0** | LCD 純色測試 | ILI9341 顯示紅/綠/藍全畫面 |
| **M1** | OV7670 SCCB 通訊 | 讀取 OV7670 PID (0x76) |
| **M2** | DCMI 單幀擷取 | DMA 寫入 Frame Buffer，UART 輸出前幾筆 pixel |
| **M3** | Camera → LCD 即時顯示 | QVGA 320×240 1:1 連續顯示 |
| **M4** | 效能優化 | Double Buffer + SPI 串流顯示優化 |
| **M5** | 進階功能 | 灰階轉換 / 簡易邊緣偵測 |
| **M6** | 物件追蹤 | 紅色物體追蹤、UART/GUI 協助調參 |

---

## 10. 目前除錯工作流

目前專案除了板上 LCD GUI 外，也搭配 `tools/debug_gui.py` 作為桌面除錯工具。其用途包括：

- 透過 ST-Link VCP 切換韌體模式：`Preview / Color Bar / Tracking / Diag`
- 手動調整影像參數：`HSHIFT / Sharpness / Denoise / PSHFT / Byte Swap`
- 取得 `Window/Freeze` snapshot 與 frame dump
- 用 PC 端顯示四種 decode 變體，快速判斷 byte / word packing 問題
- 啟用 `Live Grab` 形成近即時桌面預覽

---

## 11. 接線總表

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
VSYNC ─────────  PB7                 
HSYNC ─────────  PA4                 
PCLK ──────────  PA6                 
XCLK ──────────  PA8 (MCO1)         
SDA  ──────────  PF0 (I2C2)         
SCL  ──────────  PF1 (I2C2)         
RESET ─────────  PG0                 
PWDN ──────────  PG1                 
                 PA5  ────────────── SCK
                 PA7  ────────────── MOSI (SDA)
                 PE3  ────────────── CS
                 PE4  ────────────── DC (RS)
                 PE5  ────────────── RST
                 3.3V ────────────── LED (背光)
                 GND  ────────────── GND
```

---

## 12. 注意事項

1. **SCCB ≠ I2C**：OV7670 的 ACK 行為不符合 I2C 規範，需在韌體中處理 `HAL_I2C_ERROR_AF` 並加入 Retry
2. **D-Cache 一致性**：啟用 D-Cache 後，DMA 寫入的 Frame Buffer 需用 `SCB_InvalidateDCache_by_Addr()` 或設為 Non-Cacheable (MPU)
3. **Sync 極性**：目前實際穩定工作點為 `PCLK=Rising / VSYNC=High / HSYNC=Low`，需與 DCMI 設定一致
4. **PC11 DCMI_D4**：匯入 CubeMX 後請確認 AF13 正確配對
5. **PF0/PF1 (I2C2)**：NUCLEO 板上 Morpho connector 的這兩支 pin 確認沒有被其他 jumper 佔用
