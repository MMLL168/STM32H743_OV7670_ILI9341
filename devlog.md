# Development Log

## 索引 (Index)

| # | 日期時間 | 摘要 |
|---|---------|------|
| 1 | 2026-04-09 | 完成全部韌體架構：ILI9341 驅動、OV7670 驅動、GUI 覆蓋層、應用狀態機 |
| 2 | 2026-04-09 | 新增 README.md 專案文件 |
| 3 | 2026-04-09 | 新增 debug 設定：launch.json、tasks.json、openocd.cfg |
| 4 | 2026-04-09 | 啟用 MPU Non-Cacheable，移除手動 cache 操作 |
| 5 | 2026-04-09 | LCD 顯示修正：MADCTL 上下翻轉、字體放大至 2x |
| 6 | 2026-04-09 | 時鐘修正：HSE BYPASS 失敗，改用 HSI；PLL DIVM1=/8, DIVN1=x120 |
| 7 | 2026-04-10 | LCD 白屏偵錯：新增 lcd_dbg 診斷結構、bit-bang ID 讀取、確認 MOSI 接線錯誤 |
| 8 | 2026-04-10 | OV7670 花屏偵錯：新增 cam_dbg 診斷、SCCB 暫存器回讀驗證 |
| 9 | 2026-04-10 | 啟用 USART3 VCP (921600 baud)、UART frame dump 功能 |
| 10 | 2026-04-10 | DCMI 擷取偵錯：M2 改 continuous 模式、UART 輸出 DCMI/DMA 暫存器狀態 |
| 11 | 2026-04-13 | 花屏根因：RGB565 byte order 修正；Debug GUI 改 Big-Endian 解析；auto sweep 結束恢復 PSHFT=0 |
| 12 | 2026-04-13 | 百葉窗條紋修正：DCMI Crop Window 限制每行精確 320 bytes × 120 lines |
| 13 | 2026-04-13 | OV7670 畫面方向與置中：MVFP mirror+vflip、HSTART/HSTOP 偏移校正 |
| 14 | 2026-04-13 | Debug GUI 排版修正：按鈕自動換行、視窗可縮放不截斷 |
| 15 | 2026-04-13 | M6 追蹤狀態改善：LCD status 簡化、UART 送 [TRACK] 資料、GUI 畫 overlay |
| 16 | 2026-04-13 | 紅色追蹤閾值可調：GUI 滑桿即時調參、Apply to FW 同步韌體、GUI-side 偵測模式 |
| 17 | 2026-04-13 | 預覽清晰度提升：改為 QVGA 320x240 1:1 顯示，新增 GUI 即時抓圖與 sharpness/denoise 手動調整 |
| 18 | 2026-04-13 | 紅色偵測閾值收緊：IsRedPixel565 加 G/B 上限與 2x 比率門檻 |
| 19 | 2026-04-13 | TrackRedObject byte order 修正：ARM little-endian 讀 uint16 需先 swap |
| 20 | 2026-04-13 | GUI 偵測診斷工具：Show Mask、點擊查看像素 RGB/HSV、FW Pixel Dump |
| 21 | 2026-04-13 | GUI 偵測改用 HSV 色彩空間：解決 OV7670 色偏導致 RGB 閾值無效的問題 |
| 22 | 2026-04-13 | OV7670 色彩矩陣調校：更新 MTX1-6 與飽和度，改善紅色表現 |
| 23 | 2026-04-14 | 效能瓶頸量測與加速：加入 [WIN] perf、F0 overlay 節流、OV7670 XCLK 24MHz 穩定點、LCD SPI runtime toggle |
| 24 | 2026-04-14 | Debug GUI 加滾動軸與 Camera Tuning 滑桿：Canvas scrollable wrapper、OV7670 register 滑桿 |
| 25 | 2026-04-14 | Debug GUI 訊息欄改善：Snapshot 區加滾動軸、Log 區加 Clear 按鈕 |
| 26 | 2026-04-14 | 關閉 AWB 改手動白平衡：B=0x60 R=0x40 G=0x40，修正偏黃問題 |
| 27 | 2026-04-14 | 修正滑桿調參當機：W 命令改 deferred write，GUI 端加 500ms 節流 |
| 28 | 2026-04-14 | PB15 Servo PWM 輸出：TIM12_CH2 400Hz 1000~2000us，紅色 X 映射角度 0~359° |

---
## 1. 完成全部韌體架構 — 2026-04-09

**原因 (Why)**
依照 proposal.md 規格書，實作 STM32H743ZI2 + OV7670 (DCMI) + ILI9341 (SPI) 即時影像預覽系統，涵蓋 M0~M5 所有里程碑。

**處理方式 (How)**

### 新增檔案（不受 CubeMX 覆蓋）
| 檔案 | 說明 |
|------|------|
| `App/Inc/ili9341.h` | ILI9341 LCD 驅動標頭 — SPI + DMA |
| `App/Src/ili9341.c` | ILI9341 實作：初始化序列、視窗設定、填色、DMA 傳輸 |
| `App/Inc/ov7670.h` | OV7670 攝影機驅動標頭 — SCCB + DCMI |
| `App/Src/ov7670.c` | OV7670 實作：SCCB retry、QQVGA RGB565 暫存器表、連續/快照擷取 |
| `App/Inc/gui.h` | GUI 覆蓋層標頭 — 文字、狀態列、資訊框 |
| `App/Src/gui.c` | GUI 實作：5x7 字型渲染、狀態列、啟動畫面、測試結果顯示 |
| `App/Inc/app.h` | 應用控制器標頭 — 狀態機、幀管線 |
| `App/Src/app.c` | 應用實作：M0~M5 狀態機、Ping-Pong 雙緩衝、2x 縮放、灰階/邊緣偵測 |
| `App/Inc/font5x7.h` | 5x7 點陣字型 (ASCII 32-127) |

### 修改既有檔案（僅在 USER CODE 區塊內）
| 檔案 | 修改內容 |
|------|---------|
| `Core/Src/main.c` | `#include "app.h"` + `App_Init()` + `App_Run()` |
| `Core/Src/stm32h7xx_it.c` | 加入 `DCMI_IRQHandler`、`HAL_DCMI_FrameEventCallback`、`HAL_SPI_TxCpltCallback` |
| `STM32H743XX_FLASH.ld` | 新增 `.axi_sram` section → RAM (0x24000000) 放 frame buffer |
| `CMakeLists.txt` | 加入 `App/Src/*.c` 和 `App/Inc` include path |

### 架構重點
- **D-Cache 安全**：DCMI DMA 完成後 `SCB_InvalidateDCache`，SPI DMA 傳輸前 `SCB_CleanDCache`
- **SCCB 容錯**：OV7670 ACK 不規範，I2C 通訊失敗時自動 retry 最多 5 次
- **自動化流程**：上電後自動依序執行 Splash → M0 純色 → M1 SCCB → M2 單幀 → M3 即時 → M4 雙緩衝 → M5 灰階/邊緣
- **CubeMX 相容**：所有修改都在 `USER CODE BEGIN/END` 區塊內，App 程式碼獨立於 `App/` 目錄

---

## 2. 新增 README.md 專案文件 — 2026-04-09

**原因 (Why)**
提供完整的專案說明，包含硬體接線、架構說明、編譯燒錄方式、里程碑流程、關鍵技術設計。

**處理方式 (How)**
新增 `README.md`，涵蓋：硬體需求、接線圖、專案結構、CubeMX 相容性說明、時鐘/記憶體配置、自動執行流程、GUI 介面說明、D-Cache/SCCB/縮放/雙緩衝等技術設計、編譯燒錄指令、注意事項。

---

## 3. 新增 debug 設定 — 2026-04-09

**原因 (Why)**
專案缺少 VSCode debug 設定，無法單步執行與即時監控變數。參照 H743_MPU6050 專案已驗證可用的設定。

**處理方式 (How)**
- 新增 `openocd.cfg`：ST-Link DAP + SWD、`STOP_WATCHDOG 1` 防止 Examination failed、`reset-deassert-post` halt
- 新增 `.vscode/launch.json`：Cortex Debug (OpenOCD)、`liveWatch` 即時監控、SWO ITM console、`cpuFrequency` 修正為 480MHz
- 新增 `.vscode/tasks.json`：`CMake Build` preLaunchTask，按 F5 自動編譯再燒錄

---

## 4. 啟用 MPU Non-Cacheable，移除手動 cache 操作 — 2026-04-09

**原因 (Why)**
CubeMX 中啟用 MPU，將 AXI SRAM (0x24000000, 256KB) 設為 Non-Cacheable (TEX=1, C=0, B=0, S=1)。DMA 和 CPU 資料一致性由 MPU 硬體保證，不再需要手動 cache 維護。

**處理方式 (How)**
- CubeMX 設定 MPU Region 0 → 重新 Generate Code → `MPU_Config()` 自動生成於 `main.c`
- 移除 `app.c` 中所有 `SCB_InvalidateDCache_by_Addr()` (4 處) 和 `SCB_CleanDCache_by_Addr()` (3 處)
- 移除 `ili9341.c` 中 `LCD_SendData_DMA()` 的 `SCB_CleanDCache_by_Addr()` (1 處)
- 驗證 App/ 目錄下零 cache 操作殘留

---

## 5. LCD 顯示修正：MADCTL 翻轉 + 字體放大 — 2026-04-09

**原因 (Why)**
LCD 影像上下顛倒，字體太小不易辨識。

**處理方式 (How)**
- `ili9341.c`：MADCTL 從 0x28 改為 0xE8（landscape + BGR + flip Y+X）
- `gui.c`：FONT_SCALE 改為 2，字元大小從 6×8 變為 12×16

---

## 6. 時鐘修正：HSE → HSI — 2026-04-09

**原因 (Why)**
NUCLEO-H743ZI2 的 HSE 由 ST-Link MCO 提供，需焊接 SB148 才能使用。HSE BYPASS 模式啟動失敗，`HAL_RCC_OscConfig()` 進入 Error_Handler。

**處理方式 (How)**
- CubeMX 改用 HSI (64 MHz) 作為 PLL 來源
- PLL 設定：DIVM1=/8 (8 MHz input)、DIVN1=x120、DIVP1=/2 → SYSCLK=480 MHz
- MCO1 改為 HSI/8 = 8 MHz 提供 OV7670 XCLK

---

## 7. LCD 白屏偵錯 — 2026-04-10

**原因 (Why)**
LCD 初始化完成但白屏，lcd_dbg 顯示 SPI 通訊正常（335898 tx, 0 errors），bit-bang ID 讀取回傳全 0xFF。

**處理方式 (How)**
- `ili9341.c`：新增 `lcd_dbg` 診斷結構（SPI 狀態、GPIO 狀態、ID 回讀）
- `ili9341.c`：新增 `LCD_ReadID_BitBang()` 透過 bit-bang 讀取 ILI9341 ID (0xD3)
- `.vscode/settings.json`：Live Watch 加入 lcd_dbg
- **結果**：確認使用者 MOSI 接線錯誤，修正後 LCD 正常

---

## 8. OV7670 花屏偵錯：cam_dbg 診斷 — 2026-04-10

**原因 (Why)**
OV7670 影像顯示為花屏雜訊，需確認 SCCB 暫存器是否正確寫入。

**處理方式 (How)**
- `ov7670.c`：新增 `cam_dbg` 診斷結構，含 PID/VER、SCCB 錯誤計數、關鍵暫存器回讀值
- `ov7670.c`：OV7670_Init 結束後回讀 CLKRC/COM7/COM15/COM3/COM14/COM10 並比對預期值
- `app.c`：M1 畫面顯示 `RegChk:OK/FAIL` 及 SCCB 錯誤率
- `.vscode/settings.json`：Live Watch 加入 cam_dbg、hdcmi.State、hdcmi.ErrorCode
- **結果**：cam_dbg 全部正確（PID=0x76, reg_verify_ok=1, sccb_errors=0），排除 SCCB 問題

---

## 9. 啟用 USART3 VCP + UART frame dump — 2026-04-10

**原因 (Why)**
需要將 OV7670 原始 frame buffer 傳到 PC 分析，判斷花屏是 DCMI 資料問題還是 LCD 顯示問題。

**處理方式 (How)**
- CubeMX 啟用 USART3 (PD8 TX / PD9 RX)，921600 baud（ST-Link VCP）
- `app.c`：開機發送 UART 測試訊息確認通訊
- `app.c`：新增 `DumpFrameToUART()`，以 "FRAME" header + raw RGB565 + "END" 格式傳送
- `app.c`：M2 擷取完成後自動 dump frame buffer 到 UART
- `tools/frame_viewer.py`：Python 接收工具，自動解析 RGB565 並顯示（含 byte-swap 版本比對）

---

## 10. DCMI 擷取偵錯：continuous 模式 + 暫存器追蹤 — 2026-04-10

**原因 (Why)**
DCMI SR 顯示有 HSYNC/VSYNC 信號但 frame callback 未觸發（TIMEOUT）。Snapshot 模式對 VSYNC 極性敏感，需排查極性與 DMA 狀態。

**處理方式 (How)**
- `dcmi.c` USER CODE：曾嘗試覆寫 VSYNC 極性（FALLING/HIGH），均未解決
- `app.c`：M2 改用 continuous 模式取代 snapshot（更穩定）
- `app.c`：UART 輸出 DCMI SR/CR 暫存器、DMA NDTR/PAR/M0AR、每幀前 32 bytes hex、non-zero byte 統計
- `app.c`：等待 3 幀後才停止並 dump，避免第一幀不完整
- **進行中**：SR=0x00000001 表示有 HSYNC 但 FIFO 空，待進一步排查 PCLK 或 D0-D7 接線

---

## 11. 花屏根因：RGB565 Byte Order 修正 — 2026-04-13

**原因 (Why)**
Debug GUI 顯示花屏，但 auto_sweep shift_00 的 "Byte Swapped" 變體呈現完美色條（White/Yellow/Cyan/Green/Magenta）。分析結果：

1. **D0-D7 硬體接線正確** — 色條圖案正確證明 bit 順序無誤
2. **PCLK = 1MHz 正確** — XCLK=8MHz, CLKRC=/2=4MHz, COM14 PCLK/2=2MHz, SCALING_PCLK_DIV/2=1MHz
3. **OV7670 輸出 Big-Endian RGB565** — 先送 [R|G_hi]，再送 [G_lo|B]
4. **Python viewer 以 Little-Endian 解讀** — `px = byte0 | (byte1 << 8)` 導致 RGB 通道錯亂
5. **COM3[6] byte swap 在此 OV7670 模組無效** — 設了沒效果，常見於翻版晶片
6. **LCD 顯示路徑其實正確** — SPI 按記憶體順序送 bytes，與 ILI9341 期望的 Big-Endian 一致
7. **Auto sweep 結束後 PSHFT=4 未還原** — 後續幀全部花屏

**處理方式 (How)**
- `tools/debug_gui.py`：`rgb565_to_image()` 改為 Big-Endian 解析 `px = (byte0 << 8) | byte1`
- `tools/debug_gui.py`：變體標籤改為 "Normal (BE)" / "Byte Swapped (LE)"
- `tools/debug_gui.py`：`finish_auto_sweep()` 結束後送 `z` 指令恢復 PSHFT=0
- `App/Src/app.c`：`z` 指令改為 `sensor_byte_swap=false`（COM3[6] 無效，不再依賴）

---

## 12. 百葉窗條紋修正：DCMI Crop Window — 2026-04-13

**原因 (Why)**
LCD 畫面有明顯的水平「百葉窗」條紋（alternating dark/bright lines）。根因：OV7670 QQVGA 模式每行多輸出 1 個 byte（321 bytes 而非 320 bytes）。DCMI DMA 連續寫入不分行，導致：

- 偶數行：byte 對齊 → 顏色正確
- 奇數行：偏移 1 byte → MSB/LSB 錯位 → 顏色錯亂
- 交替出現 → 百葉窗效果

**處理方式 (How)**
- `App/Src/app.c`：新增 `ConfigureDcmiCrop()`，使用 STM32 DCMI 的 Crop Window 功能
  - `HAL_DCMI_ConfigCrop(&hdcmi, 0, 0, 319, 119)` = 精確擷取 320 bytes/line × 120 lines
  - 多餘的 byte 由硬體自動丟棄，確保每行 pixel 對齊
- 覆蓋所有 4 處 `HAL_DCMI_Init` 呼叫點（ApplyDcmiSync, D0 probe, M2 probe, ProbeDcmiCapture）
- **結果**：百葉窗條紋完全消除

---

## 13. OV7670 畫面方向與置中校正 — 2026-04-13

**原因 (Why)**
1. LCD 畫面上下顛倒
2. LCD 畫面左右鏡像（鏡頭對尺的 4，LCD 顯示為 6）
3. 畫面水平偏移（鏡頭對尺的 2，LCD 顯示為 1）

**處理方式 (How)**
- `App/Src/ov7670.c`：MVFP (0x1E) 從 `0x07` 改為 `0x37`
  - bit 5 = 1：水平鏡像（修正左右顛倒）
  - bit 4 = 1：垂直翻轉（修正上下顛倒）
- `App/Src/ov7670.c`：HSTART/HSTOP 水平窗口偏移 +18 校正置中
  - HSTART: `0x16` → `0x28`（+18 units）
  - HSTOP: `0x04` → `0x16`（+18 units，保持 640px VGA 窗口大小不變）
- **微調方式**：若仍偏移，HSTART/HSTOP 同步加減即可

---

## 14. Debug GUI 排版修正 — 2026-04-13

**原因 (Why)**
視窗拉大拉小時，Firmware Control 區域的 20 個按鈕全部排在同一行（grid row=0, column 0..19），超出視窗寬度就被截斷看不到。第二行的 Auto Sweep / Live Grab 控制項也因固定 column 位置而溢出。

**處理方式 (How)**
- `tools/debug_gui.py`：按鈕改用 flow-wrap 排版
  - 監聽容器 `<Configure>` 事件，根據可用寬度自動計算每行能放幾個按鈕，動態 re-grid
  - 第二行（Auto Sweep + Live Grab）改用 `pack(side="left")` 自然排列
- 預設視窗改為 1280x900，設定 minsize(800, 600) 避免過小
- 底部 diag_text 高度從 7 縮為 5、log_text 從 14 縮為 10，讓圖片預覽區有更多空間
- status_bar 改為只有 log_text (row=2) 有 weight=1，避免 diag 區不必要地佔用垂直空間

---

## 15. M6 追蹤狀態改善 — 2026-04-13

**原因 (Why)**
- LCD status bar 上 `M6: Red x=160 y=109` 和 FPS 數字黏在一起難以辨識
- GUI App 只能看到原始影像，無法即時看到追蹤偵測結果

**處理方式 (How)**
- `App/Src/app.c`：LCD status 簡化為 `M6: (cx,cy)`，避免與 FPS 混淆
- `App/Src/app.c`：M6 每幀後透過 UART 送 `[TRACK] cx cy count min_x min_y max_x max_y`（或 `[TRACK] none`）
- `tools/debug_gui.py`：解析 `[TRACK]` 訊息，右下角顯示 tracking 狀態文字
- `tools/debug_gui.py`：主預覽畫面疊加綠色 bounding box、青色十字準星、黃色座標標籤

---

## 16. 紅色追蹤閾值可調 — 2026-04-13

**原因 (Why)**
原本 `IsRedPixel565` 閾值寫死在韌體中，偵測到太多假紅色（棕色、膚色），且無法在不重燒韌體的情況下調整。

**處理方式 (How)**
- `App/Src/app.c`：`IsRedPixel565` 改用 runtime 變數 `red_r_min`, `red_g_max`, `red_b_max`, `red_min_px`
- `App/Src/app.c`：新增 `T` UART 命令，格式 `T<r>,<g>,<b>,<n>\n`，即時更新追蹤閾值
- `tools/debug_gui.py`：新增 "Red Tracking Threshold" 控制面板
  - 四個 Spinbox：R min (0-31)、G max (0-31)、B max (0-31)、Min px (0-500)
  - "GUI-side detect" 勾選框：啟用時在 PC 端用 raw frame 做偵測，滑桿調整即時反映
  - "Apply to FW" 按鈕：將目前閾值透過 `T` 命令同步到韌體

---

## 17. 預覽清晰度提升：QVGA 1:1 顯示 + GUI 影像調參 — 2026-04-13

**原因 (Why)**
- 現場預覽雖然已可穩定顯示，但仍明顯模糊。
- 根因不是單一寄存器，而是整體預覽路徑仍停留在 `QQVGA 160x120`，再用 2x 最近鄰放大到 `320x240` LCD。
- 使用者也需要一個更直觀的 PC 端工具，能同時看影像、調參、持續抓圖，而不是只靠單次 dump。

**處理方式 (How)**
- `App/Inc/ov7670.h`
  - `CAM_WIDTH / CAM_HEIGHT` 從 `160x120` 改為 `320x240`
  - `CAM_FRAME_SIZE` 隨之更新為 `320 * 240 * 2 = 153600 bytes`
- `App/Src/ov7670.c`
  - 將初始化表切到 `QVGA RGB565`
  - `COM7` 改為 `0x14`（QVGA + RGB）
  - 驗證讀回值 `cam_dbg.com7_rb` 也同步改為檢查 `0x14`
- `App/Src/app.c`
  - `TransferFrameToLCD()`、`TransferFrameToLCD_Grayscale()`、`TransferFrameToLCD_Edge()` 改為依 `CAM_WIDTH/CAM_HEIGHT` 自動映射到 LCD，而不再寫死 2x 複製
  - `ConfigureDcmiCrop()` 保持用 `CAM_WIDTH * 2` 與 `CAM_HEIGHT` 自動配置，讓 QVGA 擷取與顯示一致
  - `DrawTrackingOverlay()` 改為用 `LCD_WIDTH / CAM_WIDTH`、`LCD_HEIGHT / CAM_HEIGHT` 計算座標，不再依賴舊的 `*2` 假設
  - baseline 仍維持現場驗證過的安全值 `HSHIFT=4`
  - `z` 指令也改成回到 `HSHIFT=4`，避免自動掃描後預覽偏回不安全位置
- `App/Src/gui.c`
  - Splash 畫面解析度文字從 `QQVGA 160x120` 更新為 `QVGA 320x240`
- `tools/debug_gui.py`
  - 保留四種 frame decode 檢視：`Normal / Byte Swapped / Word Swapped / Word+Byte Swapped`
  - 新增主預覽區 `Camera View`，可選擇主顯示的 decode 模式
  - 新增 `Start Live Grab`，定時送 `u` 指令持續抓 frame，在 PC 端形成近即時預覽
  - 新增 `Interval ms` 控制 live grab 週期
  - 保留並整合既有影像調參按鈕：`Sharp-/Sharp+`、`Noise-/Noise+`、`H-/H+`、`Shift-/+`、`Byte Swap`、`Sensor Swap`

**結果 (Result)**
- 韌體預覽路徑從「低解析度放大」改為 `320x240` 直接顯示，這是目前最有感的清晰度改善。
- GUI 端從單次 dump 工具提升成可持續抓圖、切換 decode、搭配手動影像參數調整的 debug 工具。
- 專案重新編譯成功，記憶體使用量仍在安全範圍內：
  - `RAM (AXI SRAM)` 約 `308480 / 512 KB`
  - `DTCMRAM` 約 `74872 / 128 KB`

---

## 18. 紅色偵測閾值收緊 — 2026-04-13

**原因 (Why)**
原始 `IsRedPixel565` 閾值太寬鬆（`r >= 10`, 只比較 `r > g5 + 3`），導致棕色、膚色、暖灰色全部被判為紅色。TRACK 資料顯示 bounding box 為整個畫面 `(0,0)-(319,239)`，n=14000+ 假紅像素。

**處理方式 (How)**
- `App/Src/app.c`：`IsRedPixel565` 閾值收緊
  - R 最低：10 → 16（~210/255）
  - 新增 G5 上限 ≤12、B 上限 ≤12
  - 比率：`r > g5 + 3` → `r > g5 * 2` 且 `r > b * 2`

---

## 19. TrackRedObject byte order 修正 — 2026-04-13

**原因 (Why)**
`TrackRedObject` 直接用 `uint16_t*` 讀 frame buffer，但 OV7670 via DCMI 寫入的是 big-endian bytes。ARM Cortex-M7 是 little-endian，`uint16_t` 讀出的值 byte 順序是反的，導致 R/G/B 位元欄位解錯。LCD 顯示正常是因為 SPI 直接送 raw bytes（恰好也是 BE），不經 CPU 解析。

**處理方式 (How)**
- `App/Src/app.c`：`TrackRedObject` 迴圈內讀取 `px` 後加上 byte swap
  ```c
  px = (uint16_t)((px << 8) | (px >> 8));
  ```

---

## 20. GUI 偵測診斷工具 — 2026-04-13

**原因 (Why)**
修正 byte order 和閾值後韌體追蹤仍顯示 `Red not found`，需要更精確的診斷工具來確認實際像素值與偵測邏輯。

**處理方式 (How)**
- `tools/debug_gui.py`：新增三個診斷功能
  1. **Show Mask** 勾選框：紅色像素以紅色高亮顯示，其他像素變暗，視覺化偵測結果
  2. **點擊查看像素**：點擊主預覽畫面顯示該點的 RGB565 raw 值、RGB888 近似值、HSV 值，以及是否通過紅色判定
  3. **FW Pixel Dump** 按鈕（對應韌體 `D` 命令）：在 9 個取樣點印出 raw bytes 與兩種 byte order 的 RGB 值，並統計全幀紅色像素數（有 swap vs 無 swap）
- `App/Src/app.c`：新增 `'D'` UART 命令處理器，輸出 `[PIXDUMP]` 診斷資料

---

## 21. GUI 偵測改用 HSV 色彩空間 — 2026-04-13

**原因 (Why)**
透過 pixel inspector 發現 OV7670 拍攝的紅圓中心像素為 RGB=(238,250,189)，G 通道比 R 還高。這表示 OV7670 色彩嚴重失準（過曝+白平衡偏移），任何基於「R 必須是最大通道」的 RGB 閾值法都無法偵測到紅色。

**處理方式 (How)**
- `tools/debug_gui.py`：GUI-side 偵測從 RGB 閾值改為 HSV 色彩空間
  - 新增 `_rgb_to_hsv()` 靜態方法
  - 新增 `_is_red_hsv()` 方法：紅色 Hue 在 340°~360° 或 0°~30°（環繞 0°）
  - 滑桿改為 H lo (340)、H hi (30)、S min (30)、V min (50)、Min px (60)
  - Pixel inspector 改為顯示 RGB + HSV + 是否通過判定
  - `_detect_red_local` 全面改用 HSV 判定
- 韌體端暫時維持 RGB 閾值（HSV 計算在 MCU 上較耗時），GUI-side 先驗證 HSV 可行性

---

## 22. OV7670 色彩矩陣校正 — 2026-04-13

**原因 (Why)**
OV7670 預設色彩矩陣 MTX1-6 = `{0x80, 0x80, 0x00, 0x22, 0x5E, 0x80}` 導致紅色通道增益不足，拍出來的紅色偏向白/黃/綠。

**處理方式 (How)**
- `App/Src/ov7670.c`：更新色彩矩陣為高飽和度版本
  - MTX1-6：`{0xB3, 0xB3, 0x00, 0x3D, 0xA7, 0xE4}`（OV7670 Application Note 推薦值）
  - 新增 `0xC9 = 0x60`（Saturation control，提高色彩飽和度）
- 預期效果：紅色更飽和，HSV 偵測更容易在 H=0° 附近找到紅色像素
---
---

## 23. 效能瓶頸量測與穩定加速點 — 2026-04-14

**原因 (Why)**
F0 live preview 長時間停在 4.9~5.1 FPS 左右，需要停止猜測，分開量出 camera capture、LCD transfer、GUI overlay 三段的真瓶頸。同時也需要一個能安全調高 OV7670 與 ILI9341 clock 的流程，避免一次改太多變數導致系統一起失穩。

**處理方式 (How)**
- `App/Src/app.c`
  - QVGA 1:1 preview 路徑改為 `LCD_SendFrameChunkedDMA()`，使用 chunked SPI DMA 傳整張 frame。
  - Grayscale / edge preview 路徑改為 `LCD_SendData_DMA()` line-by-line，移除 blocking SPI transmit。
  - F0 新增 perf counters，透過 `w` / `[WIN]` snapshot 輸出：
    - `xfer_avg_ms`
    - `overlay_avg_ms`
    - `capture_avg_ms`
  - F0 overlay 改為節流更新：status bar 週期性更新，info box 只在 entry / dirty / freeze warning 時重畫。
  - 新增 runtime LCD SPI prescaler 控制：
    - `8` = LCD SPI `/8`
    - `4` = LCD SPI `/4`
- `tools/debug_gui.py`
  - 補回 `Latest Window / Freeze Snapshot` 與 `[WIN]` parser。
  - 新增 GUI 按鈕：`LCD SPI /8`、`LCD SPI /4`。
- `Core/Src/main.c`
  - OV7670 XCLK 做分階段測試，最後確認目前穩定加速點為 `MCO1 = PLL1Q / 8 = 24MHz`。
- `Core/Src/spi.c`
  - LCD SPI baseline 保持 `/8`；`/4` 改為 runtime 測試，不再直接強推成預設值。

**量測結果 (Measured results)**
- 初始 perf 量測：
  - `fps=2.6`
  - `xfer_avg_ms=51`
  - `overlay_avg_ms=76`
  - `capture_avg_ms=391`
- 套用 DMA preview path + overlay 節流後：
  - `fps=5.1`
  - `xfer_avg_ms=51`
  - `overlay_avg_ms=4`
  - `capture_avg_ms=196`
- 目前穩定加速點（`OV7670 XCLK = 24MHz`、`LCD SPI = /8`）：
  - `fps=7.6`
  - `xfer_avg_ms=51`
  - `overlay_avg_ms=3`
  - `capture_avg_ms=131`

**結論 (Findings)**
1. `thread` 不是有效解法；本專案跑在單核心 H743，主瓶頸不是 task scheduling。
2. GUI overlay 原本意外地很重，但節流後從約 `76 ms` 降到約 `3~4 ms`。
3. LCD transfer 仍是第二瓶頸，約 `51 ms/frame`。
4. 目前第一瓶頸已收斂為 OV7670 capture timing。把 XCLK 從 `16MHz` 提到 `24MHz` 後，`capture_avg_ms` 由約 `196` 降到約 `131`，整體 preview FPS 由約 `5.1` 提到約 `7.6`。
5. 同時把 `OV7670 XCLK` 與 `LCD SPI` 都往上推會失穩，所以後續調校流程改為：sensor clock 與 LCD clock 分開驗證。

**目前最佳穩定配置 (Current best-known stable config)**
- `OV7670 XCLK = 24MHz`
- `DCMI sync = PCLK Rising / VSYNC High / HSYNC Low`
- `ILI9341 SPI = /8`
- `QVGA 320x240 1:1 preview`
- F0 overlay throttled
- GUI 可顯示 `[WIN]` perf snapshot

**下一步 (Next steps)**
- 長時間 soak-test runtime `LCD SPI /4`
- 在確認 `24MHz XCLK` 穩定後，再繼續調整 OV7670 divider path（`CLKRC / COM14 / SCALING_PCLK_DIV`）

---
## 24. Debug GUI 加滾動軸與 Camera Tuning 滑桿 — 2026-04-14

**原因 (Why)**
加入 HSV 滑桿、Camera Tuning 滑桿後，GUI 面板過多導致視窗高度超出螢幕，預覽畫面被壓縮。需要整體滾動功能與攝影機參數即時調整。

**處理方式 (How)**
- `tools/debug_gui.py`
  - `_build_ui()` 改用 Canvas + Scrollbar scrollable wrapper：所有子面板放在 `self._inner` Frame 內，外層 Canvas 提供垂直/水平滾動軸。
  - 新增 `_on_inner_configure()`、`_on_canvas_configure()`、`_on_mousewheel()` 事件處理。
  - 新增 Camera Tuning (OV7670) LabelFrame，包含 9 組滑桿：R Gain(0x02)、B Gain(0x01)、Saturation(0xC9)、Brightness(0x55)、Contrast(0x56)、MTX1-2(0x4F-0x50)、MTX5(0x53)、MTX6(0x54)。
  - 滑桿拖動即時透過 `W<reg>,<val>` UART 命令寫入 OV7670 暫存器。

---
## 25. Debug GUI 訊息欄改善 — 2026-04-14

**原因 (Why)**
Snapshot 訊息欄內容過長時無法捲動查看；UART Log 累積大量訊息後不易閱讀，需要清除功能。

**處理方式 (How)**
- `tools/debug_gui.py`
  - Snapshot 訊息欄（Latest Window / Freeze Snapshot）右側新增垂直 Scrollbar。
  - UART Log 區上方新增標題列 "UART Log" 與 **Clear** 按鈕，點擊清空所有 log 文字。
  - 新增 `_clear_log()` 方法。

---
## 26. 關閉 AWB 改手動白平衡 — 2026-04-14

**原因 (Why)**
OV7670 AWB 不準確，畫面整體偏黃（紅+綠過多、藍不足）。AWB 跑在 sensor 內部 DSP，關閉不影響 FPS，但可避免色溫飄移，有利紅色追蹤穩定性。

**處理方式 (How)**
- `App/Src/ov7670.c`
  - `COM8` 從 `0xE5` 改為 `0xE3`（保留 AEC+AGC，關閉 AWB bit2）
  - 新增手動白平衡暫存器：`BLUE(0x01)=0x60`、`RED(0x02)=0x40`、`GGAIN(0x6A)=0x40`
- `tools/debug_gui.py`
  - Camera Tuning 新增 G Gain 滑桿 (reg 0x6A)
  - R/B Gain 預設值更新為韌體對應值

---
## 27. 修正滑桿調參導致系統當機 — 2026-04-14

**原因 (Why)**
拖動 GUI Camera Tuning 滑桿時系統卡死在 `SPI_WaitOnFlagUntilTimeout`。根因：`PollFieldControlCommand()` 在主迴圈每次被呼叫，`W` 命令直接執行 `OV7670_WriteReg()`（I2C SCCB），而此時 LCD SPI DMA 傳輸可能正在進行或 SPI 狀態不穩定，導致下一次 SPI 操作 hang。加上 GUI 端滑桿拖動每像素都送一筆 `W` 命令，大量命令湧入加劇了問題。

**處理方式 (How)**
- `App/Src/app.c`
  - `W` 命令改為 deferred write：僅設定 `ov_write_pending` / `ov_write_reg` / `ov_write_val`
  - 新增 `ProcessDeferredOvWrite()`：在 frame 間隙（LCD 傳輸前）安全執行實際的 I2C 寫入
  - 在 `State_F0_LivePreview`、`State_M5_Grayscale`、`State_M6_RedTracking` 各插入呼叫點
- `tools/debug_gui.py`
  - `_on_cam_slider()` 加入 500ms 節流（`after` + `after_cancel`），避免拖動時每像素都送命令
  - `ProcessDeferredOvWrite()` 移至 `frame_ready` 區塊內、LCD SPI 完成之後才執行
  - 加入 SPI READY / I2C READY 雙重檢查 + 200ms 最小間隔

---
## 28. PB15 Servo PWM 輸出 — 2026-04-14

**原因 (Why)**
需要將紅色物體的水平位置（畫面 X 軸 0~319）映射為 0~359° 角度，透過 400Hz PWM (1000~2000us pulse) 送給 B-G431B-ESC1 開發板，驅動跟隨馬達轉到對應角度。

**處理方式 (How)**
- `Core/Inc/stm32h7xx_hal_conf.h`
  - 啟用 `HAL_TIM_MODULE_ENABLED`
- `App/Inc/servo_pwm.h` + `App/Src/servo_pwm.c` (新增)
  - TIM12_CH2 on PB15 (AF2)
  - PSC=239, ARR=2499 → 400Hz (APB1 timer clock 240MHz / 240 / 2500)
  - `Servo_Init()`: 初始化 TIM12、GPIO PB15、啟動 PWM
  - `Servo_SetAngle(deg)`: 0~359° → CCR 1000~2000
  - `Servo_SetPulseUs(us)`: 直接設定 pulse width
- `App/Src/app.c`
  - `App_Init()` 呼叫 `Servo_Init()`
  - `State_M6_RedTracking()`: 偵測到紅色時計算 `angle = cx * 359 / 319`，呼叫 `Servo_SetAngle(angle)`
  - LCD status bar 顯示 `M6: (x,y) Ndeg`
  - UART log 增加 `ang=N` 欄位
- `CMakeLists.txt`
  - 加入 `App/Src/servo_pwm.c`
- `tools/debug_gui.py`
  - `_TRACK_RE` 增加 `ang=(\d+)` optional capture
  - tracking overlay 顯示角度資訊

**接線 (Wiring)**
- NUCLEO-H743ZI2 PB15 (CN12 pin) → B-G431B-ESC1 PWM input
- 共地 GND
