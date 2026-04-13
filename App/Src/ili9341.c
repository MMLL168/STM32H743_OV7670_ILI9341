/**
 * @file    ili9341.c
 * @brief   ILI9341 LCD driver — SPI + DMA on STM32H7
 */
#include "ili9341.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/*  Private state                                                      */
/* ------------------------------------------------------------------ */
static SPI_HandleTypeDef *lcd_spi;
static volatile bool      dma_busy;

/* ---- Debug diagnostics (watch in Live Watch) ---- */
volatile struct {
    HAL_StatusTypeDef last_spi_status;  /* last SPI transmit result */
    uint32_t          tx_count;         /* total SPI transmit calls */
    uint32_t          tx_errors;        /* SPI transmit error count */
    uint8_t           cs_pin_state;     /* PE3 readback */
    uint8_t           dc_pin_state;     /* PE4 readback */
    uint8_t           rst_pin_state;    /* PE5 readback */
    uint8_t           init_done;        /* 1 = LCD_Init completed */
    uint8_t           last_cmd;         /* last command sent */
    uint8_t           read_id[4];       /* ILI9341 ID readback (0xD3) */
    uint8_t           read_ok;          /* 1=ID match ILI9341 */
} lcd_dbg;

/* ------------------------------------------------------------------ */
/*  Bit-bang read ID — temporarily reconfigure PA5/PA7 as GPIO         */
/*  ILI9341 0xD3 "Read ID4" → dummy + 0x00 + 0x93 + 0x41             */
/* ------------------------------------------------------------------ */
#define BB_SCK_PORT  GPIOA
#define BB_SCK_PIN   GPIO_PIN_5
#define BB_SDA_PORT  GPIOA
#define BB_SDA_PIN   GPIO_PIN_7

static void BB_Delay(void)
{
    for (volatile int i = 0; i < 8; i++) __NOP();
}

static void BB_SendByte(uint8_t byte)
{
    /* Configure SDA as output */
    GPIO_InitTypeDef g = {0};
    g.Pin   = BB_SDA_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BB_SDA_PORT, &g);

    for (int i = 7; i >= 0; i--) {
        HAL_GPIO_WritePin(BB_SDA_PORT, BB_SDA_PIN,
                          (byte >> i) & 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        BB_Delay();
        HAL_GPIO_WritePin(BB_SCK_PORT, BB_SCK_PIN, GPIO_PIN_SET);
        BB_Delay();
        HAL_GPIO_WritePin(BB_SCK_PORT, BB_SCK_PIN, GPIO_PIN_RESET);
        BB_Delay();
    }
}

static uint8_t BB_ReadByte(void)
{
    /* Configure SDA as input */
    GPIO_InitTypeDef g = {0};
    g.Pin   = BB_SDA_PIN;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BB_SDA_PORT, &g);

    uint8_t val = 0;
    for (int i = 7; i >= 0; i--) {
        HAL_GPIO_WritePin(BB_SCK_PORT, BB_SCK_PIN, GPIO_PIN_SET);
        BB_Delay();
        if (HAL_GPIO_ReadPin(BB_SDA_PORT, BB_SDA_PIN)) val |= (1 << i);
        HAL_GPIO_WritePin(BB_SCK_PORT, BB_SCK_PIN, GPIO_PIN_RESET);
        BB_Delay();
    }
    return val;
}

static void LCD_ReadID_BitBang(void)
{
    /* Disable SPI so we can use PA5/PA7 as GPIO */
    HAL_SPI_DeInit(lcd_spi);

    /* SCK as GPIO output, idle low */
    GPIO_InitTypeDef g = {0};
    g.Pin   = BB_SCK_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BB_SCK_PORT, &g);
    HAL_GPIO_WritePin(BB_SCK_PORT, BB_SCK_PIN, GPIO_PIN_RESET);

    /* Send command 0xD3 (Read ID4) */
    LCD_DC_CMD();
    LCD_CS_LOW();
    BB_SendByte(0xD3);

    /* Switch to data read mode */
    LCD_DC_DATA();

    /* Read 4 bytes: 1 dummy + 3 ID bytes */
    lcd_dbg.read_id[0] = BB_ReadByte();  /* dummy */
    lcd_dbg.read_id[1] = BB_ReadByte();  /* 0x00 */
    lcd_dbg.read_id[2] = BB_ReadByte();  /* 0x93 expected */
    lcd_dbg.read_id[3] = BB_ReadByte();  /* 0x41 expected */
    LCD_CS_HIGH();

    /* Check: ILI9341 should return 0x93, 0x41 */
    lcd_dbg.read_ok = (lcd_dbg.read_id[2] == 0x93 && lcd_dbg.read_id[3] == 0x41) ? 1 : 0;

    /* Restore SPI */
    HAL_SPI_Init(lcd_spi);
}

/* ------------------------------------------------------------------ */
/*  Low-level SPI helpers                                              */
/* ------------------------------------------------------------------ */
static void LCD_WriteCmd(uint8_t cmd)
{
    LCD_DC_CMD();
    LCD_CS_LOW();
    HAL_StatusTypeDef s = HAL_SPI_Transmit(lcd_spi, &cmd, 1, 10);
    LCD_CS_HIGH();
    lcd_dbg.last_spi_status = s;
    lcd_dbg.last_cmd = cmd;
    lcd_dbg.tx_count++;
    if (s != HAL_OK) lcd_dbg.tx_errors++;
}

static void LCD_WriteData8(const uint8_t *data, uint32_t len)
{
    LCD_DC_DATA();
    LCD_CS_LOW();
    HAL_StatusTypeDef s = HAL_SPI_Transmit(lcd_spi, (uint8_t *)data, (uint16_t)len, 100);
    LCD_CS_HIGH();
    lcd_dbg.last_spi_status = s;
    lcd_dbg.tx_count++;
    if (s != HAL_OK) lcd_dbg.tx_errors++;
}

static void LCD_WriteCmdData(uint8_t cmd, const uint8_t *data, uint32_t len)
{
    LCD_WriteCmd(cmd);
    if (len > 0) {
        LCD_WriteData8(data, len);
    }
}

/* ------------------------------------------------------------------ */
/*  Init sequence (table-driven)                                       */
/* ------------------------------------------------------------------ */
#define DELAY_FLAG 0x80

typedef struct {
    uint8_t cmd;
    uint8_t len;   /* bit 7 = delay after, bits 6:0 = data length */
    uint8_t data[15];
} LCD_InitStep_t;

static const LCD_InitStep_t init_seq[] = {
    {0xCB, 5,  {0x39,0x2C,0x00,0x34,0x02}},             /* Power Control A */
    {0xCF, 3,  {0x00,0xC1,0x30}},                         /* Power Control B */
    {0xE8, 3,  {0x85,0x00,0x78}},                         /* Driver Timing A */
    {0xEA, 2,  {0x00,0x00}},                               /* Driver Timing B */
    {0xED, 4,  {0x64,0x03,0x12,0x81}},                    /* Power On Seq */
    {0xF7, 1,  {0x20}},                                    /* Pump Ratio */
    {0xC0, 1,  {0x23}},                                    /* Power Control 1 */
    {0xC1, 1,  {0x10}},                                    /* Power Control 2 */
    {0xC5, 2,  {0x3E,0x28}},                               /* VCOM Control 1 */
    {0xC7, 1,  {0x86}},                                    /* VCOM Control 2 */
    {0x36, 1,  {0xE8}},                                    /* MADCTL: landscape + BGR + flip Y+X */
    {0x3A, 1,  {0x55}},                                    /* Pixel Format: 16-bit */
    {0xB1, 2,  {0x00,0x18}},                               /* Frame Rate: 79 Hz */
    {0xB6, 3,  {0x08,0x82,0x27}},                         /* Display Function */
    {0xF2, 1,  {0x00}},                                    /* 3-Gamma OFF */
    {0x26, 1,  {0x01}},                                    /* Gamma Curve 1 */
    {0xE0, 15, {0x0F,0x31,0x2B,0x0C,0x0E,0x08,0x4E,
                0xF1,0x37,0x07,0x10,0x03,0x0E,0x09,0x00}}, /* Positive Gamma */
    {0xE1, 15, {0x00,0x0E,0x14,0x03,0x11,0x07,0x31,
                0xC1,0x48,0x08,0x0F,0x0C,0x31,0x36,0x0F}}, /* Negative Gamma */
    {0x11, DELAY_FLAG, {0}},                                /* Sleep Out + 120ms */
    {0x29, DELAY_FLAG, {0}},                                /* Display ON + 20ms */
};

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */
void LCD_Init(SPI_HandleTypeDef *hspi)
{
    lcd_spi  = hspi;
    dma_busy = false;
    memset((void *)&lcd_dbg, 0, sizeof(lcd_dbg));

    /* Hardware reset */
    LCD_RST_HIGH();
    HAL_Delay(5);
    LCD_RST_LOW();
    HAL_Delay(20);
    LCD_RST_HIGH();
    HAL_Delay(150);

    /* Snapshot GPIO states after reset */
    lcd_dbg.cs_pin_state  = (uint8_t)HAL_GPIO_ReadPin(LCD_CS_GPIO_Port,  LCD_CS_Pin);
    lcd_dbg.dc_pin_state  = (uint8_t)HAL_GPIO_ReadPin(LCD_DC_GPIO_Port,  LCD_DC_Pin);
    lcd_dbg.rst_pin_state = (uint8_t)HAL_GPIO_ReadPin(LCD_RST_GPIO_Port, LCD_RST_Pin);

    /* ---- Read ILI9341 ID via bit-bang (diagnostic) ---- */
    LCD_ReadID_BitBang();

    /* Send init commands */
    for (uint32_t i = 0; i < sizeof(init_seq) / sizeof(init_seq[0]); i++) {
        uint8_t len = init_seq[i].len & 0x7F;
        LCD_WriteCmdData(init_seq[i].cmd, init_seq[i].data, len);
        if (init_seq[i].len & DELAY_FLAG) {
            HAL_Delay(init_seq[i].cmd == 0x11 ? 120 : 20);
        }
    }
    lcd_dbg.init_done = 1;
}

void LCD_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t ca[4] = {x0 >> 8, x0 & 0xFF, x1 >> 8, x1 & 0xFF};
    uint8_t ra[4] = {y0 >> 8, y0 & 0xFF, y1 >> 8, y1 & 0xFF};
    LCD_WriteCmdData(0x2A, ca, 4);  /* Column Address Set */
    LCD_WriteCmdData(0x2B, ra, 4);  /* Row Address Set */
    LCD_WriteCmd(0x2C);             /* Memory Write */
}

void LCD_FillColor(uint16_t color)
{
    LCD_SetWindow(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);

    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;
    uint8_t buf[640];
    for (uint32_t i = 0; i < sizeof(buf); i += 2) {
        buf[i]     = hi;
        buf[i + 1] = lo;
    }

    LCD_DC_DATA();
    LCD_CS_LOW();
    uint32_t total = (uint32_t)LCD_WIDTH * LCD_HEIGHT * 2;
    while (total > 0) {
        uint32_t chunk = (total > sizeof(buf)) ? sizeof(buf) : total;
        HAL_SPI_Transmit(lcd_spi, buf, (uint16_t)chunk, 200);
        total -= chunk;
    }
    LCD_CS_HIGH();
}

void LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT) return;
    if (x + w > LCD_WIDTH)  w = LCD_WIDTH - x;
    if (y + h > LCD_HEIGHT) h = LCD_HEIGHT - y;

    LCD_SetWindow(x, y, x + w - 1, y + h - 1);

    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;
    uint8_t buf[640];
    for (uint32_t i = 0; i < sizeof(buf); i += 2) {
        buf[i]     = hi;
        buf[i + 1] = lo;
    }

    LCD_DC_DATA();
    LCD_CS_LOW();
    uint32_t total = (uint32_t)w * h * 2;
    while (total > 0) {
        uint32_t chunk = (total > sizeof(buf)) ? sizeof(buf) : total;
        HAL_SPI_Transmit(lcd_spi, buf, (uint16_t)chunk, 200);
        total -= chunk;
    }
    LCD_CS_HIGH();
}

void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT) return;
    LCD_SetWindow(x, y, x, y);
    uint8_t d[2] = {color >> 8, color & 0xFF};
    LCD_DC_DATA();
    LCD_CS_LOW();
    HAL_SPI_Transmit(lcd_spi, d, 2, 10);
    LCD_CS_HIGH();
}

/* ---- DMA transfer ---- */
void LCD_SendData_DMA(const uint8_t *data, uint32_t len)
{
    LCD_WaitDMA();
    dma_busy = true;
    LCD_DC_DATA();
    LCD_CS_LOW();
    HAL_SPI_Transmit_DMA(lcd_spi, (uint8_t *)data, (uint16_t)len);
}

void LCD_WaitDMA(void)
{
    while (dma_busy) {
        __WFI();
    }
}

bool LCD_IsDMABusy(void)
{
    return dma_busy;
}

void LCD_SendData(const uint8_t *data, uint32_t len)
{
    LCD_DC_DATA();
    LCD_CS_LOW();
    HAL_SPI_Transmit(lcd_spi, (uint8_t *)data, (uint16_t)len, 500);
    LCD_CS_HIGH();
}

void LCD_DMA_TxCpltCallback(void)
{
    LCD_CS_HIGH();
    dma_busy = false;
}
