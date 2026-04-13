/**
 * @file    ov7670.c
 * @brief   OV7670 camera driver — SCCB + DCMI on STM32H7
 */
#include "ov7670.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/*  Private state                                                      */
/* ------------------------------------------------------------------ */
static I2C_HandleTypeDef  *cam_i2c;
static DCMI_HandleTypeDef *cam_dcmi;

#define SCCB_RETRIES    5
#define SCCB_TIMEOUT    50
#define OV7670_HSTART_BASE 0x16U
#define OV7670_HSTOP_BASE  0x04U

/* ---- Debug diagnostics (watch in Live Watch) ---- */
volatile struct {
    uint8_t  pid;               /* PID readback (expect 0x76) */
    uint8_t  ver;               /* VER readback (expect 0x73) */
    uint8_t  id_ok;             /* 1 = PID match */
    uint8_t  init_done;         /* 1 = register table sent */
    uint8_t  reg_verify_ok;     /* 1 = all key regs verified */
    uint8_t  reg_verify_fail;   /* count of readback mismatches */
    uint8_t  clkrc_rb;          /* CLKRC readback */
    uint8_t  com7_rb;           /* COM7 readback */
    uint8_t  com15_rb;          /* COM15 readback */
    uint8_t  com3_rb;           /* COM3 readback */
    uint8_t  com14_rb;          /* COM14 readback */
    uint8_t  com10_rb;          /* COM10 readback */
    uint32_t sccb_errors;       /* total SCCB write failures */
    uint32_t sccb_writes;       /* total SCCB writes attempted */
} cam_dbg;

/* ------------------------------------------------------------------ */
/*  SCCB (I2C with retry — OV7670 ACK quirk)                          */
/* ------------------------------------------------------------------ */
HAL_StatusTypeDef OV7670_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    HAL_StatusTypeDef status;

    for (int i = 0; i < SCCB_RETRIES; i++) {
        status = HAL_I2C_Master_Transmit(cam_i2c, OV7670_ADDR, buf, 2, SCCB_TIMEOUT);
        if (status == HAL_OK) return HAL_OK;
        /* Clear AF flag and retry */
        __HAL_I2C_CLEAR_FLAG(cam_i2c, I2C_FLAG_AF);
        cam_i2c->ErrorCode = 0;
        HAL_Delay(1);
    }
    return status;
}

HAL_StatusTypeDef OV7670_ReadReg(uint8_t reg, uint8_t *val)
{
    HAL_StatusTypeDef status;

    for (int i = 0; i < SCCB_RETRIES; i++) {
        status = HAL_I2C_Master_Transmit(cam_i2c, OV7670_ADDR, &reg, 1, SCCB_TIMEOUT);
        if (status != HAL_OK) {
            __HAL_I2C_CLEAR_FLAG(cam_i2c, I2C_FLAG_AF);
            cam_i2c->ErrorCode = 0;
            HAL_Delay(1);
            continue;
        }
        status = HAL_I2C_Master_Receive(cam_i2c, OV7670_ADDR | 1, val, 1, SCCB_TIMEOUT);
        if (status == HAL_OK) return HAL_OK;
        __HAL_I2C_CLEAR_FLAG(cam_i2c, I2C_FLAG_AF);
        cam_i2c->ErrorCode = 0;
        HAL_Delay(1);
    }
    return status;
}

/* ------------------------------------------------------------------ */
/*  OV7670 register table — QQVGA RGB565                               */
/* ------------------------------------------------------------------ */
typedef struct { uint8_t reg; uint8_t val; } RegVal_t;

static const RegVal_t ov7670_qvga_rgb565[] = {
    /* ---- Basic / format ---- */
    {0x12, 0x80},   /* COM7: software reset */
    {0xFF, 100},     /* delay 100 ms (sentinel) */

    {0x12, 0x14},   /* COM7: QVGA + RGB output */
    {0x11, 0x01},   /* CLKRC: prescaler = 2 */
    {0x3A, 0x04},   /* TSLB: auto output window */
    {0x40, 0xD0},   /* COM15: RGB565, full output range */
    {0x8C, 0x00},   /* RGB444: disabled */
    {0x04, 0x00},   /* COM1: no CCIR656 */

    /* ---- QQVGA scaling (VGA → QVGA via COM7, then /2 manual) ---- */
    {0x0C, 0x04},   /* COM3: enable scaling */
    {0x3E, 0x19},   /* COM14: manual scaling, PCLK divider = /2 */
    {0x70, 0x3A},   /* SCALING_XSC */
    {0x71, 0x35},   /* SCALING_YSC */
    {0x72, 0x11},   /* SCALING_DCWCTR: H/2 + V/2 */
    {0x73, 0xF1},   /* SCALING_PCLK_DIV */
    {0xA2, 0x02},   /* SCALING_PCLK_DELAY */

    /* ---- Sync / window ---- */
    {0x15, 0x00},   /* COM10: HREF mode, default sync polarity */
    /* Safe horizontal window baseline.  The later centering tweak moved
     * the start/stop window and likely skewed the RGB565 byte phase,
     * producing a geometrically correct image with wrong colours. */
    {0x17, OV7670_HSTART_BASE},   /* HSTART */
    {0x18, OV7670_HSTOP_BASE},    /* HSTOP  */
    {0x32, 0x24},   /* HREF */
    {0x19, 0x02},   /* VSTART */
    {0x1A, 0x7A},   /* VSTOP */
    {0x03, 0x0A},   /* VREF */

    /* ---- AEC / AGC / AWB ---- */
    {0x13, 0xE0},   /* COM8: disable all auto */
    {0x00, 0x00},   /* GAIN */
    {0x10, 0x00},   /* AECH */
    {0x0D, 0x40},   /* COM4 */
    {0x14, 0x18},   /* COM9: AGC ceiling = 4x */
    {0x24, 0x95},   /* AEW: AGC/AEC stable upper limit */
    {0x25, 0x33},   /* AEB: AGC/AEC stable lower limit */
    {0x26, 0xE3},   /* VPT: fast mode threshold */
    {0x9F, 0x78},   /* HAECC1 */
    {0xA0, 0x68},   /* HAECC2 */
    {0xA1, 0x03},   /* Reserved */
    {0xA6, 0xD8},   /* HAECC3 */
    {0xA7, 0xD8},   /* HAECC4 */
    {0xA8, 0xF0},   /* HAECC5 */
    {0xA9, 0x90},   /* HAECC6 */
    {0xAA, 0x94},   /* HAECC7 */
    {0x13, 0xE5},   /* COM8: enable AWB + AEC + AGC */

    /* ---- Colour matrix (high saturation, reduce G cross-talk) ---- */
    {0x4F, 0xC0}, {0x50, 0x90}, {0x51, 0x00},   /* MTX1-3: boost R, cut G-to-R */
    {0x52, 0x33}, {0x53, 0xB0}, {0x54, 0xE3},   /* MTX4-6 */
    {0x58, 0x9E},   /* MTXS */

    /* ---- UV saturation / colour control ---- */
    {0x3D, 0xC0},   /* COM13: gamma enable, no UV swap */
    {0xC9, 0x80},   /* Saturation control (higher = more vivid) */

    /* ---- Gamma curve ---- */
    {0x7A, 0x20},
    {0x7B, 0x10}, {0x7C, 0x1E}, {0x7D, 0x35},
    {0x7E, 0x5A}, {0x7F, 0x69}, {0x80, 0x76},
    {0x81, 0x80}, {0x82, 0x88}, {0x83, 0x8F},
    {0x84, 0x96}, {0x85, 0xA3}, {0x86, 0xAF},
    {0x87, 0xC4}, {0x88, 0xD7}, {0x89, 0xE8},

    /* ---- Misc ---- */
    {0x3D, 0xC0},   /* COM12 */
    {0xB0, 0x84},   /* Reserved / color bar test disabled */
    {0x0E, 0x61},   /* COM5 */
    {0x0F, 0x4B},   /* COM6 */
    {0x16, 0x02},   /* Reserved */
    {0x1E, 0x37},   /* MVFP: mirror (bit5) + vertical flip (bit4) */
    {0x21, 0x02},   /* ADCCTR1 */
    {0x22, 0x91},   /* ADCCTR2 */
    {0x29, 0x07},   /* RSVD */
    {0x33, 0x0B},   /* CHLF */
    {0x35, 0x0B},   /* RSVD */
    {0x37, 0x1D},   /* ADC */
    {0x38, 0x71},   /* ACOM */
    {0x39, 0x2A},   /* OFON */
    {0x3C, 0x78},   /* COM12 */
    {0x4D, 0x40},   /* RSVD */
    {0x4E, 0x20},   /* RSVD */
    {0x69, 0x00},   /* GFIX */
    {0x74, 0x10},   /* REG74 */
    {0x8D, 0x4F},   /* RSVD */
    {0x8E, 0x00},   /* RSVD */
    {0x8F, 0x00},   /* RSVD */
    {0x90, 0x00},   /* RSVD */
    {0x91, 0x00},   /* RSVD */
    {0x96, 0x00},   /* RSVD */
    {0x9A, 0x00},   /* RSVD */
    {0xB1, 0x0C},   /* ABLC1 */
    {0xB2, 0x0E},   /* RSVD */
    {0xB3, 0x82},   /* THL_ST */
    {0xB8, 0x0A},   /* RSVD */

    {0xFF, 0xFF},   /* END sentinel */
};

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */
void OV7670_HardwareReset(void)
{
    OV7670_PWDN_OFF();
    HAL_Delay(10);
    OV7670_RST_LOW();
    HAL_Delay(20);
    OV7670_RST_HIGH();
    HAL_Delay(100);
}

void OV7670_Init(I2C_HandleTypeDef *hi2c, DCMI_HandleTypeDef *hdcmi)
{
    uint8_t rb = 0;
    cam_i2c  = hi2c;
    cam_dcmi = hdcmi;
    memset((void *)&cam_dbg, 0, sizeof(cam_dbg));

    OV7670_HardwareReset();

    /* Send register table */
    for (uint32_t i = 0; ov7670_qvga_rgb565[i].reg != 0xFF ||
                          ov7670_qvga_rgb565[i].val != 0xFF; i++) {
        if (ov7670_qvga_rgb565[i].reg == 0xFF) {
            /* Delay entry */
            HAL_Delay(ov7670_qvga_rgb565[i].val);
        } else {
            cam_dbg.sccb_writes++;
            HAL_StatusTypeDef s = OV7670_WriteReg(ov7670_qvga_rgb565[i].reg,
                                                   ov7670_qvga_rgb565[i].val);
            if (s != HAL_OK) cam_dbg.sccb_errors++;
            HAL_Delay(1);
        }
    }
    cam_dbg.init_done = 1;

    /* Readback key registers to verify configuration */
    uint8_t fails = 0;
    OV7670_ReadReg(0x11, &rb); cam_dbg.clkrc_rb = rb;   /* expect 0x01 */
    OV7670_ReadReg(0x12, &rb); cam_dbg.com7_rb = rb;    /* expect 0x14 */
    OV7670_ReadReg(0x40, &rb); cam_dbg.com15_rb = rb;   /* expect 0xD0 */
    OV7670_ReadReg(0x0C, &rb); cam_dbg.com3_rb = rb;    /* expect 0x04 */
    OV7670_ReadReg(0x3E, &rb); cam_dbg.com14_rb = rb;   /* expect 0x19 */
    OV7670_ReadReg(0x15, &rb); cam_dbg.com10_rb = rb;   /* expect 0x00 */

    if (cam_dbg.clkrc_rb != 0x01) fails++;
    if (cam_dbg.com7_rb  != 0x14) fails++;
    if (cam_dbg.com15_rb != 0xD0) fails++;
    if (cam_dbg.com3_rb  != 0x04) fails++;
    if (cam_dbg.com14_rb != 0x19) fails++;

    cam_dbg.reg_verify_fail = fails;
    cam_dbg.reg_verify_ok   = (fails == 0) ? 1 : 0;
}

bool OV7670_ReadID(uint8_t *pid, uint8_t *ver)
{
    uint8_t p = 0, v = 0;
    HAL_StatusTypeDef s1 = OV7670_ReadReg(OV7670_REG_PID, &p);
    HAL_StatusTypeDef s2 = OV7670_ReadReg(OV7670_REG_VER, &v);

    cam_dbg.pid = p;
    cam_dbg.ver = v;
    cam_dbg.id_ok = (s1 == HAL_OK && s2 == HAL_OK && p == 0x76) ? 1 : 0;

    if (pid) *pid = p;
    if (ver) *ver = v;

    return (cam_dbg.id_ok == 1);
}

bool OV7670_StartCapture(uint32_t *dst_buf)
{
    if (cam_dcmi == NULL) {
        return false;
    }

    HAL_StatusTypeDef s = HAL_DCMI_Start_DMA(
        cam_dcmi, DCMI_MODE_CONTINUOUS,
        (uint32_t)dst_buf,
        CAM_FRAME_SIZE / 4   /* length in 32-bit words */
    );
    return (s == HAL_OK);
}

void OV7670_StopCapture(void)
{
    if (cam_dcmi != NULL) {
        HAL_DCMI_Stop(cam_dcmi);
    }
}

bool OV7670_SetColorBar(bool enable)
{
    uint8_t com7 = 0;
    uint8_t com17 = 0;
    uint8_t rb = 0;

    if (OV7670_ReadReg(0x12, &com7) != HAL_OK) {
        return false;
    }
    if (OV7670_ReadReg(0x42, &com17) != HAL_OK) {
        return false;
    }

    if (enable) {
        com7 |= 0x02U;
        com17 |= 0x08U;
    } else {
        com7 &= (uint8_t)~0x02U;
        com17 &= (uint8_t)~0x08U;
    }

    if (OV7670_WriteReg(0x12, com7) != HAL_OK) {
        return false;
    }
    if (OV7670_WriteReg(0x42, com17) != HAL_OK) {
        return false;
    }

    HAL_Delay(2);
    if (OV7670_ReadReg(0x12, &rb) != HAL_OK) {
        return false;
    }
    cam_dbg.com7_rb = rb;
    if (((rb & 0x02U) != 0U) != enable) {
        return false;
    }
    if (OV7670_ReadReg(0x42, &rb) != HAL_OK) {
        return false;
    }
    if (((rb & 0x08U) != 0U) != enable) {
        return false;
    }

    return true;
}

bool OV7670_SetOutputByteSwap(bool enable)
{
    uint8_t com3 = 0;

    if (OV7670_ReadReg(0x0C, &com3) != HAL_OK) {
        return false;
    }

    if (enable) {
        com3 |= 0x40U;
    } else {
        com3 &= (uint8_t)~0x40U;
    }

    if (OV7670_WriteReg(0x0C, com3) != HAL_OK) {
        return false;
    }

    if (OV7670_ReadReg(0x0C, &com3) != HAL_OK) {
        return false;
    }

    cam_dbg.com3_rb = com3;
    return (((com3 & 0x40U) != 0U) == enable);
}

bool OV7670_SetPixelShift(uint8_t shift)
{
    if (OV7670_WriteReg(0x1B, shift) != HAL_OK) {
        return false;
    }

    return true;
}

bool OV7670_SetHorizontalWindowShift(int8_t steps)
{
    int16_t hstart = (int16_t)OV7670_HSTART_BASE + steps;
    int16_t hstop = (int16_t)OV7670_HSTOP_BASE + steps;
    uint8_t rb = 0;

    if (hstart < 0 || hstart > 0xFF || hstop < 0 || hstop > 0xFF) {
        return false;
    }

    if (OV7670_WriteReg(0x17, (uint8_t)hstart) != HAL_OK) {
        return false;
    }
    if (OV7670_WriteReg(0x18, (uint8_t)hstop) != HAL_OK) {
        return false;
    }

    if (OV7670_ReadReg(0x17, &rb) != HAL_OK || rb != (uint8_t)hstart) {
        return false;
    }
    if (OV7670_ReadReg(0x18, &rb) != HAL_OK || rb != (uint8_t)hstop) {
        return false;
    }

    return true;
}

bool OV7670_SetSharpness(bool manual, uint8_t edge)
{
    uint8_t com16 = 0;
    uint8_t rb = 0;

    if (OV7670_ReadReg(0x41, &com16) != HAL_OK) {
        return false;
    }

    if (manual) {
        com16 &= (uint8_t)~0x20U;
    } else {
        com16 |= 0x20U;
    }

    if (OV7670_WriteReg(0x41, com16) != HAL_OK) {
        return false;
    }

    if (manual) {
        if (OV7670_WriteReg(0x3F, (uint8_t)(edge & 0x1FU)) != HAL_OK) {
            return false;
        }
        if (OV7670_ReadReg(0x3F, &rb) != HAL_OK || (rb & 0x1FU) != (edge & 0x1FU)) {
            return false;
        }
    }

    return true;
}

bool OV7670_SetDenoise(bool manual, uint8_t threshold)
{
    uint8_t com16 = 0;
    uint8_t rb = 0;

    if (OV7670_ReadReg(0x41, &com16) != HAL_OK) {
        return false;
    }

    if (manual) {
        com16 &= (uint8_t)~0x10U;
    } else {
        com16 |= 0x10U;
    }

    if (OV7670_WriteReg(0x41, com16) != HAL_OK) {
        return false;
    }

    if (manual) {
        if (OV7670_WriteReg(0x4C, threshold) != HAL_OK) {
            return false;
        }
        if (OV7670_ReadReg(0x4C, &rb) != HAL_OK || rb != threshold) {
            return false;
        }
    }

    return true;
}

bool OV7670_SnapshotCapture(uint32_t *dst_buf)
{
    if (cam_dcmi == NULL) {
        return false;
    }

    HAL_StatusTypeDef s = HAL_DCMI_Start_DMA(
        cam_dcmi, DCMI_MODE_SNAPSHOT,
        (uint32_t)dst_buf,
        CAM_FRAME_SIZE / 4
    );
    return (s == HAL_OK);
}
