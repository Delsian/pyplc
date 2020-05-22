/*
Copyright (c) 2020 Eug Krashtan

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#pragma once

#include "c_gpio.h"
#include <linux/spi/spidev.h>

#define _VERSION_ "3.4"
#define PYPLC_MAX_BLOCK_SIZE 160

/** SPI Header size. */
#define PDC_SPI_HEADER_SIZE				4
/** SPI Max Msg_Data size. */
#define PDC_SPI_MSG_DATA_SIZE			PYPLC_MAX_BLOCK_SIZE
/** SPI Max Msg_Data size. */
#define PDC_SPI_MSG_PARAMS_SIZE			118   /* Worst case = 118: sizeof(rx_msg_t) [G3] */
/** PDC buffer us_size. */
#define PDC_PPLC_BUFFER_SIZE			(PDC_SPI_HEADER_SIZE + PDC_SPI_MSG_DATA_SIZE + PDC_SPI_MSG_PARAMS_SIZE)
/** PDC buffer us_size to firmware update process */
#define PDC_SPI_FUP_BUFFER_SIZE        256
#define MAX_PLC_PKT_LEN PYPLC_MAX_BLOCK_SIZE

typedef enum {
	PLC_CMD_READ,
	PLC_CMD_WRITE
} plc_cmd_t;

#define PLC_WR_RD_POS							15
#define PLC_LEN_MASK							0x7FFF
#define PL360_REG_CMD_WR                        (1 << 10)
#define PL360_REG_LEN_MASK                      0x1FF
#define PL360_REG_ID_MASK                       0xF000
#define PL360_REG_OFFSET_MASK                   0x0FFF

/* -------- Register Definition -------- */
#define ATPL360_MISCR                               (0x400E1800U) /**< \brief (MISC) Miscelaneous Register */
#define ATPL360_RSTR                                (0x400E1804U) /**< \brief (RSTR) Reset Register */
#define ATPL360_SR                                  (0x400E1808U) /**< \brief (SR) Status Register */

/* -------- ATPL360_MISCR : Miscelaneous Register -------- */
#define ATPL360_MISCR_CPUWAIT                       (0x1u << 0) /**< \brief (ATPL360_MISCR) Cortex M7 Hold */
#define ATPL360_MISCR_PPM_CALIB_ON                  (0x1u << 8) /**< \brief (ATPL360_MISCR) PPM Calibration On */
#define ATPL360_MISCR_PPM_CALIB_OFF                 (0x0u << 8) /**< \brief (ATPL360_MISCR) PPM Calibration Off */
#define ATPL360_MISCR_MEM_128_64_CFG                (0x0u << 16) /**< \brief (ATPL360_MISCR) Memory configuration: 128kB ITCM - 64kB DTCM */
#define ATPL360_MISCR_MEM_96_96_CFG                 (0x1u << 16) /**< \brief (ATPL360_MISCR) Memory configuration: 96kB ITCM - 96kB DTCM */
#define ATPL360_MISCR_EN_ACCESS_ERROR               (0x1u << 24) /**< \brief (ATPL360_MISCR) Access Errors from CM7 enable */
#define ATPL360_MISCR_SET_GPIO_12_ZC                (0x0u << 25) /**< \brief (ATPL360_MISCR) Change GPIO ZeroCross: ZC by GPIO_12 */
#define ATPL360_MISCR_SET_GPIO_2_ZC                 (0x1u << 25) /**< \brief (ATPL360_MISCR) Change GPIO ZeroCross: ZC by GPIO_2 */
#define ATPL360_MISCR_SIGN_FAIL                     (0x1u << 26) /**< \brief (ATPL360_MISCR) Check fail in Signature check */

/* ! \name G3 Modulation types */
enum mod_types {
	MOD_TYPE_BPSK = 0,
	MOD_TYPE_QPSK = 1,
	MOD_TYPE_8PSK = 2,
	MOD_TYPE_QAM = 3,
	MOD_TYPE_BPSK_ROBO = 4,
};

/* ! \name G3 Modulation schemes */
enum mod_schemes {
	MOD_SCHEME_DIFFERENTIAL = 0,
	MOD_SCHEME_COHERENT = 1,
};

/* ! \name G3 Frame Delimiter Types */
enum delimiter_types {
	DT_SOF_NO_RESP = 0,
	DT_SOF_RESP = 1,
	DT_ACK = 2,
	DT_NACK = 3,
};

/* ! Internal Memory Map */
typedef enum atpl360_mem_id {
	ATPL360_STATUS_INFO_ID = 0,
	ATPL360_TX_PARAM_ID,
	ATPL360_TX_DATA_ID,
	ATPL360_TX_CFM_ID,
	ATPL360_RX_PARAM_ID,
	ATPL360_RX_DATA_ID,
	ATPL360_REG_INFO_ID,
	ATPL360_IDS,
} pl360_mem_id_t;

/* Defines relatives to configuration parameter (G3) */
typedef enum atpl360_reg_id {
	ATPL360_REG_PRODID = 0x4000,
	ATPL360_REG_MODEL,
	ATPL360_REG_VERSION_STR,
	ATPL360_REG_VERSION_NUM,
	ATPL360_REG_TONE_MASK,
	ATPL360_REG_TONE_MAP_RSP_DATA,
	ATPL360_REG_TX_TOTAL,
	ATPL360_REG_TX_TOTAL_BYTES,
	ATPL360_REG_TX_TOTAL_ERRORS,
	ATPL360_REG_TX_BAD_BUSY_TX,
	ATPL360_REG_TX_BAD_BUSY_CHANNEL,
	ATPL360_REG_TX_BAD_LEN,
	ATPL360_REG_TX_BAD_FORMAT,
	ATPL360_REG_TX_TIMEOUT,
	ATPL360_REG_RX_TOTAL,
	ATPL360_REG_RX_TOTAL_BYTES,
	ATPL360_REG_RX_RS_ERRORS,
	ATPL360_REG_RX_EXCEPTIONS,
	ATPL360_REG_RX_BAD_LEN,
	ATPL360_REG_RX_BAD_CRC_FCH,
	ATPL360_REG_RX_FALSE_POSITIVE,
	ATPL360_REG_RX_BAD_FORMAT,
	ATPL360_REG_ENABLE_AUTO_NOISE_CAPTURE,
	ATPL360_REG_TIME_BETWEEN_NOISE_CAPTURES,
	ATPL360_REG_DELAY_NOISE_CAPTURE_AFTER_RX,
	ATPL360_REG_RRC_NOTCH_ACTIVE,
	ATPL360_REG_RRC_NOTCH_INDEX,
	ATPL360_REG_NOISE_PEAK_POWER,
	ATPL360_REG_RSV0,
	ATPL360_REG_RSV1,
	ATPL360_REG_CFG_AUTODETECT_IMPEDANCE,
	ATPL360_REG_CFG_IMPEDANCE,
	ATPL360_REG_ZC_PERIOD,
	ATPL360_REG_FCH_SYMBOLS,
	ATPL360_REG_PAY_SYMBOLS_TX,
	ATPL360_REG_PAY_SYMBOLS_RX,
	ATPL360_REG_RRC_NOTCH_AUTODETECT,
	ATPL360_REG_MAX_RMS_TABLE_HI,
	ATPL360_REG_MAX_RMS_TABLE_VLO,
	ATPL360_REG_THRESHOLDS_TABLE_HI,
	ATPL360_REG_THRESHOLDS_TABLE_LO,
	ATPL360_REG_THRESHOLDS_TABLE_VLO,
	ATPL360_REG_PREDIST_COEF_TABLE_HI,
	ATPL360_REG_PREDIST_COEF_TABLE_LO,
	ATPL360_REG_PREDIST_COEF_TABLE_VLO,
	ATPL360_REG_GAIN_TABLE_HI,
	ATPL360_REG_GAIN_TABLE_LO,
	ATPL360_REG_GAIN_TABLE_VLO,
	ATPL360_REG_DACC_TABLE_CFG,
	ATPL360_REG_RSV2,
	ATPL360_REG_NUM_TX_LEVELS,
	ATPL360_REG_CORRECTED_RMS_CALC,
	ATPL360_REG_RRC_NOTCH_THR_ON,
	ATPL360_REG_RRC_NOTCH_THR_OFF,
	ATPL360_REG_CURRENT_GAIN,
	ATPL360_REG_ZC_CONF_INV,
	ATPL360_REG_ZC_CONF_FREQ,
	ATPL360_REG_ZC_CONF_DELAY,
	ATPL360_REG_NOISE_PER_CARRIER,
	ATPL360_REG_END_ID,
} pl360_reg_id_t;

/* ! Defines relatives to some ATPL360 registers */
#define ATPL360_REG_ADC_MASK                    0x1000
#define ATPL360_REG_DAC_MASK                    0x2000
#define ATPL360_REG_MASK                        0x4000
#define ATPL360_FUSES_MASK                      0x8000
#define ATPL360_REG_ADC_BASE                    0x40000000
#define ATPL360_REG_DAC_BASE                    0x40004000
#define ATPL360_REG_BASE                        0x80000000
#define ATPL360_FUSES_BASE                      0x400E1800

/* ! FLAG MASKs for set events */
#define ATPL360_TX_CFM_FLAG_MASK                 0x0001
#define ATPL360_RX_DATA_IND_FLAG_MASK            0x0002
#define ATPL360_CD_FLAG_MASK                     0x0004
#define ATPL360_REG_RSP_MASK                     0x0008
#define ATPL360_RX_QPAR_IND_FLAG_MASK            0x0010

/* ! Event Info MASKs */
#define ATPL360_EV_DAT_LEN_MASK                  0x0000FFFF
#define ATPL360_EV_REG_LEN_MASK                  0xFFFF0000
#define ATPL360_GET_EV_DAT_LEN_INFO(x)           ((uint32_t)x & ATPL360_EV_DAT_LEN_MASK)
#define ATPL360_GET_EV_REG_LEN_INFO(x)           (((uint32_t)x & ATPL360_EV_REG_LEN_MASK) >> 16)

#define PL360_IF_DEFAULT_TONE_MAP									{ 0x03, 0xFF, 0xFF }
#define PL360_IF_TX_POWER										0
/* ! TX Mode: Delayed transmission */
#define TX_MODE_RELATIVE             (1 << 1)
/* ! Maximum number of subbands */
#define NUM_SUBBANDS_MAX                       24
/* ! Maximum number of tone map */
#define TONE_MAP_SIZE_MAX                      3
/* ! Maximum number of protocol carriers */
#define PROTOCOL_CARRIERS_MAX                  72

typedef struct {
    uint16_t len;
    uint16_t addr;
    uint8_t buf[0];
} plc_pkt_t;

typedef void (*pl360_rx_cb)(plc_pkt_t *pkt);

typedef struct {
	PyObject_HEAD
	int fd;	/* open file descriptor: /dev/spidevX.Y */
	uint8_t mode;	/* current SPI mode */
	uint8_t bits_per_word;	/* current SPI bits per word setting */
	uint32_t max_speed_hz;	/* current SPI max speed setting in Hz */
	int cs; /* CS pin number */
    int ldo; /* LDO pin number */
    int rst; /* RST pin number */
	int irq; /* IRQ pin number */
	PyObject *rxcb; /* Callback to RX function */
	uint16_t events; /* events state from PL360 */
} PyPlcObject;

void pl360_reset(PyPlcObject *self);
void pl360_tx(PyPlcObject *self, uint8_t* buf, int len);
void pl360_datapkt(PyPlcObject *self, plc_cmd_t cmd, plc_pkt_t* pkt);
int pl360_init(PyPlcObject *self);
