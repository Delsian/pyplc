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

#define PLC_CMD_READ							0
#define PLC_CMD_WRITE							1

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
    int ldo; /* LDO pin number */
    int rst; /* RST pin number */
} PyPlcObject;

void pl360_reset(PyPlcObject *self);
int pl360_transfer(PyPlcObject *self, uint8_t* buf, int len);
