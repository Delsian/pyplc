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

#include <Python.h>
#include "pyplc.h"
#include <unistd.h> 
#include <sys/ioctl.h>
#include <linux/ioctl.h>

/* ! Reset pin of transceiver */
#define PLC_RST_DELAY                (100)
/** SPI Header field when bootload is in the other side of spi*/
#define PLC_SPI_HEADER_BOOT					0x5634
/** SPI Header MASK for bootloader heade*/
#define PLC_SPI_HEADER_BOOT_MASK				0xFFFe
/** SPI Header field when atpl360 is in the other side of spi*/
#define PLC_SPI_HEADER_CORTEX					0x1022
/** Bootloader Passwords for enable writing  */
#define ATPL360_BOOT_WRITE_KEY              0x5345ACBA
/** Bootloader Address of CPUWAIT */
#define ATPL360_BOOT_CMD_ENABLE_WRITE       0xDE05
/** Bootloader Address for writing program */
#define ATPL360_BOOT_PROGRAM_ADDR           0x00000000
/** Bootloader command: Write Word (32 bits) */
#define ATPL360_BOOT_CMD_WRITE_WORD         0x0000
/** Bootloader command: Write buffer */
#define ATPL360_BOOT_CMD_WRITE_BUF          0x0001
/** Bootloader command: Disable SPI control to bootloader */
#define ATPL360_BOOT_CMD_DIS_SPI_CTRL       0xA55A

/* PDC Receive buffer */
static uint8_t plc_rx_buffer[PDC_PPLC_BUFFER_SIZE];
/* PDC Transmission buffer */
static uint8_t plc_tx_buffer[PDC_PPLC_BUFFER_SIZE];

#include "boot/boot.h"

struct spi_ioc_transfer xfer = {
    .bits_per_word = 8,
    .delay_usecs = 0,
    .speed_hz = 1000000,
    .cs_change = 0
};

void pl360_reset(PyPlcObject *self) {
    Py_BEGIN_ALLOW_THREADS
	output_gpio(self->ldo, 0);
    usleep(100000);
    output_gpio(self->rst, 0);
    output_gpio(self->cs, 1);
    output_gpio(self->ldo, 1);

	xfer.len = 1;
    xfer.rx_buf = (unsigned long)&plc_rx_buffer[0];
    int err = ioctl(self->fd, SPI_IOC_MESSAGE(1), &xfer);
	if(err<0) {
		PyErr_SetFromErrno(PyExc_IOError);
	}

	usleep(10000);
    output_gpio(self->rst, 1);
    usleep(10000);
    Py_END_ALLOW_THREADS
}

static void pl360_boot_pkt(PyPlcObject *self, uint16_t cmd, uint32_t addr, uint16_t data_len, uint8_t *data_buf) {
    Py_BEGIN_ALLOW_THREADS
	memcpy(plc_tx_buffer, &addr, sizeof(uint32_t));
	memcpy(&(plc_tx_buffer[4]), &cmd, sizeof(uint16_t));
	memcpy(&(plc_tx_buffer[6]), data_buf, data_len);

	xfer.len = data_len + 6;
    xfer.rx_buf = (unsigned long)&plc_rx_buffer[0];
    xfer.tx_buf = (unsigned long)&plc_tx_buffer[0];
    output_gpio(self->cs, 0);
	int err = ioctl(self->fd, SPI_IOC_MESSAGE(1), &xfer);
    output_gpio(self->cs, 1);
	if(err<0) {
		PyErr_SetFromErrno(PyExc_IOError);
	}
    Py_END_ALLOW_THREADS
}

static void pl360_booload(PyPlcObject *self) {
	uint8_t buf[4];
	pl360_reset(self);

	buf[3] = (uint8_t)(ATPL360_BOOT_WRITE_KEY >> 24);
	buf[2] = (uint8_t)(ATPL360_BOOT_WRITE_KEY >> 16);
	buf[1] = (uint8_t)(ATPL360_BOOT_WRITE_KEY >> 8);
	buf[0] = (uint8_t)(ATPL360_BOOT_WRITE_KEY);
	pl360_boot_pkt(self, ATPL360_BOOT_CMD_ENABLE_WRITE, 0, sizeof(buf), buf);

	buf[3] = (uint8_t)(ATPL360_BOOT_WRITE_KEY >> 8);
	buf[2] = (uint8_t)(ATPL360_BOOT_WRITE_KEY);
	buf[1] = (uint8_t)(ATPL360_BOOT_WRITE_KEY >> 24);
	buf[0] = (uint8_t)(ATPL360_BOOT_WRITE_KEY >> 16);
	pl360_boot_pkt(self, ATPL360_BOOT_CMD_ENABLE_WRITE, 0, sizeof(buf), buf);

/* Send CPU Wait Cmd */
 	uint32_t reg_value = ATPL360_MISCR_CPUWAIT | ATPL360_MISCR_PPM_CALIB_OFF | ATPL360_MISCR_MEM_96_96_CFG |
						 ATPL360_MISCR_EN_ACCESS_ERROR | ATPL360_MISCR_SET_GPIO_12_ZC;
	buf[3] = (uint8_t)(reg_value >> 24);
	buf[2] = (uint8_t)(reg_value >> 16);
	buf[1] = (uint8_t)(reg_value >> 8);
	buf[0] = (uint8_t)(reg_value);
	pl360_boot_pkt(self, ATPL360_BOOT_CMD_WRITE_WORD, ATPL360_MISCR, sizeof(buf), buf);

	// Load FW
	uint32_t fw_pending_len = sizeof(bootcode);
	uint8_t *fw_data = (uint8_t *)bootcode;
	uint32_t fw_prog_addr  = ATPL360_BOOT_PROGRAM_ADDR;
	uint16_t fw_fragment_len;
	while (fw_pending_len) {
		if (fw_pending_len > PDC_SPI_FUP_BUFFER_SIZE) {
			fw_fragment_len = PDC_SPI_FUP_BUFFER_SIZE;
			fw_pending_len -= fw_fragment_len;
		} else {
			fw_fragment_len = fw_pending_len;
			fw_pending_len = 0;
			fw_fragment_len += (4 - (fw_fragment_len & 3)) & 3; // padding
		}

		/* Write fw block data */
		pl360_boot_pkt(self, ATPL360_BOOT_CMD_WRITE_BUF, fw_prog_addr, fw_fragment_len, fw_data);

		/* Update counters */
		fw_data += fw_fragment_len;
		fw_prog_addr += fw_fragment_len;
	}

	// Disable CpuWait
	reg_value = ATPL360_MISCR_PPM_CALIB_OFF | ATPL360_MISCR_MEM_96_96_CFG | ATPL360_MISCR_EN_ACCESS_ERROR | ATPL360_MISCR_SET_GPIO_12_ZC;
	buf[3] = (uint8_t)(reg_value >> 24);
	buf[2] = (uint8_t)(reg_value >> 16);
	buf[1] = (uint8_t)(reg_value >> 8);
	buf[0] = (uint8_t)(reg_value);
	pl360_boot_pkt(self, ATPL360_BOOT_CMD_WRITE_WORD, ATPL360_MISCR, sizeof(uint32_t), buf);
	// Give control of the MISO signal to M7-SPI
	pl360_boot_pkt(self, ATPL360_BOOT_CMD_DIS_SPI_CTRL, 0, 0, NULL);
	Py_BEGIN_ALLOW_THREADS
	usleep(10000);
	Py_END_ALLOW_THREADS
}

static void pl360_check_status(PyPlcObject *self, uint32_t st) {
	uint16_t id;
	id = st & 0xFEFF;

	/* Check who is in the other side (bootloader / atpl360) */
	if (PLC_SPI_HEADER_BOOT == id) {
		pl360_booload(self);
	} else if (PLC_SPI_HEADER_CORTEX == id) {
		self->events = st >> 16;
	} else {
		/* Unexpected ID value -> Reset HW ATPL360 */
		pl360_booload(self);
	}
}

static void pktwr(plc_pkt_t* pkt) {
    uint16_t ptr = 0;
    // change word seq to bytes
    Py_BEGIN_ALLOW_THREADS
    while( ptr < pkt->len) {
        plc_tx_buffer[ptr+4] = pkt->buf[ptr+1];
        plc_tx_buffer[ptr+5] = pkt->buf[ptr];
        ptr += 2;
    }
    Py_END_ALLOW_THREADS
}

static void pktrd(plc_pkt_t* pkt) {
    uint16_t ptr = 0;
    // change word seq to bytes
    Py_BEGIN_ALLOW_THREADS
    while( ptr < pkt->len) {
        pkt->buf[ptr+1] = plc_rx_buffer[ptr+4];
        pkt->buf[ptr] = plc_rx_buffer[ptr+5];
        ptr += 2;
    }
    Py_END_ALLOW_THREADS
}

void pl360_datapkt(PyPlcObject *self, plc_cmd_t cmd, plc_pkt_t* pkt)
{
    int err;

	uint16_t us_len_wr_rd = (((pkt->len + 1) >> 1) & PLC_LEN_MASK) | (cmd << PLC_WR_RD_POS);

	printf("PLC len %d cmd %x addr %x\n", pkt->len, cmd, pkt->addr);
	/* Check length */
	if (!pkt->len) {
		return;
	}

	/** Configure PLC Tx buffer **/
	/* Address */
	plc_tx_buffer[0] = (uint8_t)(pkt->addr >> 8);
	plc_tx_buffer[1] = (uint8_t)(pkt->addr);
	/* Length & read/write */
	plc_tx_buffer[2] = (uint8_t)(us_len_wr_rd >> 8);
	plc_tx_buffer[3] = (uint8_t)(us_len_wr_rd);

	if (cmd == PLC_CMD_WRITE) {
		pktwr(pkt);
	} else {
        Py_BEGIN_ALLOW_THREADS
		memset(&plc_tx_buffer[PDC_SPI_HEADER_SIZE], 0, pkt->len);
        Py_END_ALLOW_THREADS
	}

	xfer.len  = pkt->len + PDC_SPI_HEADER_SIZE;
	if (xfer.len % 2)	{
		xfer.len ++;
	}

    Py_BEGIN_ALLOW_THREADS
    xfer.rx_buf = (unsigned long)&plc_rx_buffer[0];
    xfer.tx_buf = (unsigned long)&plc_tx_buffer[0];
    output_gpio(self->cs, 0);
	err = ioctl(self->fd, SPI_IOC_MESSAGE(1), &xfer);
    output_gpio(self->cs, 1);
	if(err < 0) {
        printf("Err %d", err);
		PyErr_SetFromErrno(PyExc_IOError);
	}
    Py_END_ALLOW_THREADS

	if (cmd == PLC_CMD_READ) {
		pktrd(pkt);
	}

	uint32_t status = (plc_rx_buffer[2] << 24) + (plc_rx_buffer[3] << 16) +
		(plc_rx_buffer[0] << 8) +  plc_rx_buffer[1];
	pl360_check_status(self, status);
}
