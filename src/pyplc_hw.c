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
uint8_t plc_rx_buffer[PDC_PPLC_BUFFER_SIZE] __attribute__((__aligned__(2)));
/* PDC Transmission buffer */
uint8_t plc_tx_buffer[PDC_PPLC_BUFFER_SIZE] __attribute__((__aligned__(2)));

int pl360_transfer(PyPlcObject *self, uint8_t* buf, int len) {
    printf("Len %d\n", len);
    return len;
}

void pl360_reset(PyPlcObject *self) {
	output_gpio(self->ldo, 0);
    usleep(100);
    output_gpio(self->rst, 0);
    output_gpio(self->ldo, 1);

	/*tx_buf.len = 1;
	err = spi_transceive(data->spi_dev, &data->spi_cfg, &tx, &rx);
	if(err<0) {
		LOG_ERR("SPI error %d\n", err);
	}*/

	usleep(10);
    output_gpio(self->rst, 1);
    usleep(10);
}

static void pl360_boot_pkt(PyPlcObject *self, uint16_t cmd, uint32_t addr, uint16_t data_len, uint8_t *data_buf) {
	memcpy(plc_tx_buffer, &addr, sizeof(uint32_t));
	memcpy(&(plc_tx_buffer[4]), &cmd, sizeof(uint16_t));
	memcpy(&(plc_tx_buffer[6]), data_buf, data_len);
	tx_buf.len = data_len + 6;
	int err = spi_write(data->spi_dev, &data->spi_cfg, &tx);
	if(err<0) {
		LOG_ERR("SPI error %d\n", err);
	}
}