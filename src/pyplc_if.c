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

#pragma pack(push,1)

/* ! \name G3 TX Result values */
enum tx_result_values {
	TX_RESULT_PROCESS = 0,                  /* Transmission result: already in process */
	TX_RESULT_SUCCESS = 1,                  /* Transmission result: end successfully */
	TX_RESULT_INV_LENGTH = 2,               /* Transmission result: invalid length error */
	TX_RESULT_BUSY_CH = 3,                  /* Transmission result: busy channel error */
	TX_RESULT_BUSY_TX = 4,                  /* Transmission result: busy in transmission error */
	TX_RESULT_BUSY_RX = 5,                  /* Transmission result: busy in reception error */
	TX_RESULT_INV_SCHEME = 6,               /* Transmission result: invalid modulation scheme error */
	TX_RESULT_TIMEOUT = 7,                  /* Transmission result: timeout error */
	TX_RESULT_INV_TONEMAP = 8,              /* Transmission result: invalid tone map error */
	TX_RESULT_INV_MODE = 9,                 /* Transmission result: invalid G3 Mode error */
	TX_RESULT_NO_TX = 255,                  /* Transmission result: No transmission ongoing */
};

typedef struct __attribute__((__packed__)) {
	uint32_t tx_time;
	uint16_t data_len;
	uint8_t tone_groups[NUM_SUBBANDS_MAX];
	uint8_t tone_map[TONE_MAP_SIZE_MAX];
	uint8_t tx_mode;                            /* Transmission Mode (absolute, relative, forced, continuous, cancel). Constants above */
	uint8_t tx_power;                           /* Power to transmit */
	uint8_t mod_type;                           /* Modulation type */
	uint8_t mod_scheme;                         /* Modulation scheme */
	uint8_t pdc;                                /* Phase Detector Counter */
	uint8_t rs_blocks;                          /* Flag to indicate whether 2 RS blocks have to be used (only used for FCC) */
	uint8_t uc_delimiter_type;                  /* DT field to be used in header */
} pl360_tx_config_t;

/* ! \name G3 Structure defining Rx message */
typedef struct __attribute__((__packed__)) {
	uint32_t ul_rx_time;                           /* /< Instant when frame was received */
	uint32_t ul_frame_duration;                    /* /< Frame duration referred to 1ms PHY counter (FCH + Payload) */
	uint16_t us_rssi;                              /* /< Reception RSSI */
	uint16_t us_data_len;                          /* /< Length of the data buffer */
	uint8_t uc_zct_diff;                           /* /< ZCT info */
	uint8_t uc_rs_corrected_errors;                /* /< Errors corrected by RS */
	enum mod_types uc_mod_type;                    /* /< Modulation type of the last received message */
	enum mod_schemes uc_mod_scheme;                /* /< Modulation scheme of the last received message */
	uint32_t ul_agc_factor;                        /* /< Test data information */
	uint16_t us_agc_fine;                          /* /< Test data information */
	int16_t ss_agc_offset_meas;                    /* /< Test data information */
	uint8_t uc_agc_active;                         /* /< Test data information */
	uint8_t uc_agc_pga_value;                      /* /< Test data information */
	int16_t ss_snr_fch;                            /* /< Test data information */
	int16_t ss_snr_pay;                            /* /< Test data information */
	uint16_t us_payload_corrupted_carriers;        /* /< BER: Number of corrupted carriers */
	uint16_t us_payload_noised_symbols;            /* /< BER: Number of noised symbols */
	uint8_t uc_payload_snr_worst_carrier;          /* /< BER: SNR of the worst carrier */
	uint8_t uc_payload_snr_worst_symbol;           /* /< BER: SNR of the worst symbol */
	uint8_t uc_payload_snr_impulsive;              /* /< BER: SNR on impulsive noise */
	uint8_t uc_payload_snr_band;                   /* /< BER: Narrowband SNR */
	uint8_t uc_payload_snr_background;             /* /< BER: Background SNR */
	uint8_t uc_lqi;                                /* /< BER: Link Quality */
	enum delimiter_types uc_delimiter_type;        /* /< DT field coming in header */
	uint8_t uc_rsrv0;                              /* /< MAC CRC. 1: OK; 0: NOK (CRC capability can be enabled/disabled). 16 bits for allignement  */
	uint8_t puc_tone_map[TONE_MAP_SIZE_MAX];       /* /< Reception Tone Map */
	uint8_t puc_carrier_snr[PROTOCOL_CARRIERS_MAX]; /* /< SNR per carrier */
	uint8_t uc_rsrv1;                              /* /< Reserved byte */
	uint8_t *puc_data_buf;                         /* /< Pointer to data buffer */
} rx_msg_t;

/* ! \name G3 Structure defining result of a transmission */
typedef struct __attribute__((__packed__)) {
	uint32_t ul_rms_calc;                          /* RMS_CALC it allows to estimate tx power injected */
	uint32_t ul_tx_time;                           /* Instant when frame transmission ended referred to 1ms PHY counter */
	enum tx_result_values uc_tx_result;            /* Tx Result (see "TX Result values" above) */
} tx_cfm_t;

typedef struct __attribute__((__packed__)) {
	uint32_t time;
	uint32_t evt;
} status_t;

#pragma pack(pop)

#define ATPL360_CMF_PKT_SIZE                      sizeof(tx_cfm_t)

/* Init packet data size */
#define INIT_PKT_DATA_SIZE (8)

static pl360_tx_config_t txconf;

void pl360_tx(PyPlcObject *self, uint8_t* buf, int len) {
    assert(len<=PYPLC_MAX_BLOCK_SIZE);
    plc_pkt_t* pkt = (plc_pkt_t*) malloc(sizeof(plc_pkt_t)+len);
	assert(pkt);
    memcpy(pkt->buf, buf, len);
    pkt->len = len;
    pkt->addr = ATPL360_TX_DATA_ID;

    pl360_datapkt(self, PLC_CMD_WRITE, pkt);
    free(pkt);
}

static uint32_t access_type(uint16_t param_id)
{
	uint32_t address = 0;

	if (param_id & ATPL360_REG_ADC_MASK) {
		address = (uint32_t)ATPL360_REG_ADC_BASE;
	} else if (param_id & ATPL360_REG_DAC_MASK) {
		address = (uint32_t)ATPL360_REG_DAC_BASE;
	} else if (param_id & ATPL360_FUSES_MASK) {
		address = (uint32_t)ATPL360_FUSES_BASE;
	} else if ((param_id & ATPL360_REG_MASK) && (param_id < ATPL360_REG_END_ID)) {
		address = (uint32_t)ATPL360_REG_BASE;
	}
	return address;
}

static void pl360_set_config(plc_pkt_t* pkt, uint16_t param_id, uint16_t value) {
	uint32_t reg_addr = (uint32_t)(param_id & PL360_REG_OFFSET_MASK) + access_type(param_id);
	uint16_t reg_len = PL360_REG_CMD_WR | (1 & PL360_REG_LEN_MASK);
	pkt->buf[0] = (uint8_t)(reg_addr >> 24);
	pkt->buf[1] = (uint8_t)(reg_addr >> 16);
	pkt->buf[2] = (uint8_t)(reg_addr >> 8);
	pkt->buf[3] = (uint8_t)(reg_addr);
	pkt->buf[4] = (uint8_t)(reg_len >> 8);
	pkt->buf[5] = (uint8_t)(reg_len);
	pkt->buf[6] = value&0xFF;
	pkt->buf[7] = value>>8;

	pkt->addr = ATPL360_REG_INFO_ID;
	pkt->len = 8;
}

void pl360_init(PyPlcObject *self) {
	plc_pkt_t* pkt = malloc(INIT_PKT_DATA_SIZE+sizeof(plc_pkt_t));
	assert(pkt);
	/* Read Time Ref to get SPI status and boot if necessary */
	pkt->addr = ATPL360_STATUS_INFO_ID;
	pkt->len = INIT_PKT_DATA_SIZE;
	pl360_datapkt(self, PLC_CMD_READ, pkt);

	// Restart IRQ after boot
	pl360_datapkt(self, PLC_CMD_READ, pkt);

	/* Disable AUTO mode and set VLO behavior by default in order to maximize signal level in anycase */
	pl360_set_config(pkt, ATPL360_REG_CFG_AUTODETECT_IMPEDANCE, 0);
	pl360_datapkt(self, PLC_CMD_WRITE, pkt);
    Py_BEGIN_ALLOW_THREADS
	usleep(1000);
    Py_END_ALLOW_THREADS
	pl360_set_config(pkt, ATPL360_REG_CFG_IMPEDANCE, 2);
	pl360_datapkt(self, PLC_CMD_WRITE, pkt);
    free(pkt);
    Py_BEGIN_ALLOW_THREADS
	usleep(1000);

	/* Prepare default TX config */
	const uint8_t tonemap[] = PL360_IF_DEFAULT_TONE_MAP;
	txconf.tx_time = 0;
	memcpy(txconf.tone_map, tonemap, sizeof(tonemap));

	txconf.tx_mode =  TX_MODE_RELATIVE; // uc_tx_mode
	txconf.tx_time = 0x3E8;
	txconf.tx_power =  PL360_IF_TX_POWER; // uc_tx_power

	txconf.mod_type =  MOD_TYPE_BPSK; // uc_mod_type
	txconf.mod_scheme =  MOD_SCHEME_DIFFERENTIAL; // uc_mod_scheme
	txconf.uc_delimiter_type =  DT_SOF_NO_RESP; // uc_delimiter_type

    // Configure interrupt

    Py_END_ALLOW_THREADS
}