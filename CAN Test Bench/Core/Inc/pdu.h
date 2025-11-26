// Code to make human readable CAN messages for the device

// This file will define message data as human readable stuff


#include "main.h"
#include "uvfr_utils.h"

typedef struct abstract_conifer_channel abstract_conifer_channel;

// Can ID: 0x710

#ifndef __PDU_H__
#define __PDU_H__

typedef struct uv19_pdu_settings{
	uint32_t PDU_rx_addr;
	uint32_t PDU_tx_addr;
	uint32_t expected_period;
}uv19_pdu_settings;

typedef enum u19_PDU_ch{
	U19_PDU_5A_1 = 0x00,
	U19_PDU_5A_2 = 0x01,
	U19_PDU_5A_3 = 0x02,
	U19_PDU_5A_4 = 0x03,
	U19_PDU_5A_5 = 0x07,
	U19_PDU_5A_6 = 0x06,
	U19_PDU_5A_7 = 0x05,
	U19_PDU_5A_8 = 0x04,
	U19_PDU_5A_9 = 0x0B,
	U19_PDU_5A_10 = 0x0A,
	U19_PDU_5A_11 = 0x09,
	U19_PDU_5A_12 = 0x08,
	U19_PDU_5A_13 = 0x0C,
	U19_PDU_5A_14 = 0x0D,
	U19_PDU_5A_15 = 0x0E,
	U19_PDU_5A_16 = 0x0F,
	U19_PDU_20A_1 = 0x23,
	U19_PDU_20A_2 = 0x24,
	U19_PDU_20A_3 = 0x21,
	U19_PDU_20A_4 = 0x22,
	U19_PDU_20A_5 = 0x27,
	U19_PDU_20A_6 = 0x28,
	U19_PDU_20A_7 = 0x25,
	U19_PDU_20A_8 = 0x26
}u19_PDU_ch;


//Update a PDU channel to match it's conifer abstraction
uv_status u19updatePduChannel(abstract_conifer_channel* ch_ptr, uint32_t* ecode);

uv_status initPDU(uint32_t* ecode);

#endif
