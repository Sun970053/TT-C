/*
 * si4463_huang.h
 *
 *  Created on: June 24, 2024
 *      Author: Ting-Shan, Huang
 */

#ifndef INC_RADIO_CONFIG_SELECTION_H_
#define INC_RADIO_CONFIG_SELECTION_H_

/* Configuration array for every combination of data rate and modulation */

/* GMSK 9600 bd */
/* TX */
#define GMSK_9600_MODEM_MOD_TYPE_12 0x11, 0x20, 0x0C, 0x00, 0x03, 0x00, 0x07, 0x05, 0xDC, 0x00, 0x05, 0xC9, 0xC3, 0x80, 0x00, 0x00
#define GMSK_9600_MODEM_FREQ_DEV_0_1 0x11, 0x20, 0x01, 0x0C, 0xA8
/* RX */
#define GMSK_9600_MODEM_DECIMATION_CFG_1_2 0x11, 0x20, 0x02, 0x1E, 0x30, 0x10
#define GMSK_9600_MODEM_BCR_OSR_1_9 0x11, 0x20, 0x09, 0x22, 0x00, 0x82, 0x03, 0xEE, 0xA2, 0x07, 0xE0, 0x02, 0x00
#define GMSK_9600_MODEM_AFC_GAIN_1_4 0x11, 0x20, 0x04, 0x2E, 0xC0, 0x54, 0x03, 0x8F
#define GMSK_9600_MODEM_AGC_RFPD_DECAY_8 0x11, 0x20, 0x06, 0x39, 0x1C, 0x1C, 0x00, 0x1A, 0x20, 0x00, 0x00, 0x29
#define GMSK_9600_MODEM_RAW_EYE_1_2 0x11, 0x20, 0x02, 0x46, 0x00, 0x4A
#define GMSK_9600_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12 0x11, 0x21, 0x0C, 0x00, 0xFF, 0xBA, 0x0F, 0x51, 0xCF, 0xA9, 0xC9, 0xFC, 0x1B, 0x1E, 0x0F, 0x01
#define GMSK_9600_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12 0x11, 0x21, 0x0C, 0x0C, 0xFC, 0xFD, 0x15, 0xFF, 0x00, 0x0F, 0xF9, 0xF0, 0xD7, 0xB1, 0x86, 0x59
#define GMSK_9600_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12 0x11, 0x21, 0x0C, 0x18, 0x33, 0x15, 0x01, 0xF7, 0xF4, 0xF6, 0xF9, 0xFC, 0x00, 0x00, 0xFC, 0x0F

/* GMSK 4800 bd */
/* TX */
#define GMSK_4800_MODEM_MOD_TYPE_12 0x11, 0x20, 0x0C, 0x00, 0x03, 0x00, 0x07, 0x02, 0xEE, 0x00, 0x05, 0xC9, 0xC3, 0x80, 0x00, 0x00
#define GMSK_4800_MODEM_FREQ_DEV_0_1 0x11, 0x20, 0x01, 0x0C, 0x54
/* RX */
#define GMSK_4800_MODEM_DECIMATION_CFG_1_2 0x11, 0x20, 0x02, 0x1E, 0x30, 0x10
#define GMSK_4800_MODEM_BCR_OSR_1_9 0x11, 0x20, 0x09, 0x22, 0x01, 0x04, 0x01, 0xF7, 0x51, 0x03, 0xF0, 0x02, 0x00
#define GMSK_4800_MODEM_AFC_GAIN_1_4 0x11, 0x20, 0x04, 0x2E, 0xC0, 0x2A, 0x06, 0xF5
#define GMSK_4800_MODEM_AGC_RFPD_DECAY_8 0x11, 0x20, 0x06, 0x39, 0x39, 0x39, 0x00, 0x1A, 0x20, 0x00, 0x00, 0x2A
#define GMSK_4800_MODEM_RAW_EYE_1_2 0x11, 0x20, 0x02, 0x46, 0x00, 0x25
#define GMSK_4800_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12 0x11, 0x21, 0x0C, 0x00, 0xFF, 0xBA, 0x0F, 0x51, 0xCF, 0xA9, 0xC9, 0xFC, 0x1B, 0x1E, 0x0F, 0x01
#define GMSK_4800_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12 0x11, 0x21, 0x0C, 0x0C, 0xFC, 0xFD, 0x15, 0xFF, 0x00, 0x0F, 0xA2, 0xA0, 0x97, 0x8A, 0x79, 0x66
#define GMSK_4800_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12 0x11, 0x21, 0x0C, 0x18, 0x52, 0x3F, 0x2E, 0x1F, 0x14, 0x0B, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00

/* GMSK 2400 bd */
/* TX */
#define GMSK_2400_MODEM_MOD_TYPE_12 0x11, 0x20, 0x0C, 0x00, 0x03, 0x00, 0x07, 0x01, 0x77, 0x00, 0x05, 0xC9, 0xC3, 0x80, 0x00, 0x00
#define GMSK_2400_MODEM_FREQ_DEV_0_1 0x11, 0x20, 0x01, 0x0C, 0x2A
/* RX */
#define GMSK_2400_MODEM_DECIMATION_CFG_1_2 0x11, 0x20, 0x02, 0x1E, 0x30, 0x10
#define GMSK_2400_MODEM_BCR_OSR_1_9 0x11, 0x20, 0x09, 0x22, 0x02, 0x09, 0x00, 0xFB, 0xA9, 0x01, 0xF7, 0x02, 0x00
#define GMSK_2400_MODEM_AFC_GAIN_1_4 0x11, 0x20, 0x04, 0x2E, 0xC0, 0x15, 0x0D, 0xE9
#define GMSK_2400_MODEM_AGC_RFPD_DECAY_8 0x11, 0x20, 0x06, 0x39, 0x72, 0x72, 0x00, 0x1A, 0x20, 0x00, 0x00, 0x2B
#define GMSK_2400_MODEM_RAW_EYE_1_2 0x11, 0x20, 0x02, 0x46, 0x00, 0x12
#define GMSK_2400_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12 0x11, 0x21, 0x0C, 0x00, 0xFF, 0xBA, 0x0F, 0x51, 0xCF, 0xA9, 0xC9, 0xFC, 0x1B, 0x1E, 0x0F, 0x01
#define GMSK_2400_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12 0x11, 0x21, 0x0C, 0x0C, 0xFC, 0xFD, 0x15, 0xFF, 0x00, 0x0F, 0xA2, 0xA0, 0x97, 0x8A, 0x79, 0x66
#define GMSK_2400_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12 0x11, 0x21, 0x0C, 0x18, 0x52, 0x3F, 0x2E, 0x1F, 0x14, 0x0B, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00

/* GMSK 1200 bd */
/* TX */
#define GMSK_1200_MODEM_MOD_TYPE_12 0x11, 0x20, 0x0C, 0x00, 0x03, 0x00, 0x07, 0x00, 0xBB, 0x80, 0x05, 0xC9, 0xC3, 0x80, 0x00, 0x00
#define GMSK_1200_MODEM_FREQ_DEV_0_1 0x11, 0x20, 0x01, 0x0C, 0x15
/* RX MODEM_MDM_CTRL??*/
#define GMSK_1200_MODEM_DECIMATION_CFG_1_2 0x11, 0x20, 0x02, 0x1E, 0x30, 0x10
#define GMSK_1200_MODEM_BCR_OSR_1_9 0x11, 0x20, 0x09, 0x22, 0x04, 0x12, 0x00, 0x7D, 0xD4, 0x00, 0x3F, 0x02, 0xC2
#define GMSK_1200_MODEM_AFC_GAIN_1_4 0x11, 0x20, 0x04, 0x2E, 0xC0, 0x03, 0x4C, 0x2C
#define GMSK_1200_MODEM_AGC_RFPD_DECAY_8 0x11, 0x20, 0x06, 0x39, 0xE4, 0xE4, 0x00, 0x1A, 0x20, 0x00, 0x00, 0x2B
#define GMSK_1200_MODEM_RAW_EYE_1_2 0x11, 0x20, 0x02, 0x46, 0x00, 0x09
#define GMSK_1200_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12 0x11, 0x21, 0x0C, 0x00, 0xFF, 0xBA, 0x0F, 0x51, 0xCF, 0xA9, 0xC9, 0xFC, 0x1B, 0x1E, 0x0F, 0x01
#define GMSK_1200_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12 0x11, 0x21, 0x0C, 0x0C, 0xFC, 0xFD, 0x15, 0xFF, 0x00, 0x0F, 0xA2, 0xA0, 0x97, 0x8A, 0x79, 0x66
#define GMSK_1200_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12 0x11, 0x21, 0x0C, 0x18, 0x52, 0x3F, 0x2E, 0x1F, 0x14, 0x0B, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00

/* FSK 9600 bd */
/* TX */
#define FSK_9600_MODEM_MOD_TYPE_12 0x11, 0x20, 0x0C, 0x00, 0x02, 0x00, 0x07, 0x01, 0x77, 0x00, 0x01, 0xC9, 0xC3, 0x80, 0x00, 0x01
#define FSK_9600_MODEM_FREQ_DEV_0_1 0x11, 0x20, 0x01, 0x0C, 0x50
/* RX */
#define FSK_9600_MODEM_DECIMATION_CFG_1_2 0x11, 0x20, 0x02, 0x1E, 0x30, 0x20
#define FSK_9600_MODEM_BCR_OSR_1_9 0x11, 0x20, 0x09, 0x22, 0x00, 0xC3, 0x02, 0x9F, 0x17, 0x02, 0xA0, 0x02, 0x00
#define FSK_9600_MODEM_AFC_GAIN_1_4 0x11, 0x20, 0x04, 0x2E, 0xC0, 0x54, 0x03, 0xC4
#define FSK_9600_MODEM_AGC_RFPD_DECAY_8 0x11, 0x20, 0x06, 0x39, 0x2B, 0x2B, 0x00, 0x02, 0x40, 0x00, 0x00, 0x29
#define FSK_9600_MODEM_RAW_EYE_1_2 0x11, 0x20, 0x02, 0x46, 0x00, 0xA4
#define FSK_9600_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12 0x11, 0x21, 0x0C, 0x00, 0xA2, 0x81, 0x26, 0xAF, 0x3F, 0xEE, 0xC8, 0xC7, 0xDB, 0xF2, 0x02, 0x08
#define FSK_9600_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12 0x11, 0x21, 0x0C, 0x0C, 0x07, 0x03, 0x15, 0xFC, 0x0F, 0x00, 0xE7, 0xDF, 0xCA, 0xAA, 0x84, 0x5D
#define FSK_9600_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12 0x11, 0x21, 0x0C, 0x18, 0x3A, 0x1E, 0x0A, 0xFE, 0xF9, 0xF9, 0xFA, 0xFD, 0x00, 0x00, 0xFC, 0x0F

/* FSK 4800 bd */
/* TX */
#define FSK_4800_MODEM_MOD_TYPE_12 0x11, 0x20, 0x0C, 0x00, 0x02, 0x00, 0x07, 0x00, 0xBB, 0x80, 0x01, 0xC9, 0xC3, 0x80, 0x00, 0x00
#define FSK_4800_MODEM_FREQ_DEV_0_1 0x11, 0x20, 0x01, 0x0C, 0xA8
/* RX */
#define FSK_4800_MODEM_DECIMATION_CFG_1_2 0x11, 0x20, 0x02, 0x1E, 0x30, 0x10
#define FSK_4800_MODEM_BCR_OSR_1_9 0x11, 0x20, 0x09, 0x22, 0x01, 0x04, 0x01, 0xF7, 0x51, 0x01, 0xF8, 0x02, 0x00
#define FSK_4800_MODEM_AFC_GAIN_1_4 0x11, 0x20, 0x04, 0x2E, 0xC0, 0x2A, 0x06, 0xF5
#define FSK_4800_MODEM_AGC_RFPD_DECAY_8 0x11, 0x20, 0x06, 0x39, 0x39, 0x39, 0x00, 0x02, 0x40, 0x00, 0x00, 0x2A
#define FSK_4800_MODEM_RAW_EYE_1_2 0x11, 0x20, 0x02, 0x46, 0x00, 0x7B
#define FSK_4800_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12 0x11, 0x21, 0x0C, 0x00, 0xFF, 0xBA, 0x0F, 0x51, 0xCF, 0xA9, 0xC9, 0xFC, 0x1B, 0x1E, 0x0F, 0x01
#define FSK_4800_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12 0x11, 0x21, 0x0C, 0x0C, 0xFC, 0xFD, 0x15, 0xFF, 0x00, 0x0F, 0xC6, 0xC1, 0xB2, 0x9C, 0x80, 0x63
#define FSK_4800_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12 0x11, 0x21, 0x0C, 0x18, 0x47, 0x2F, 0x1B, 0x0E, 0x05, 0x00, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x0F

/* FSK 2400 bd */
/* TX */
#define FSK_2400_MODEM_MOD_TYPE_12 0x11, 0x20, 0x0C, 0x00, 0x02, 0x00, 0x07, 0x00, 0x5D, 0xC0, 0x01, 0xC9, 0xC3, 0x80, 0x00, 0x00
#define FSK_2400_MODEM_FREQ_DEV_0_1 0x11, 0x20, 0x01, 0x0C, 0x54
/* RX */
#define FSK_2400_MODEM_DECIMATION_CFG_1_2 0x11, 0x20, 0x02, 0x1E, 0x30, 0x10
#define FSK_2400_MODEM_BCR_OSR_1_9 0x11, 0x20, 0x09, 0x22, 0x02, 0x09, 0x00, 0xFB, 0xA9, 0x00, 0xFC, 0x02, 0x00
#define FSK_2400_MODEM_AFC_GAIN_1_4 0x11, 0x20, 0x04, 0x2E, 0xC0, 0x15, 0x0D, 0xE9
#define FSK_2400_MODEM_AGC_RFPD_DECAY_8 0x11, 0x20, 0x06, 0x39, 0x72, 0x72, 0x00, 0x02, 0x40, 0x00, 0x00, 0x2B
#define FSK_2400_MODEM_RAW_EYE_1_2 0x11, 0x20, 0x02, 0x46, 0x00, 0x3D
#define FSK_2400_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12 0x11, 0x21, 0x0C, 0x00, 0xFF, 0xBA, 0x0F, 0x51, 0xCF, 0xA9, 0xC9, 0xFC, 0x1B, 0x1E, 0x0F, 0x01
#define FSK_2400_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12 0x11, 0x21, 0x0C, 0x0C, 0xFC, 0xFD, 0x15, 0xFF, 0x00, 0x0F, 0xA2, 0xA0, 0x97, 0x8A, 0x79, 0x66
#define FSK_2400_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12 0x11, 0x21, 0x0C, 0x18, 0x52, 0x3F, 0x2E, 0x1F, 0x14, 0x0B, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00

/* FSK 1200 bd */
/* TX */
#define FSK_1200_MODEM_MOD_TYPE_12 0x11, 0x20, 0x0C, 0x00, 0x02, 0x00, 0x07, 0x00, 0x2E, 0xE0, 0x01, 0xC9, 0xC3, 0x80, 0x00, 0x00
#define FSK_1200_MODEM_FREQ_DEV_0_1 0x11, 0x20, 0x01, 0x0C, 0x2A
/* RX */
#define FSK_1200_MODEM_DECIMATION_CFG_1_2 0x11, 0x20, 0x02, 0x1E, 0x30, 0x10
#define FSK_1200_MODEM_BCR_OSR_1_9 0x11, 0x20, 0x09, 0x22, 0x04, 0x12, 0x00, 0x7D, 0xD4, 0x00, 0x3F, 0x02, 0xC2
#define FSK_1200_MODEM_AFC_GAIN_1_4 0x11, 0x20, 0x04, 0x2E, 0xC0, 0x03, 0x4C, 0x2C
#define FSK_1200_MODEM_AGC_RFPD_DECAY_8 0x11, 0x20, 0x06, 0x39, 0xE4, 0xE4, 0x00, 0x02, 0x40, 0x00, 0x00, 0x2B
#define FSK_1200_MODEM_RAW_EYE_1_2 0x11, 0x20, 0x02, 0x46, 0x00, 0x1F
#define FSK_1200_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12 0x11, 0x21, 0x0C, 0x00, 0xFF, 0xBA, 0x0F, 0x51, 0xCF, 0xA9, 0xC9, 0xFC, 0x1B, 0x1E, 0x0F, 0x01
#define FSK_1200_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12 0x11, 0x21, 0x0C, 0x0C, 0xFC, 0xFD, 0x15, 0xFF, 0x00, 0x0F, 0xA2, 0xA0, 0x97, 0x8A, 0x79, 0x66
#define FSK_1200_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12 0x11, 0x21, 0x0C, 0x18, 0x52, 0x3F, 0x2E, 0x1F, 0x14, 0x0B, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00

/* OOK */
#define OOK_GPIO_PIN_CFG 0x13, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#define OOK_MODEM_MOD_TYPE_12 0x11, 0x20, 0x0C, 0x00, 0x09, 0x00, 0x07, 0x00, 0x4E, 0x20, 0x01, 0x8C, 0xBA, 0x80, 0x00, 0x00
#define OOK_MODEM_FREQ_DEV_0_1 0x11, 0x20, 0x01, 0x0C, 0x00

/* Configuration array for transmission and reception. */
#define RADIO_CONFIGURATION_GMSK_9600_TX \
{ \
    0x10, GMSK_9600_MODEM_MOD_TYPE_12, \
    0x05, GMSK_9600_MODEM_FREQ_DEV_0_1, \
    0x00 \
} \

#define RADIO_CONFIGURATION_GMSK_9600_RX \
{ \
    0x06, GMSK_9600_MODEM_DECIMATION_CFG_1_2, \
    0x0D, GMSK_9600_MODEM_BCR_OSR_1_9, \
    0x08, GMSK_9600_MODEM_AFC_GAIN_1_4, \
    0x0C, GMSK_9600_MODEM_AGC_RFPD_DECAY_8, \
    0x06, GMSK_9600_MODEM_RAW_EYE_1_2, \
    0x10, GMSK_9600_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12, \
    0x10, GMSK_9600_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12, \
    0x10, GMSK_9600_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12, \
    0x00 \
} \

#define RADIO_CONFIGURATION_GMSK_4800_TX \
{ \
    0x10, GMSK_4800_MODEM_MOD_TYPE_12, \
    0x05, GMSK_4800_MODEM_FREQ_DEV_0_1, \
    0x00 \
} \

#define RADIO_CONFIGURATION_GMSK_4800_RX \
{ \
    0x06, GMSK_4800_MODEM_DECIMATION_CFG_1_2, \
    0x0D, GMSK_4800_MODEM_BCR_OSR_1_9, \
    0x08, GMSK_4800_MODEM_AFC_GAIN_1_4, \
    0x0C, GMSK_4800_MODEM_AGC_RFPD_DECAY_8, \
    0x06, GMSK_4800_MODEM_RAW_EYE_1_2, \
    0x10, GMSK_4800_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12, \
    0x10, GMSK_4800_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12, \
    0x10, GMSK_4800_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12, \
    0x00 \
} \

#define RADIO_CONFIGURATION_GMSK_2400_TX \
{ \
    0x10, GMSK_2400_MODEM_MOD_TYPE_12, \
    0x05, GMSK_2400_MODEM_FREQ_DEV_0_1, \
    0x00 \
}

#define RADIO_CONFIGURATION_GMSK_2400_RX \
{ \
    0x06, GMSK_2400_MODEM_DECIMATION_CFG_1_2, \
    0x0D, GMSK_2400_MODEM_BCR_OSR_1_9, \
    0x08, GMSK_2400_MODEM_AFC_GAIN_1_4, \
    0x0C, GMSK_2400_MODEM_AGC_RFPD_DECAY_8, \
    0x06, GMSK_2400_MODEM_RAW_EYE_1_2, \
    0x10, GMSK_2400_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12, \
    0x10, GMSK_2400_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12, \
    0x10, GMSK_2400_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12, \
    0x00 \
} \

#define RADIO_CONFIGURATION_GMSK_1200_TX \
{ \
    0x10, GMSK_1200_MODEM_MOD_TYPE_12, \
    0x05, GMSK_1200_MODEM_FREQ_DEV_0_1, \
    0x00 \
}

#define RADIO_CONFIGURATION_GMSK_1200_RX \
{ \
    0x06, GMSK_1200_MODEM_DECIMATION_CFG_1_2, \
    0x0D, GMSK_1200_MODEM_BCR_OSR_1_9, \
    0x08, GMSK_1200_MODEM_AFC_GAIN_1_4, \
    0x0C, GMSK_1200_MODEM_AGC_RFPD_DECAY_8, \
    0x06, GMSK_1200_MODEM_RAW_EYE_1_2, \
    0x10, GMSK_1200_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12, \
    0x10, GMSK_1200_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12, \
    0x10, GMSK_1200_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12, \
    0x00 \
} \

#define RADIO_CONFIGURATION_FSK_9600_TX \
{ \
    0x10, FSK_9600_MODEM_MOD_TYPE_12, \
    0x05, FSK_9600_MODEM_FREQ_DEV_0_1, \
    0x00 \
} \

#define RADIO_CONFIGURATION_FSK_9600_RX \
{ \
    0x06, FSK_9600_MODEM_DECIMATION_CFG_1_2, \
    0x0D, FSK_9600_MODEM_BCR_OSR_1_9, \
    0x08, FSK_9600_MODEM_AFC_GAIN_1_4, \
    0x0C, FSK_9600_MODEM_AGC_RFPD_DECAY_8, \
    0x06, FSK_9600_MODEM_RAW_EYE_1_2, \
    0x10, FSK_9600_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12, \
    0x10, FSK_9600_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12, \
    0x10, FSK_9600_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12, \
    0x00 \
} \

#define RADIO_CONFIGURATION_FSK_4800_TX \
{ \
    0x10, FSK_4800_MODEM_MOD_TYPE_12, \
    0x05, FSK_4800_MODEM_FREQ_DEV_0_1, \
    0x00 \
} \

#define RADIO_CONFIGURATION_FSK_4800_RX \
{ \
    0x06, FSK_4800_MODEM_DECIMATION_CFG_1_2, \
    0x0D, FSK_4800_MODEM_BCR_OSR_1_9, \
    0x08, FSK_4800_MODEM_AFC_GAIN_1_4, \
    0x0C, FSK_4800_MODEM_AGC_RFPD_DECAY_8, \
    0x06, FSK_4800_MODEM_RAW_EYE_1_2, \
    0x10, FSK_4800_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12, \
    0x10, FSK_4800_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12, \
    0x10, FSK_4800_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12, \
    0x00 \
} \

#define RADIO_CONFIGURATION_FSK_2400_TX \
{ \
    0x10, FSK_2400_MODEM_MOD_TYPE_12, \
    0x05, FSK_2400_MODEM_FREQ_DEV_0_1, \
    0x00 \
} \

#define RADIO_CONFIGURATION_FSK_2400_RX \
{ \
    0x06, FSK_2400_MODEM_DECIMATION_CFG_1_2, \
    0x0D, FSK_2400_MODEM_BCR_OSR_1_9, \
    0x08, FSK_2400_MODEM_AFC_GAIN_1_4, \
    0x0C, FSK_2400_MODEM_AGC_RFPD_DECAY_8, \
    0x06, FSK_2400_MODEM_RAW_EYE_1_2, \
    0x10, FSK_2400_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12, \
    0x10, FSK_2400_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12, \
    0x10, FSK_2400_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12, \
    0x00 \
} \

#define RADIO_CONFIGURATION_FSK_1200_TX \
{ \
    0x10, FSK_1200_MODEM_MOD_TYPE_12, \
    0x05, FSK_1200_MODEM_FREQ_DEV_0_1, \
    0x00 \
} \

#define RADIO_CONFIGURATION_FSK_1200_RX \
{ \
    0x06, FSK_1200_MODEM_DECIMATION_CFG_1_2, \
    0x0D, FSK_1200_MODEM_BCR_OSR_1_9, \
    0x08, FSK_1200_MODEM_AFC_GAIN_1_4, \
    0x0C, FSK_1200_MODEM_AGC_RFPD_DECAY_8, \
    0x06, FSK_1200_MODEM_RAW_EYE_1_2, \
    0x10, FSK_1200_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12, \
    0x10, FSK_1200_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12, \
    0x10, FSK_1200_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12, \
    0x00 \
} \

#define RADIO_CONFIGURATION_OOK_TX \
{ \
    0x08, OOK_GPIO_PIN_CFG, \
    0x10, OOK_MODEM_MOD_TYPE_12, \
    0x05, OOK_MODEM_FREQ_DEV_0_1, \
    0x00 \
} \

#endif /* RADIO_CONFIG_SELECTION_H_ */
