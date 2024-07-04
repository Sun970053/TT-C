/*
 * ax25_huang.c
 *
 *  Created on: June 16, 2024
 *      Author: Ting-Shan, Huang
 */

#include "ax25_huang.h"
#include <stdlib.h>
#include <string.h>

uint32_t reflect(uint32_t in, uint8_t bits);
uint32_t checksum(ax25frame_t* ax25frame, uint8_t* buff, size_t len);

uint32_t d_shift_register = 0;
uint32_t d_taps[32];
uint32_t d_tap_count;


ax25sendframe_t* createAX25SendFrame(const char* destCallsign, uint8_t destSSID, const char* srcCallsign, uint8_t srcSSID, uint8_t control, uint8_t protocolID, uint8_t* info, uint16_t infoLen, uint16_t preLen)
{
  ax25sendframe_t* ax25frame = (ax25sendframe_t*)malloc(sizeof(ax25sendframe_t));

  // destination callsign/SSID
  memcpy(ax25frame->destCallsign, destCallsign, strlen(destCallsign));
  ax25frame->destCallsign[strlen(destCallsign)] = '\0';
  ax25frame->destSSID = destSSID;

  // source callsign/SSID
  memcpy(ax25frame->srcCallsign, srcCallsign, strlen(srcCallsign));
  ax25frame->srcCallsign[strlen(srcCallsign)] = '\0';
  ax25frame->srcSSID = srcSSID;

  // control field
  ax25frame->control = control;

  // sequence numbers
  ax25frame->rcvSeqNumber = 0;
  ax25frame->sendSeqNumber = 0;

  // PID field
  ax25frame->protocolID = protocolID;

  // info field
  ax25frame->infoLen = infoLen;
  if(infoLen > 0) 
  {
      ax25frame->info = (uint8_t*)malloc(infoLen*sizeof(uint8_t));
      memcpy(ax25frame->info, info, infoLen);
  }

  // save preamble length
  ax25frame->preambleLen = preLen;

  return ax25frame;
}

ax25receiveframe_t* createAX25ReceiveFrame(uint8_t control, uint8_t protocolID,  uint16_t preLen)
{
  ax25receiveframe_t* ax25frame = (ax25receiveframe_t*)malloc(sizeof(ax25receiveframe_t));

  // control field
  ax25frame->control = control;

  // sequence numbers
  ax25frame->rcvSeqNumber = 0;
  ax25frame->sendSeqNumber = 0;

  // PID field
  ax25frame->protocolID = protocolID;

  // save preamble length
  ax25frame->preambleLen = preLen;

  return ax25frame;
}

void deleteAX25SendFrame(ax25frame_t* ax25frame)
{
  // deallocate info field
  if(ax25frame->ax25SendFrame->infoLen > 0)
  {
      free(ax25frame->ax25SendFrame->info);
      ax25frame->ax25SendFrame->info = NULL;
  }
  free(ax25frame->ax25SendFrame);
  ax25frame->ax25SendFrame = NULL;
}

void deleteAX25ReceiveFrame(ax25frame_t* ax25frame)
{
  // deallocate info field
  if(ax25frame->ax25RcvFrame->infoLen > 0)
  {
      free(ax25frame->ax25SendFrame->info);
      ax25frame->ax25SendFrame->info = NULL;
  }
  free(ax25frame->ax25SendFrame);
  ax25frame->ax25SendFrame = NULL;
}

void initCRC(ax25frame_t* ax25frame)
{
  ax25frame->crc.size = 16;
  ax25frame->crc.poly = RADIOLIB_CRC_CCITT_POLY;
  ax25frame->crc.init = RADIOLIB_CRC_CCITT_INIT;
  ax25frame->crc.out = RADIOLIB_CRC_CCITT_OUT;
  ax25frame->crc.refIn = false;
  ax25frame->crc.refOut = false;
}

uint8_t* AX25Frame_HDLC_Generator(ax25sendframe_t* ax25frame, uint16_t* stuffedFrameLen)
{
  // check destination callsign length (6 characters max)
  if(strlen(ax25frame->destCallsign) > RADIOLIB_AX25_MAX_CALLSIGN_LEN) {
      return(RADIOLIB_ERR_INVALID_CALLSIGN);
  }

  // calculate frame length without FCS (destination address, source address, repeater addresses, control, PID, info)
  size_t frameBuffLen = (2*(RADIOLIB_AX25_MAX_CALLSIGN_LEN + 1)) + 1 + 1 + ax25frame->infoLen;
  // create frame buffer without preamble, start or stop flags
  uint8_t* frameBuff = (uint8_t*)malloc((frameBuffLen + 2)*sizeof(uint8_t));
  uint8_t* frameBuffPtr = frameBuff;

  // set destination callsign - all address field bytes are shifted by one bit to make room for HDLC address extension bit
  memset(frameBuffPtr, ' ' << 1, RADIOLIB_AX25_MAX_CALLSIGN_LEN);
  for(size_t i = 0; i < strlen(ax25frame->destCallsign); i++) 
  {
      *(frameBuffPtr + i) = ax25frame->destCallsign[i] << 1;
  }
  frameBuffPtr += RADIOLIB_AX25_MAX_CALLSIGN_LEN;

  // set destination SSID
  *(frameBuffPtr++) = RADIOLIB_AX25_SSID_RESPONSE_DEST | RADIOLIB_AX25_SSID_RESERVED_BITS | (ax25frame->destSSID & 0x0F) << 1 | RADIOLIB_AX25_SSID_HDLC_EXTENSION_CONTINUE;

  // set source callsign - all address field bytes are shifted by one bit to make room for HDLC address extension bit
  memset(frameBuffPtr, ' ' << 1, RADIOLIB_AX25_MAX_CALLSIGN_LEN);
  for(size_t i = 0; i < strlen(ax25frame->srcCallsign); i++) 
  {
      *(frameBuffPtr + i) = ax25frame->srcCallsign[i] << 1;
  }
  frameBuffPtr += RADIOLIB_AX25_MAX_CALLSIGN_LEN;

  // set source SSID
  *(frameBuffPtr++) = RADIOLIB_AX25_SSID_COMMAND_SOURCE | RADIOLIB_AX25_SSID_RESERVED_BITS | (ax25frame->srcSSID & 0x0F) << 1 | RADIOLIB_AX25_SSID_HDLC_EXTENSION_CONTINUE;

  // set HDLC extension end bit
  *(frameBuffPtr - 1) |= RADIOLIB_AX25_SSID_HDLC_EXTENSION_END;

  // set sequence numbers of the frames that have it
  uint8_t controlField = ax25frame->control;
  if((ax25frame->control & 0x01) == 0) {
      // information frame, set both sequence numbers
      controlField |= ax25frame->rcvSeqNumber << 5;
      controlField |= ax25frame->sendSeqNumber << 1;
  } else if((ax25frame->control & 0x02) == 0) {
      // supervisory frame, set only receive sequence number
      controlField |= ax25frame->rcvSeqNumber << 5;
  }

  // set control field
  *(frameBuffPtr++) = controlField;

  // set PID field of the frames that have it
  if(ax25frame->protocolID != 0x00) {
      *(frameBuffPtr++) = ax25frame->protocolID;
  }

  // set info field of the frames that have it
  if(ax25frame->infoLen > 0) {
      memcpy(frameBuffPtr, ax25frame->info, ax25frame->infoLen);
      frameBuffPtr += ax25frame->infoLen;
  }

  // flip bit order
  for(size_t i = 0; i < frameBuffLen; i++) 
  {
      frameBuff[i] = reflect(frameBuff[i], 8);
  }

  uint16_t fcs = checksum(ax25frame, frameBuff, frameBuffLen);
  *(frameBuffPtr++) = (uint8_t)((fcs >> 8) & 0xFF);
  *(frameBuffPtr++) = (uint8_t)(fcs & 0xFF);

  // prepare buffer for the final frame (stuffed, with added preamble + flags and NRZI-encoded)
  // worst-case scenario: sequence of 1s, will have 120% of the original length, stuffed frame also includes both flags
  uint8_t* stuffedFrameBuff = (uint8_t*)malloc((ax25frame->preambleLen + 1 + (6*frameBuffLen)/5 + 2)*sizeof(uint8_t));

  // initialize buffer to all zeros
  memset(stuffedFrameBuff, 0x00, ax25frame->preambleLen + 1 + (6*frameBuffLen)/5 + 2);

  // stuff bits (skip preamble and both flags)
  uint16_t stuffedFrameBuffLenBits = 8*(ax25frame->preambleLen + 1);
  uint8_t count = 0;
  for(size_t i = 0; i < frameBuffLen + 2; i++) 
  {
    for(int8_t shift = 7; shift >= 0; shift--) 
    {
      uint16_t stuffedFrameBuffPos = stuffedFrameBuffLenBits + 7 - 2*(stuffedFrameBuffLenBits%8);
      if((frameBuff[i] >> shift) & 0x01) 
      {
        // copy 1 and increment counter
        SET_BIT_IN_ARRAY(stuffedFrameBuff, stuffedFrameBuffPos);
        stuffedFrameBuffLenBits++;
        count++;

        // check 5 consecutive 1s
        if(count == 5) 
        {
          // get the new position in stuffed frame
          stuffedFrameBuffPos = stuffedFrameBuffLenBits + 7 - 2*(stuffedFrameBuffLenBits%8);

          // insert 0 and reset counter
          CLEAR_BIT_IN_ARRAY(stuffedFrameBuff, stuffedFrameBuffPos);
          stuffedFrameBuffLenBits++;
          count = 0;
        }
      } 
      else 
      {
        // copy 0 and reset counter
        CLEAR_BIT_IN_ARRAY(stuffedFrameBuff, stuffedFrameBuffPos);
        stuffedFrameBuffLenBits++;
        count = 0;
      }
    }
  }

    // deallocate memory
    free(frameBuff);

    // set preamble bytes and start flag field
    for(uint16_t i = 0; i < ax25frame->preambleLen + 1; i++) 
        stuffedFrameBuff[i] = RADIOLIB_AX25_FLAG;

    // get stuffed frame length in bytes
    size_t stuffedFrameBuffLen = stuffedFrameBuffLenBits/8 + 1;
    uint8_t trailingLen = stuffedFrameBuffLenBits % 8;

    // set end flag field (may be split into two bytes due to misalignment caused by extra stuffing bits)
    if(trailingLen != 0) 
    {
        stuffedFrameBuffLen++;
        stuffedFrameBuff[stuffedFrameBuffLen - 2] |= RADIOLIB_AX25_FLAG >> trailingLen;
        stuffedFrameBuff[stuffedFrameBuffLen - 1] = RADIOLIB_AX25_FLAG << (8 - trailingLen);
    } 
    else 
    {
        stuffedFrameBuff[stuffedFrameBuffLen - 1] = RADIOLIB_AX25_FLAG;
    }

    // convert to NRZI
	for(size_t i = ax25frame->preambleLen + 1; i < stuffedFrameBuffLen*8; i++) {
		size_t currBitPos = i + 7 - 2*(i%8);
		size_t prevBitPos = (i - 1) + 7 - 2*((i - 1)%8);
		// Serial.print(prevBitPos);
		// Serial.print(" ");
		if(TEST_BIT_IN_ARRAY(stuffedFrameBuff, currBitPos)) {
			// bit is 1, no change, copy previous bit
			if(TEST_BIT_IN_ARRAY(stuffedFrameBuff, prevBitPos)) {
				SET_BIT_IN_ARRAY(stuffedFrameBuff, currBitPos);
			} else {
				CLEAR_BIT_IN_ARRAY(stuffedFrameBuff, currBitPos);
			}
		} else {
			// bit is 0, transition, copy inversion of the previous bit
			if(TEST_BIT_IN_ARRAY(stuffedFrameBuff, prevBitPos)) {
				CLEAR_BIT_IN_ARRAY(stuffedFrameBuff, currBitPos);
			} else {
				SET_BIT_IN_ARRAY(stuffedFrameBuff, currBitPos);
			}
		}
	}

    *stuffedFrameLen = stuffedFrameBuffLen;
    return stuffedFrameBuff;
}

uint32_t reflect(uint32_t in, uint8_t bits) {
  uint32_t res = 0;
  for(uint8_t i = 0; i < bits; i++) {
    res |= (((in & ((uint32_t)1 << i)) >> i) << (bits - i - 1));
  }
  return(res);
}

uint8_t AX25Frame_HDLC_Parser(ax25receiveframe_t* ax25frame , uint8_t** inputStuffedFrame, uint16_t* inputStuffedFrameLen)
{
  // prepare buffer for the unstuffed frame (only AX.25 frame)
  // worst-case scenario: sequence of 1s, will have 120% of the original length, stuffed frame also includes both flags
  uint8_t* ax25FrameBuff = (uint8_t*)malloc(*inputStuffedFrameLen - ax25frame->preambleLen);
  
  // initialize buffer to all zeros
  memset(ax25FrameBuff, 0x00, *inputStuffedFrameLen - ax25frame->preambleLen);

  // unstuff bits (skip preamble and both flags)
  uint16_t stuffedFrameBuffLenBits = 8*(ax25frame->preambleLen + 1);
  uint8_t count = 0;
  for(size_t i = 0; i < frameBuffLen + 2; i++) 
  {
    for(int8_t shift = 7; shift >= 0; shift--) 
    {
      uint16_t stuffedFrameBuffPos = stuffedFrameBuffLenBits + 7 - 2*(stuffedFrameBuffLenBits%8);
      if((frameBuff[i] >> shift) & 0x01) 
      {
        // copy 1 and increment counter
        SET_BIT_IN_ARRAY(stuffedFrameBuff, stuffedFrameBuffPos);
        stuffedFrameBuffLenBits++;
        count++;

        // check 5 consecutive 1s
        if(count == 5) 
        {
          // get the new position in stuffed frame
          stuffedFrameBuffPos = stuffedFrameBuffLenBits + 7 - 2*(stuffedFrameBuffLenBits%8);

          // insert 0 and reset counter
          CLEAR_BIT_IN_ARRAY(stuffedFrameBuff, stuffedFrameBuffPos);
          stuffedFrameBuffLenBits++;
          count = 0;
        }
      } 
      else 
      {
        // copy 0 and reset counter
        CLEAR_BIT_IN_ARRAY(stuffedFrameBuff, stuffedFrameBuffPos);
        stuffedFrameBuffLenBits++;
        count = 0;
      }
    }
  }
}

uint32_t checksum(ax25frame_t* ax25frame, uint8_t* buff, size_t len) {
  uint32_t crc = ax25frame->crc.init;
  size_t pos = 0;
  for(size_t i = 0; i < 8*len; i++) {
    if(i % 8 == 0) {
      uint32_t in = buff[pos++];
      if(ax25frame->crc.refIn) {
        in = reflect(in, 8);
      }
      crc ^= (in << (ax25frame->crc.size - 8));
    }

    if(crc & ((uint32_t)1 << (ax25frame->crc.size - 1))) {
      crc <<= (uint32_t)1;
      crc ^= ax25frame->crc.poly;
    } else {
      crc <<= (uint32_t)1;
    }
  }

  crc ^= ax25frame->crc.out;
  if(ax25frame->crc.refOut) {
    crc = reflect(crc, ax25frame->crc.size);
  }
  crc &= (uint32_t)0xFFFFFFFF >> (32 - ax25frame->crc.size);
  return(crc);
}

void ax25_nrzi_encode(uint8_t* input, uint8_t* output, uint16_t len)
{
  uint8_t prev_nrzi_bit = 0;
  uint8_t nrz_bit;
  uint8_t nrzi_bit;
  uint16_t i, j;

  for (i=0; i<len; i++) 
  {
    for (j=7; j>=0; j--)
    {
      nrz_bit = input[i]>>j & 0x01;

      if (nrz_bit == 0)
        nrzi_bit = prev_nrzi_bit ^ 1;
      else
        nrzi_bit = prev_nrzi_bit;

      if (nrzi_bit)
        output[i] |= (1 << j);
      else
        output[i] &= ~(1 << j);

      prev_nrzi_bit = nrzi_bit;
    }
  }
}

void ax25_nrzi_decode(uint8_t* input, uint8_t* output, uint16_t len)
{
  uint8_t prev_nrzi_bit = 0;
  uint8_t nrz_bit;
  uint8_t nrzi_bit;
  uint16_t i, j;

  for (i=0; i<len; i++) 
  {
    for (j=8; j>0; j--)
    {
      nrzi_bit = input[i]>>j & 0x01;

      if (nrzi_bit == prev_nrzi_bit)
        nrz_bit = 1;
      else
        nrz_bit = 0;

      if (nrz_bit)
        output[i] |= (1 << j);
      else
        output[i] &= ~(1 << j);

      prev_nrzi_bit = nrzi_bit;
    }
  }
}

void ax25_g3ruh_scrambler_init(uint32_t tap_mask)
{
  uint8_t i;
  d_tap_count = 0;

  for (i=0; i<32; i++) 
  {
    if ((tap_mask & 0x01) == 1) 
    {
      d_taps[d_tap_count] = i;
      d_tap_count++;
    }
    tap_mask = tap_mask >> 1;
  }

  d_shift_register = 0;
}

void ax25_g3ruh_scrambler(uint8_t* unscrambled, uint8_t* scrambled, uint16_t len)
{
  uint8_t unscrambled_bit;
  uint8_t scrambled_bit;
  uint32_t tap_bit;
  int16_t i, j, t;

  for (i=0; i<len; i++) 
  {
    for (j=7; j>=0; j--) 
    {
      unscrambled_bit = unscrambled[i]>>j & 0x01;
      d_shift_register <<= 1;

      scrambled_bit = unscrambled_bit;
      for (t=0; t<d_tap_count; t++) 
      {
        tap_bit = (d_shift_register >> d_taps[t]) & 0x01;
        scrambled_bit = scrambled_bit ^ tap_bit;
      }

      d_shift_register |= scrambled_bit;

      if (scrambled_bit)
        scrambled[i] |= (1 << j);
      else
        scrambled[i] &= ~(1 << j);
    }
  }
}

void ax25_g3ruh_descrambler(int8_t* scrambled, uint8_t* unscrambled, uint16_t len)
{
  uint8_t unscrambled_bit;
  uint8_t scrambled_bit;
  uint32_t tap_bit;
  int16_t i, j, t;

  for (i=0; i<len; i++) 
  {
    for (j=7; j>=0; j--) 
    {
      scrambled_bit = scrambled[i]>>j & 0x01;
      d_shift_register <<= 1;

      unscrambled_bit = scrambled_bit;
      for (t=0; t<d_tap_count; t++) 
      {
        tap_bit = (d_shift_register >> d_taps[t]) & 0x01;
        unscrambled_bit = unscrambled_bit ^ tap_bit;
      }

      d_shift_register |= scrambled_bit;

      if (unscrambled)
        unscrambled[i] |= (1 << j);
      else
        unscrambled[i] &= ~(1 << j);
    }
  }
}
