#include "AX25.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

AX25Frame_t* createAX25Frame(const char* destCallsign, uint8_t destSSID, const char* srcCallsign, uint8_t srcSSID, uint8_t control, uint8_t protocolID, uint8_t* info, uint16_t infoLen)
{
    AX25Frame_t* ax25frame = (AX25Frame_t*)malloc(sizeof(AX25Frame_t));
    
    // destination callsign/SSID
    memcpy(ax25frame->destCallsign, destCallsign, RADIOLIB_AX25_MAX_CALLSIGN_LEN);
    ax25frame->destCallsign[strlen(destCallsign)] = '\0';
    ax25frame->destSSID = destSSID;
    
    // source callsign/SSID
    memcpy(ax25frame->srcCallsign, srcCallsign, RADIOLIB_AX25_MAX_CALLSIGN_LEN);
    ax25frame->srcCallsign[strlen(srcCallsign)] = '\0';
    ax25frame->srcSSID = srcSSID;

    // set repeaters
    ax25frame->numRepeaters = 0;
    #if !RADIOLIB_STATIC_ONLY
        ax25frame->repeaterCallsigns = NULL;
        ax25frame->repeaterSSIDs = NULL;
    #endif

    // control field
    ax25frame->control = control;
    
    // sequence numbers
    ax25frame->rcvSeqNumber = 0;
    ax25frame->sendSeqNumber = 0;

    // PID field
    ax25frame->protocolID = protocolID;

    // info field
    ax25frame->infoLen = infoLen;
    if(infoLen > 0) {
        #if !RADIOLIB_STATIC_ONLY
            ax25frame->info = (uint8_t*)malloc(infoLen*sizeof(uint8_t));
        #endif
        memcpy(ax25frame->info, info, infoLen);
    }

    return ax25frame;
}

int16_t setRepeaters(AX25Frame_t* ax25frame, char** repeaterCallsigns, uint8_t* repeaterSSIDs, uint8_t numRepeaters)
{
    // check number of repeaters
    if((numRepeaters < 1) || (numRepeaters > 8))
        return(RADIOLIB_ERR_INVALID_NUM_REPEATERS);

    // check repeater configuration
    if((repeaterCallsigns == NULL) || (repeaterSSIDs == NULL)) 
        return(RADIOLIB_ERR_INVALID_NUM_REPEATERS);

    for(uint16_t i = 0; i < numRepeaters; i++) 
    {
        if(strlen(repeaterCallsigns[i]) > RADIOLIB_AX25_MAX_CALLSIGN_LEN)
            return(RADIOLIB_ERR_INVALID_REPEATER_CALLSIGN);
    }

    // create buffers
    #if !RADIOLIB_STATIC_ONLY
        ax25frame->repeaterCallsigns = (char**)malloc(numRepeaters*sizeof(char*));
        for(uint8_t i = 0; i < numRepeaters; i++) 
        {
            ax25frame->repeaterCallsigns[i] = (char*)malloc((strlen(repeaterCallsigns[i]) + 1)*sizeof(char));
        }
        ax25frame->repeaterSSIDs = (uint8_t*)malloc(numRepeaters*sizeof(uint8_t));
    #endif

    // copy data
    ax25frame->numRepeaters = numRepeaters;
    for(uint8_t i = 0; i < numRepeaters; i++) 
    {
        memcpy(ax25frame->repeaterCallsigns[i], repeaterCallsigns[i], strlen(repeaterCallsigns[i]));
        ax25frame->repeaterCallsigns[i][strlen(repeaterCallsigns[i])] = '\0';
    }
    memcpy(ax25frame->repeaterSSIDs, repeaterSSIDs, numRepeaters);
    
    return(RADIOLIB_ERR_NONE);
}

void setRecvSequence(AX25Frame_t* ax25frame, uint8_t seqNumber) {
  ax25frame->rcvSeqNumber = seqNumber;
}

void setSendSequence(AX25Frame_t* ax25frame, uint8_t seqNumber) {
  ax25frame->sendSeqNumber = seqNumber;
}

AX25Client_t* beginAX25Client(const char* srcCallsign, uint8_t srcSSID, uint8_t preLen) {
    // create AX25 Client object
    AX25Client_t* ax25client = (AX25Client_t*)malloc(sizeof(AX25Client_t));
    
    // set source SSID
    ax25client->sourceSSID = srcSSID;

    // check source callsign length (6 characters max)
    if(strlen(srcCallsign) > RADIOLIB_AX25_MAX_CALLSIGN_LEN) {
        return NULL;
    }

    // copy callsign
    memcpy(ax25client->sourceCallsign, srcCallsign, strlen(srcCallsign));
    ax25client->sourceCallsign[strlen(srcCallsign)] = '\0';

    // save preamble length
    ax25client->preambleLen = preLen;

    // configure for direct mode
    // #if !RADIOLIB_EXCLUDE_AFSK
    // if(bellModem != nullptr) {
    //     return(phyLayer->startDirect());
    // }
    // #endif
    return ax25client;
}

int16_t sendFrame(AX25Frame_t* ax25frame, AX25Client_t* ax25client) {
    // check destination callsign length (6 characters max)
    if(strlen(ax25frame->destCallsign) > RADIOLIB_AX25_MAX_CALLSIGN_LEN) {
        return(RADIOLIB_ERR_INVALID_CALLSIGN);
    }

    // check repeater configuration
    #if !RADIOLIB_STATIC_ONLY
    if(!(((ax25frame->repeaterCallsigns == NULL) && (ax25frame->repeaterSSIDs == NULL) && (ax25frame->numRepeaters == 0)) ||
         ((ax25frame->repeaterCallsigns != NULL) && (ax25frame->repeaterSSIDs != NULL) && (ax25frame->numRepeaters != 0)))) {
            return(RADIOLIB_ERR_INVALID_NUM_REPEATERS);
    }
    for(uint16_t i = 0; i < ax25frame->numRepeaters; i++) {
        if(strlen(ax25frame->repeaterCallsigns[i]) > RADIOLIB_AX25_MAX_CALLSIGN_LEN)
            return(RADIOLIB_ERR_INVALID_REPEATER_CALLSIGN);
    }
    #endif

    // calculate frame length without FCS (destination address, source address, repeater addresses, control, PID, info)
    size_t frameBuffLen = ((2 + ax25frame->numRepeaters)*(RADIOLIB_AX25_MAX_CALLSIGN_LEN + 1)) + 1 + 1 + ax25frame->infoLen;
    // create frame buffer without preamble, start or stop flags
    #if !RADIOLIB_STATIC_ONLY
    uint8_t* frameBuff = (uint8_t*)malloc((frameBuffLen + 2)*sizeof(uint8_t));
    #else
    uint8_t frameBuff[RADIOLIB_STATIC_ARRAY_SIZE];
    #endif
    
    uint8_t* frameBuffPtr = frameBuff;

    // set destination callsign - all address field bytes are shifted by one bit to make room for HDLC address extension bit
    memset(frameBuffPtr, ' ' << 1, RADIOLIB_AX25_MAX_CALLSIGN_LEN);
    for(size_t i = 0; i < strlen(ax25frame->destCallsign); i++) {
        *(frameBuffPtr + i) = ax25frame->destCallsign[i] << 1;
    }
    frameBuffPtr += RADIOLIB_AX25_MAX_CALLSIGN_LEN;

    // set destination SSID
    *(frameBuffPtr++) = RADIOLIB_AX25_SSID_RESPONSE_DEST | RADIOLIB_AX25_SSID_RESERVED_BITS | (ax25frame->destSSID & 0x0F) << 1 | RADIOLIB_AX25_SSID_HDLC_EXTENSION_CONTINUE;

    // set source callsign - all address field bytes are shifted by one bit to make room for HDLC address extension bit
    memset(frameBuffPtr, ' ' << 1, RADIOLIB_AX25_MAX_CALLSIGN_LEN);
    for(size_t i = 0; i < strlen(ax25frame->srcCallsign); i++) {
        *(frameBuffPtr + i) = ax25frame->srcCallsign[i] << 1;
    }
    frameBuffPtr += RADIOLIB_AX25_MAX_CALLSIGN_LEN;

    // set source SSID
    *(frameBuffPtr++) = RADIOLIB_AX25_SSID_COMMAND_SOURCE | RADIOLIB_AX25_SSID_RESERVED_BITS | (ax25frame->srcSSID & 0x0F) << 1 | RADIOLIB_AX25_SSID_HDLC_EXTENSION_CONTINUE;

    // set repeater callsigns
    for(uint16_t i = 0; i < ax25frame->numRepeaters; i++) 
    {
        memset(frameBuffPtr, ' ' << 1, RADIOLIB_AX25_MAX_CALLSIGN_LEN);
        for(size_t j = 0; j < strlen(ax25frame->repeaterCallsigns[i]); j++) {
            *(frameBuffPtr + j) = ax25frame->repeaterCallsigns[i][j] << 1;
        }
        frameBuffPtr += RADIOLIB_AX25_MAX_CALLSIGN_LEN;
        *(frameBuffPtr++) = RADIOLIB_AX25_SSID_HAS_NOT_BEEN_REPEATED | RADIOLIB_AX25_SSID_RESERVED_BITS | (ax25frame->repeaterSSIDs[i] & 0x0F) << 1 | RADIOLIB_AX25_SSID_HDLC_EXTENSION_CONTINUE;
    }

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
  for(size_t i = 0; i < frameBuffLen; i++) {
    frameBuff[i] = reverse_8bits(frameBuff[i]);
  }

  // calculate
  RadioLibCRC_t* CRCInstance = createCRC(16, RADIOLIB_CRC_CCITT_POLY, RADIOLIB_CRC_CCITT_INIT, RADIOLIB_CRC_CCITT_OUT, false, false);
  uint16_t fcs = checksum(CRCInstance, frameBuff, frameBuffLen);
  *(frameBuffPtr++) = (uint8_t)((fcs >> 8) & 0xFF);
  *(frameBuffPtr++) = (uint8_t)(fcs & 0xFF);

  // prepare buffer for the final frame (stuffed, with added preamble + flags and NRZI-encoded)
  #if !RADIOLIB_STATIC_ONLY
    // worst-case scenario: sequence of 1s, will have 120% of the original length, stuffed frame also includes both flags
    uint8_t* stuffedFrameBuff = (uint8_t*)malloc((ax25client->preambleLen + 1 + (6*frameBuffLen)/5 + 2)*sizeof(uint8_t));
  #else
    uint8_t stuffedFrameBuff[RADIOLIB_STATIC_ARRAY_SIZE];
  #endif

  // initialize buffer to all zeros
  memset(stuffedFrameBuff, 0x00, ax25client->preambleLen + 1 + (6*frameBuffLen)/5 + 2);

  // stuff bits (skip preamble and both flags)
  uint16_t stuffedFrameBuffLenBits = 8*(ax25client->preambleLen + 1);
  uint8_t count = 0;
  for(size_t i = 0; i < frameBuffLen + 2; i++) {
    for(int8_t shift = 7; shift >= 0; shift--) {
      uint16_t stuffedFrameBuffPos = stuffedFrameBuffLenBits + 7 - 2*(stuffedFrameBuffLenBits%8);
      if((frameBuff[i] >> shift) & 0x01) {
        // copy 1 and increment counter
        SET_BIT_IN_ARRAY(stuffedFrameBuff, stuffedFrameBuffPos);
        stuffedFrameBuffLenBits++;
        count++;

        // check 5 consecutive 1s
        if(count == 5) {
          // get the new position in stuffed frame
          stuffedFrameBuffPos = stuffedFrameBuffLenBits + 7 - 2*(stuffedFrameBuffLenBits%8);

          // insert 0 and reset counter
          CLEAR_BIT_IN_ARRAY(stuffedFrameBuff, stuffedFrameBuffPos);
          stuffedFrameBuffLenBits++;
          count = 0;
        }

      } else {
        // copy 0 and reset counter
        CLEAR_BIT_IN_ARRAY(stuffedFrameBuff, stuffedFrameBuffPos);
        stuffedFrameBuffLenBits++;
        count = 0;
      }

    }
  }

  // deallocate memory
  #if !RADIOLIB_STATIC_ONLY
    free(frameBuff);
  #endif

  // set preamble bytes and start flag field
  for(uint16_t i = 0; i < ax25client->preambleLen + 1; i++) {
    stuffedFrameBuff[i] = RADIOLIB_AX25_FLAG;
  }

  // get stuffed frame length in bytes
  size_t stuffedFrameBuffLen = stuffedFrameBuffLenBits/8 + 1;
  uint8_t trailingLen = stuffedFrameBuffLenBits % 8;

  // set end flag field (may be split into two bytes due to misalignment caused by extra stuffing bits)
  if(trailingLen != 0) {
    stuffedFrameBuffLen++;
    stuffedFrameBuff[stuffedFrameBuffLen - 2] |= RADIOLIB_AX25_FLAG >> trailingLen;
    stuffedFrameBuff[stuffedFrameBuffLen - 1] = RADIOLIB_AX25_FLAG << (8 - trailingLen);
  } else {
    stuffedFrameBuff[stuffedFrameBuffLen - 1] = RADIOLIB_AX25_FLAG;
  }

	printf("HDLC : ");
	for(int i = 0 ; i < stuffedFrameBuffLen; i++)
	{
		printf("%02X ", stuffedFrameBuff[i]);
	}
	printf("\r\n");

  // convert to NRZI
  for(size_t i = ax25client->preambleLen + 1; i < stuffedFrameBuffLen*8; i++) {
    size_t currBitPos = i + 7 - 2*(i%8);
    size_t prevBitPos = (i - 1) + 7 - 2*((i - 1)%8);
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

  // transmit
  int16_t state = RADIOLIB_ERR_NONE;
  // #if !RADIOLIB_EXCLUDE_AFSK
  // if(bellModem != nullptr) {
  //   bellModem->idle();

  //   // iterate over all bytes in the buffer
  //   for(uint32_t i = 0; i < stuffedFrameBuffLen; i++) {
  //     bellModem->write(stuffedFrameBuff[i]);
  //   }

  //   bellModem->standby();

  // } else {
  // #endif
  printf("NRZI : ");
  for(int i = 0 ; i < stuffedFrameBuffLen; i++)
  {
	  printf("%02X ", stuffedFrameBuff[i]);
  }
  printf("\r\n");
//  uint8_t* g3ruhstuffedFrameBuff = Si446x_g3ruh_scrambler(stuffedFrameBuff, stuffedFrameBuffLen);
//
//  printf("G3RUH: ");
//    for(int i = 0 ; i < stuffedFrameBuffLen; i++)
//    {
//  	  printf("%02X ", g3ruhstuffedFrameBuff[i]);
//    }
//    printf("\r\n");
   if(Si446x_transmit(stuffedFrameBuffLen, stuffedFrameBuff) == 1)
   {
	   state = RADIOLIB_ERR_NONE;
   }
   else
   {
	   state = RADIOLIB_ERR_TX_TIMEOUT;
   }
  // #if !RADIOLIB_EXCLUDE_AFSK
  // }
  // #endif

  // deallocate memory
  #if !RADIOLIB_STATIC_ONLY
    free(stuffedFrameBuff);
  #endif

  return(state);
}

