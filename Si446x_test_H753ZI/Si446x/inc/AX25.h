
#ifndef AX25_H
#define AX25_H

#include <stdlib.h>
#include <stdint.h>
#include "CRC.h"
#include "Si446x.h"

/*
 * Enable static-only memory management: no dynamic allocation will be performed.
 * Warning: Large static arrays will be created in some methods. It is not advised to send large packets in this mode.
 */
#if !defined(RADIOLIB_STATIC_ONLY)
  #define RADIOLIB_STATIC_ONLY  (0)
#endif

// macros to access bits in byte array, from http://www.mathcs.emory.edu/~cheung/Courses/255/Syllabus/1-C-intro/bit-array.html
#define SET_BIT_IN_ARRAY(A, k)                                  ( A[(k/8)] |= (1 << (k%8)) )
#define CLEAR_BIT_IN_ARRAY(A, k)                                ( A[(k/8)] &= ~(1 << (k%8)) )
#define TEST_BIT_IN_ARRAY(A, k)                                 ( A[(k/8)] & (1 << (k%8)) )
#define GET_BIT_IN_ARRAY(A, k)                                  ( (A[(k/8)] & (1 << (k%8))) ? 1 : 0 )

// maximum callsign length in bytes
#define RADIOLIB_AX25_MAX_CALLSIGN_LEN                          6

// flag field                                                                 MSB   LSB   DESCRIPTION
#define RADIOLIB_AX25_FLAG                                      0b01111110  //  7     0     AX.25 frame start/end flag

// address field
#define RADIOLIB_AX25_SSID_COMMAND_DEST                         0b10000000  //  7     7     frame type: command (set in destination SSID)
#define RADIOLIB_AX25_SSID_COMMAND_SOURCE                       0b00000000  //  7     7                 command (set in source SSID)
#define RADIOLIB_AX25_SSID_RESPONSE_DEST                        0b00000000  //  7     7                 response (set in destination SSID)
#define RADIOLIB_AX25_SSID_RESPONSE_SOURCE                      0b10000000  //  7     7                 response (set in source SSID)
#define RADIOLIB_AX25_SSID_HAS_NOT_BEEN_REPEATED                0b00000000  //  7     7                 not repeated yet (set in repeater SSID)
#define RADIOLIB_AX25_SSID_HAS_BEEN_REPEATED                    0b10000000  //  7     7                 repeated (set in repeater SSID)
#define RADIOLIB_AX25_SSID_RESERVED_BITS                        0b01100000  //  6     5     reserved bits in SSID
#define RADIOLIB_AX25_SSID_HDLC_EXTENSION_CONTINUE              0b00000000  //  0     0     HDLC extension bit: next octet contains more address information
#define RADIOLIB_AX25_SSID_HDLC_EXTENSION_END                   0b00000001  //  0     0                         address field end

// control field
#define RADIOLIB_AX25_CONTROL_U_SET_ASYNC_BAL_MODE              0b01101100  //  7     2     U frame type: set asynchronous balanced mode (connect request)
#define RADIOLIB_AX25_CONTROL_U_SET_ASYNC_BAL_MODE_EXT          0b00101100  //  7     2                   set asynchronous balanced mode extended (connect request with module 128)
#define RADIOLIB_AX25_CONTROL_U_DISCONNECT                      0b01000000  //  7     2                   disconnect request
#define RADIOLIB_AX25_CONTROL_U_DISCONNECT_MODE                 0b00001100  //  7     2                   disconnect mode (system busy or disconnected)
#define RADIOLIB_AX25_CONTROL_U_UNNUMBERED_ACK                  0b01100000  //  7     2                   unnumbered acknowledge
#define RADIOLIB_AX25_CONTROL_U_FRAME_REJECT                    0b10000100  //  7     2                   frame reject
#define RADIOLIB_AX25_CONTROL_U_UNNUMBERED_INFORMATION          0b00000000  //  7     2                   unnumbered information
#define RADIOLIB_AX25_CONTROL_U_EXHANGE_IDENTIFICATION          0b10101100  //  7     2                   exchange ID
#define RADIOLIB_AX25_CONTROL_U_TEST                            0b11100000  //  7     2                   test
#define RADIOLIB_AX25_CONTROL_POLL_FINAL_ENABLED                0b00010000  //  4     4     control field poll/final bit: enabled
#define RADIOLIB_AX25_CONTROL_POLL_FINAL_DISABLED               0b00000000  //  4     4                                   disabled
#define RADIOLIB_AX25_CONTROL_S_RECEIVE_READY                   0b00000000  //  3     2     S frame type: receive ready (system ready to receive)
#define RADIOLIB_AX25_CONTROL_S_RECEIVE_NOT_READY               0b00000100  //  3     2                   receive not ready (TNC buffer full)
#define RADIOLIB_AX25_CONTROL_S_REJECT                          0b00001000  //  3     2                   reject (out of sequence or duplicate)
#define RADIOLIB_AX25_CONTROL_S_SELECTIVE_REJECT                0b00001100  //  3     2                   selective reject (single frame repeat request)
#define RADIOLIB_AX25_CONTROL_INFORMATION_FRAME                 0b00000000  //  0     0     frame type: information (I frame)
#define RADIOLIB_AX25_CONTROL_SUPERVISORY_FRAME                 0b00000001  //  1     0                 supervisory (S frame)
#define RADIOLIB_AX25_CONTROL_UNNUMBERED_FRAME                  0b00000011  //  1     0                 unnumbered (U frame)

// protocol identifier field
#define RADIOLIB_AX25_PID_ISO_8208                              0x01
#define RADIOLIB_AX25_PID_TCP_IP_COMPRESSED                     0x06
#define RADIOLIB_AX25_PID_TCP_IP_UNCOMPRESSED                   0x07
#define RADIOLIB_AX25_PID_SEGMENTATION_FRAGMENT                 0x08
#define RADIOLIB_AX25_PID_TEXNET_DATAGRAM_PROTOCOL              0xC3
#define RADIOLIB_AX25_PID_LINK_QUALITY_PROTOCOL                 0xC4
#define RADIOLIB_AX25_PID_APPLETALK                             0xCA
#define RADIOLIB_AX25_PID_APPLETALK_ARP                         0xCB
#define RADIOLIB_AX25_PID_ARPA_INTERNET_PROTOCOL                0xCC
#define RADIOLIB_AX25_PID_ARPA_ADDRESS_RESOLUTION               0xCD
#define RADIOLIB_AX25_PID_FLEXNET                               0xCE
#define RADIOLIB_AX25_PID_NET_ROM                               0xCF
#define RADIOLIB_AX25_PID_NO_LAYER_3                            0xF0
#define RADIOLIB_AX25_PID_ESCAPE_CHARACTER                      0xFF

/*!
  \brief No error, method executed successfully.
*/
#define RADIOLIB_ERR_NONE                                       (0)

/*!
  \brief Timed out waiting for transmission finish.
*/
#define RADIOLIB_ERR_TX_TIMEOUT                                 (-5)

/*!
  \brief The provided callsign is invalid.

  The specified callsign is longer than 6 ASCII characters.
*/
#define RADIOLIB_ERR_INVALID_CALLSIGN                           (-801)

/*!
  \brief The provided repeater configuration is invalid.

  The specified number of repeaters does not match number of repeater IDs or their callsigns.
*/
#define RADIOLIB_ERR_INVALID_NUM_REPEATERS                      (-802)

/*!
  \brief One of the provided repeater callsigns is invalid.

  The specified callsign is longer than 6 ASCII characters.
*/
#define RADIOLIB_ERR_INVALID_REPEATER_CALLSIGN                 (-803)

typedef struct
{
    /*!
      \brief Callsign of the destination station.
    */
    char destCallsign[RADIOLIB_AX25_MAX_CALLSIGN_LEN + 1];

    /*!
      \brief SSID of the destination station.
    */
    uint8_t destSSID;

    /*!
      \brief Callsign of the source station.
    */
    char srcCallsign[RADIOLIB_AX25_MAX_CALLSIGN_LEN + 1];

    /*!
      \brief SSID of the source station.
    */
    uint8_t srcSSID;

    /*!
      \brief Number of repeaters to be used.
    */
    uint8_t numRepeaters;

    /*!
      \brief The control field.
    */
    uint8_t control;

    /*!
      \brief The protocol identifier (PID) field.
    */
    uint8_t protocolID;

    /*!
      \brief Number of bytes in the information field.
    */
    uint16_t infoLen;

    /*!
      \brief Receive sequence number.
    */
    uint8_t rcvSeqNumber;

    /*!
      \brief Send sequence number.
    */
    uint16_t sendSeqNumber;

    #if !RADIOLIB_STATIC_ONLY
      /*!
        \brief The info field.
      */
      uint8_t* info;

      /*!
        \brief Array of repeater callsigns.
      */
      char** repeaterCallsigns;

      /*!
        \brief Array of repeater SSIDs.
      */
      uint8_t* repeaterSSIDs;
    #else
      /*!
        \brief The info field.
      */
      uint8_t info[RADIOLIB_STATIC_ARRAY_SIZE];

      /*!
        \brief Array of repeater callsigns.
      */
      char repeaterCallsigns[8][RADIOLIB_AX25_MAX_CALLSIGN_LEN + 1];

      /*!
        \brief Array of repeater SSIDs.
      */
      uint8_t repeaterSSIDs[8];
    #endif
}AX25Frame_t;

typedef struct
{
  char sourceCallsign[RADIOLIB_AX25_MAX_CALLSIGN_LEN + 1];
  uint8_t sourceSSID;
  uint16_t preambleLen;
}AX25Client_t;


/*!
  \brief Default constructor.
  \param destCallsign Callsign of the destination station.
  \param destSSID SSID of the destination station.
  \param srcCallsign Callsign of the source station.
  \param srcSSID SSID of the source station.
  \param control The control field.
  \param protocolID The protocol identifier (PID) field. Set to zero if the frame doesn't have this field.
  \param info Information field, in the form of arbitrary binary buffer.
  \param infoLen Number of bytes in the information field.
*/
AX25Frame_t* createAX25Frame(const char* destCallsign, uint8_t destSSID, const char* srcCallsign, uint8_t srcSSID, 
  uint8_t control, uint8_t protocolID, uint8_t* info, uint16_t infoLen);

/*!
  \brief Method to set the repeater callsigns and SSIDs.
  \param repeaterCallsigns Array of repeater callsigns in the form of null-terminated C-strings.
  \param repeaterSSIDs Array of repeater SSIDs.
  \param numRepeaters Number of repeaters, maximum is 8.
  \returns \ref status_codes
*/
int16_t setRepeaters(AX25Frame_t* ax25frame, char** repeaterCallsigns, uint8_t* repeaterSSIDs, uint8_t numRepeaters);

/*!
  \brief Method to set receive sequence number.
  \param seqNumber Sequence number to set, 0 to 7.
*/
void setRecvSequence(AX25Frame_t* ax25frame, uint8_t seqNumber);

/*!
  \brief Method to set send sequence number.
  \param seqNumber Sequence number to set, 0 to 7.
*/
void setSendSequence(AX25Frame_t* ax25frame, uint8_t seqNumber);

/*!
  \brief Set AFSK tone correction offset. On some platforms, this is required to get the audio produced
  by the setup to match the expected 1200/2200 Hz tones.
  \param mark Positive or negative correction offset for mark audio frequency in Hz.
  \param space Positive or negative correction offset for space audio frequency in Hz.
  \param length Audio tone length modifier, defaults to 1.0.
  \returns \ref status_codes
*/
int16_t setCorrection(int16_t mark, int16_t space, float length);

/*!
  \brief Initialization method.
  \param srcCallsign Callsign of the source station.
  \param srcSSID 4-bit SSID of the source station (in case there are more stations with the same callsign).
  Defaults to 0.
  \param preLen Number of "preamble" bytes (RADIOLIB_AX25_FLAG) sent ahead of the actual AX.25 frame.
  Does not include the first RADIOLIB_AX25_FLAG byte, which is considered part of the frame. Defaults to 8.
  \returns \ref status_codes
*/
AX25Client_t* beginAX25Client(const char* srcCallsign, uint8_t srcSSID, uint8_t preLen);

/*!
  \brief Transmit unnumbered information (UI) frame.
  \param str Data to be sent.
  \param destCallsign Callsign of the destination station.
  \param destSSID 4-bit SSID of the destination station (in case there are more stations with the same callsign).
  Defaults to 0.
  \returns \ref status_codes
*/
int16_t transmit(const char* str, const char* destCallsign, uint8_t destSSID);

/*!
  \brief Transmit arbitrary AX.25 frame.
  \param frame Frame to be sent.
  \returns \ref status_codes
*/
int16_t sendFrame(AX25Frame_t* ax25frame, AX25Client_t* ax25client);

#endif
