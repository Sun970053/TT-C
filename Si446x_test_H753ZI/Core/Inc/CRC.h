
#ifndef CRC_H
#define CRC_H


#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include "module.h"

// CCITT CRC properties (used by AX.25)
#define RADIOLIB_CRC_CCITT_POLY                                 (0x1021)
#define RADIOLIB_CRC_CCITT_INIT                                 (0xFFFF)
#define RADIOLIB_CRC_CCITT_OUT                                  (0xFFFF)

typedef struct
{
    /*!
      \brief CRC size in bits.
    */
    uint8_t size;

    /*!
      \brief CRC polynomial.
    */
    uint32_t poly;

    /*!
      \brief Initial value.
    */
    uint32_t init;

    /*!
      \brief Final XOR value.
    */
    uint32_t out;

    /*!
      \brief Whether to reflect input bytes.
    */
    bool refIn;

    /*!
      \brief Whether to reflect the result.
    */
    bool refOut;
}RadioLibCRC_t;

/*!
    \brief Define the configuration of the CRC instance.
    \param size CRC size in bits.
    \param poly CRC polynomial.
    \param init Initial value.
    \param out Final XOR value.
    \param refIn Whether to reflect input bytes.
    \param refOut Whether to reflect the result.
    \returns The CRC instance.
*/
RadioLibCRC_t* createCRC(uint8_t size, uint32_t poly, uint32_t init, uint32_t out, bool refIn, bool refOut);

/*!
    \brief Calculate checksum of a buffer.
    \param buff Buffer to calculate the checksum over.
    \param len Size of the buffer in bytes.
    \returns The resulting checksum.
*/
uint32_t checksum(RadioLibCRC_t* CRCInstance, uint8_t* buff, size_t len);

#endif
