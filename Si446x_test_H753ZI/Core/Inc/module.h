
#ifndef MODULE_H
#define MODULE_H

#include <stdint.h>

/*!
    \brief reverse size bits.
    \param b input number to reverse
    \param len length of the input number in bits.
    \returns The resulting checksum.
*/
uint32_t reverse_nbits(uint32_t b, uint8_t len);
uint16_t reverse_16bits(uint16_t b);
uint8_t reverse_8bits(uint8_t b);

#endif