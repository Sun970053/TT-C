
#include "module.h"

uint32_t reverse_nbits(uint32_t b, uint8_t len)
{
    uint32_t mask = 0;
    mask = ~mask;
    while(len >>= 1)
    {
        mask ^= mask << len;
        b = (b & ~mask) >> len | (b & mask) << len;
    }
    return b;
}

uint16_t reverse_16bits(uint16_t b)
{
    b = (b & 0xFF00) >> 8 | (b & 0x00FF) << 8;
    b = (b & 0xF0F0) >> 4 | (b & 0x0F0F) << 4;
    b = (b & 0xCCCC) >> 2 | (b & 0x3333) << 2;
    b = (b & 0xAAAA) >> 1 | (b & 0x5555) << 1;
    return b;
}

uint8_t reverse_8bits(uint8_t b)
{
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}