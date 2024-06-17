
#include "CRC.h"

RadioLibCRC_t* createCRC(uint8_t size, uint32_t poly, uint32_t init, uint32_t out, bool refIn, bool refOut)
{
    RadioLibCRC_t* CRCInstance = (RadioLibCRC_t*)malloc(sizeof(RadioLibCRC_t));
    CRCInstance->size = size;
    CRCInstance->poly = poly;
    CRCInstance->init = init;
    CRCInstance->refIn = false;
    CRCInstance->refOut = false;
    return CRCInstance;
}

uint32_t checksum(RadioLibCRC_t* CRCInstance, uint8_t* buff, size_t len)
{
    uint32_t crc = CRCInstance->init;
    size_t pos = 0;
    for(size_t i = 0; i < 8*len; i++)
    {
        if(i % 8 == 0)
        {
            uint32_t in = buff[pos++];
            if(CRCInstance->refIn)
            {
                in = reverse_8bits(in);
            }
            crc ^= (in << (CRCInstance->size - 8));
        }

        if(crc & ((uint32_t)1 << (CRCInstance->size - 1))) 
        {
            crc <<= (uint32_t)1;
            crc ^= CRCInstance->poly;
        } 
        else 
        {
            crc <<= (uint32_t)1;
        }
    }

    crc ^= CRCInstance->out;
	if(CRCInstance->refOut)
	{
		crc = reverse_nbits(crc, CRCInstance->size);
	}
	crc &= (uint32_t)0xFFFFFFFF >> (32 - CRCInstance->size);
	return(crc);
}
