#include "crc16.h"


uint16_t CRC16_CCITT_FALSE(
    const uint8_t *data,
    uint16_t length
)
{
    uint16_t crc = 0xFFFF;
    uint16_t i;
    uint8_t bit;


    for (i = 0; i < length; i++)
    {
        crc ^= ((uint16_t)data[i] << 8);

        for (bit = 0; bit < 8; bit++)
        {
            if (crc & 0x8000)
            {
                crc =
                    (uint16_t)((crc << 1) ^ 0x1021);
            }
            else
            {
                crc <<= 1;
            }
        }
    }


    return crc;
}