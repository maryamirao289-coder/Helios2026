#ifndef __CRC16_H
#define __CRC16_H

#include <stdint.h>

uint16_t CRC16_CCITT_FALSE(
    const uint8_t *data,
    uint16_t length
);

#endif