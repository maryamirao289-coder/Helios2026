#include "protocol.h"
#include <string.h>


static void Protocol_WriteU16LE(
    uint8_t *buffer,
    uint16_t value
)
{
    buffer[0] = (uint8_t)(value & 0xFF);
    buffer[1] = (uint8_t)((value >> 8) & 0xFF);
}


static uint16_t Protocol_ReadU16LE(
    const uint8_t *buffer
)
{
    return
        ((uint16_t)buffer[0]) |
        ((uint16_t)buffer[1] << 8);
}


static void Protocol_WriteFloatLE(
    uint8_t *buffer,
    float value
)
{
    uint32_t raw;


    memcpy(
        &raw,
        &value,
        sizeof(float)
    );


    buffer[0] =
        (uint8_t)(raw & 0xFF);

    buffer[1] =
        (uint8_t)((raw >> 8) & 0xFF);

    buffer[2] =
        (uint8_t)((raw >> 16) & 0xFF);

    buffer[3] =
        (uint8_t)((raw >> 24) & 0xFF);
}


static float Protocol_ReadFloatLE(
    const uint8_t *buffer
)
{
    uint32_t raw;
    float value;


    raw =
        ((uint32_t)buffer[0]) |
        ((uint32_t)buffer[1] << 8) |
        ((uint32_t)buffer[2] << 16) |
        ((uint32_t)buffer[3] << 24);


    memcpy(
        &value,
        &raw,
        sizeof(float)
    );


    return value;
}


uint16_t Protocol_BuildAttitudeFrame(
    uint8_t *frame,
    uint16_t sequence,
    const Protocol_Attitude_t *attitude
)
{
    uint16_t crc;


    if ((frame == NULL) ||
        (attitude == NULL))
    {
        return 0;
    }


    /* Header */
    frame[0] = PROTOCOL_HEADER_1;
    frame[1] = PROTOCOL_HEADER_2;


    /* Command */
    frame[2] = PROTOCOL_CMD_ATTITUDE;


    /* Payload length */
    frame[3] =
        PROTOCOL_ATTITUDE_PAYLOAD_LEN;


    /* Sequence number */
    Protocol_WriteU16LE(
        &frame[4],
        sequence
    );


    /* Quaternion */
    Protocol_WriteFloatLE(
        &frame[6],
        attitude->q0
    );

    Protocol_WriteFloatLE(
        &frame[10],
        attitude->q1
    );

    Protocol_WriteFloatLE(
        &frame[14],
        attitude->q2
    );

    Protocol_WriteFloatLE(
        &frame[18],
        attitude->q3
    );


    /* Euler angles */
    Protocol_WriteFloatLE(
        &frame[22],
        attitude->roll
    );

    Protocol_WriteFloatLE(
        &frame[26],
        attitude->pitch
    );

    Protocol_WriteFloatLE(
        &frame[30],
        attitude->yaw
    );


    /*
     * CRC covers:
     *
     * CMD
     * LEN
     * SEQ
     * Payload
     *
     * Header AA 55 is not included.
     */
    crc = CRC16_CCITT_FALSE(
        &frame[2],
        32
    );


    Protocol_WriteU16LE(
        &frame[34],
        crc
    );


    return PROTOCOL_ATTITUDE_FRAME_LEN;
}


Protocol_Status_t Protocol_ParseAttitudeFrame(
    const uint8_t *frame,
    uint16_t length,
    uint16_t *sequence,
    Protocol_Attitude_t *attitude
)
{
    uint16_t received_crc;
    uint16_t calculated_crc;


    if ((frame == NULL) ||
        (sequence == NULL) ||
        (attitude == NULL))
    {
        return PROTOCOL_ERROR_LENGTH;
    }


    if (length != PROTOCOL_ATTITUDE_FRAME_LEN)
    {
        return PROTOCOL_ERROR_LENGTH;
    }


    if ((frame[0] != PROTOCOL_HEADER_1) ||
        (frame[1] != PROTOCOL_HEADER_2))
    {
        return PROTOCOL_ERROR_HEADER;
    }


    if (frame[2] != PROTOCOL_CMD_ATTITUDE)
    {
        return PROTOCOL_ERROR_CMD;
    }


    if (frame[3] !=
        PROTOCOL_ATTITUDE_PAYLOAD_LEN)
    {
        return PROTOCOL_ERROR_LENGTH;
    }

/*
 * Read sequence before CRC check.
 *
 * If CRC fails, master still knows
 * which frame should be retransmitted.
 */
    *sequence =
        Protocol_ReadU16LE(
            &frame[4]
        );

		
    received_crc =
        Protocol_ReadU16LE(
            &frame[34]
        );


    calculated_crc =
        CRC16_CCITT_FALSE(
            &frame[2],
            32
        );


    if (received_crc != calculated_crc)
    {
        return PROTOCOL_ERROR_CRC;
    }





    attitude->q0 =
        Protocol_ReadFloatLE(
            &frame[6]
        );

    attitude->q1 =
        Protocol_ReadFloatLE(
            &frame[10]
        );

    attitude->q2 =
        Protocol_ReadFloatLE(
            &frame[14]
        );

    attitude->q3 =
        Protocol_ReadFloatLE(
            &frame[18]
        );


    attitude->roll =
        Protocol_ReadFloatLE(
            &frame[22]
        );

    attitude->pitch =
        Protocol_ReadFloatLE(
            &frame[26]
        );

    attitude->yaw =
        Protocol_ReadFloatLE(
            &frame[30]
        );


    return PROTOCOL_OK;
}

uint16_t Protocol_BuildRetransmitRequest(
    uint8_t *frame,
    uint16_t sequence
)
{
    uint16_t crc;


    if (frame == NULL)
    {
        return 0;
    }


    frame[0] = PROTOCOL_HEADER_1;
    frame[1] = PROTOCOL_HEADER_2;

    frame[2] = PROTOCOL_CMD_RETRANSMIT;

    /*
     * Retransmit request has no payload.
     */
    frame[3] = 0;


    Protocol_WriteU16LE(
        &frame[4],
        sequence
    );


    /*
     * CRC covers:
     * CMD + LEN + SEQ
     *
     * 4 bytes total.
     */
    crc = CRC16_CCITT_FALSE(
        &frame[2],
        4
    );


    Protocol_WriteU16LE(
        &frame[6],
        crc
    );


    return PROTOCOL_RETRANSMIT_REQUEST_LEN;
}


Protocol_Status_t Protocol_ParseRetransmitRequest(
    const uint8_t *frame,
    uint16_t length,
    uint16_t *sequence
)
{
    uint16_t received_crc;
    uint16_t calculated_crc;


    if ((frame == NULL) ||
        (sequence == NULL))
    {
        return PROTOCOL_ERROR_LENGTH;
    }


    if (length != PROTOCOL_RETRANSMIT_REQUEST_LEN)
    {
        return PROTOCOL_ERROR_LENGTH;
    }


    if ((frame[0] != PROTOCOL_HEADER_1) ||
        (frame[1] != PROTOCOL_HEADER_2))
    {
        return PROTOCOL_ERROR_HEADER;
    }


    if (frame[2] != PROTOCOL_CMD_RETRANSMIT)
    {
        return PROTOCOL_ERROR_CMD;
    }


    if (frame[3] != 0)
    {
        return PROTOCOL_ERROR_LENGTH;
    }


    *sequence =
        Protocol_ReadU16LE(
            &frame[4]
        );


    received_crc =
        Protocol_ReadU16LE(
            &frame[6]
        );


    calculated_crc =
        CRC16_CCITT_FALSE(
            &frame[2],
            4
        );


    if (received_crc != calculated_crc)
    {
        return PROTOCOL_ERROR_CRC;
    }


    return PROTOCOL_OK;
}