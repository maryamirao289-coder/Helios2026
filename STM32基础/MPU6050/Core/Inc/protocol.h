#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "main.h"
#include "crc16.h"


#define PROTOCOL_HEADER_1              0xAA
#define PROTOCOL_HEADER_2              0x55

#define PROTOCOL_CMD_ATTITUDE          0x01
#define PROTOCOL_CMD_RETRANSMIT        0x02
#define PROTOCOL_CMD_SET_GAINS         0x03

#define PROTOCOL_ATTITUDE_PAYLOAD_LEN  28
#define PROTOCOL_ATTITUDE_FRAME_LEN    36
#define PROTOCOL_RETRANSMIT_REQUEST_LEN    8
#define PROTOCOL_GAINS_PAYLOAD_LEN     8
#define PROTOCOL_SET_GAINS_FRAME_LEN   16

typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;

    float roll;
    float pitch;
    float yaw;

} Protocol_Attitude_t;


typedef enum
{
    PROTOCOL_OK = 0,

    PROTOCOL_ERROR_HEADER,
    PROTOCOL_ERROR_CMD,
    PROTOCOL_ERROR_LENGTH,
    PROTOCOL_ERROR_CRC

} Protocol_Status_t;


/* Build one attitude frame */
uint16_t Protocol_BuildAttitudeFrame(
    uint8_t *frame,
    uint16_t sequence,
    const Protocol_Attitude_t *attitude
);


/* Parse and verify one attitude frame */
Protocol_Status_t Protocol_ParseAttitudeFrame(
    const uint8_t *frame,
    uint16_t length,
    uint16_t *sequence,
    Protocol_Attitude_t *attitude
);

/* Build retransmit request frame */
uint16_t Protocol_BuildRetransmitRequest(
    uint8_t *frame,
    uint16_t sequence
);


/* Parse retransmit request frame */
Protocol_Status_t Protocol_ParseRetransmitRequest(
    const uint8_t *frame,
    uint16_t length,
    uint16_t *sequence
);
		
/* Build set-gains command frame */
uint16_t Protocol_BuildSetGainsFrame(
    uint8_t *frame,
    uint16_t sequence,
    float kp,
    float ki
);


/* Parse set-gains command frame */
Protocol_Status_t Protocol_ParseSetGainsFrame(
    const uint8_t *frame,
    uint16_t length,
    uint16_t *sequence,
    float *kp,
    float *ki
);
#endif