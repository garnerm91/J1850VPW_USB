/**
 * @file protocol.h
 * @brief Simple STX/ETX framed serial protocol handler
 *
 * Frame format:
 *   [ STX | LEN | CMD | DATA[0..LEN-1] | ETX ]
 *
 * Commands:
 *   0x01  RX Mode  — data[0]: 0x01 = enable, 0x00 = disable
 *   0x02  Send     — data[0..LEN-1]: J1850 bytes to transmit
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "serial.h"
#include "j1850vpw.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  Protocol constants                                                 */
/* ------------------------------------------------------------------ */
#define S_STX            0x02u
#define S_ETX            0x03u
#define S_ACK            0x06u
#define S_NACK           0x15u

#define CMD_RX_MODE      0x01u
#define CMD_SEND         0x02u

#define MAX_MESSAGE_SIZE 12u   /* matches J1850_MAX_MSG_SIZE */

/* ------------------------------------------------------------------ */
/*  Parser state                                                       */
/* ------------------------------------------------------------------ */
typedef struct {
    uint8_t  inFrame;
    uint8_t  len;
    uint8_t  count;
    uint8_t  cmd;
    uint8_t  buf[MAX_MESSAGE_SIZE];
} ProtocolParser;

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

void Protocol_init(ProtocolParser *parser);

/**
 * Call in your main loop to drain incoming serial bytes and dispatch
 * complete frames to the appropriate J1850 command handler.
 *
 * @param parser  parser state (call Protocol_init once before first use)
 * @param port    serial port to read commands from / send ACK/NACK on
 * @param bus     J1850 bus instance to act on
 */
void Protocol_process(ProtocolParser *parser, SerialPort *port, J1850 *bus);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_H */
