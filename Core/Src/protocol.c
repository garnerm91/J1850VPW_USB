/**
 * @file protocol.c
 * @brief Simple STX/ETX framed serial protocol handler for STM32 CubeIDE
 */

#include "protocol.h"
#include "j1850vpw.h"
#include <string.h>
#include <stdio.h>

/* ------------------------------------------------------------------ */
/*  Internal helpers                                                   */
/* ------------------------------------------------------------------ */

static void send_ack(SerialPort *port)
{
    uint8_t frame[] = { S_STX, 0x00, S_ACK, S_ETX };
    Serial_write(port, frame, sizeof(frame));
}

static void send_nack(SerialPort *port)
{
    uint8_t frame[] = { S_STX, 0x00, S_NACK, S_ETX };
    Serial_write(port, frame, sizeof(frame));
}

/* ------------------------------------------------------------------ */
/*  Command handlers                                                   */
/* ------------------------------------------------------------------ */

/**
 * CMD 0x01 — RX Mode
 * data[0]: 0x01 = enable RX, 0x00 = disable RX
 */
static void handle_rx_mode(SerialPort *port, J1850 *bus, uint8_t *data, uint8_t len)
{
    if (len < 1u) {
        send_nack(port);
        return;
    }

    J1850_setRxEnabled(bus, (data[0] == 0x01u) ? 1 : 0);

    send_ack(port);
}

/**
 * CMD 0x02 — Send
 * data[0..len-1]: raw J1850 bytes to transmit (CRC appended by driver)
 */
static void handle_send(SerialPort *port, J1850 *bus, uint8_t *data, uint8_t len)
{
    if (len == 0u) {
        send_nack(port);
        return;
    }

    if (J1850_send(bus, data, len)) {
        send_ack(port);
    } else {
        send_nack(port);
    }
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

void Protocol_init(ProtocolParser *parser)
{
    memset(parser, 0, sizeof(ProtocolParser));
}

void Protocol_process(ProtocolParser *parser, SerialPort *port, J1850 *bus)
{
    while (Serial_available(port)) {
        uint8_t b = (uint8_t)Serial_read(port);

        /* ---- Waiting for frame start ------------------------------ */
        if (!parser->inFrame) {
            if (b == S_STX) {
                parser->inFrame = 1;
                parser->len     = 0;
                parser->count   = 0;
                parser->cmd     = 0;
            }
            continue;
        }

        /* ---- LEN byte -------------------------------------------- */
        if (parser->len == 0u) {
            if (b == 0u || b > MAX_MESSAGE_SIZE) {
                send_nack(port);
                parser->inFrame = 0;
            } else {
                parser->len = b;
            }
            continue;
        }

        /* ---- CMD byte -------------------------------------------- */
        if (parser->cmd == 0u) {
            parser->cmd = b;
            continue;
        }

        /* ---- DATA bytes ------------------------------------------ */
        if (parser->count < (parser->len - 1u)) {
            if (parser->count >= MAX_MESSAGE_SIZE) {
                send_nack(port);
                parser->inFrame = 0;
                continue;
            }
            parser->buf[parser->count++] = b;
            continue;
        }

        /* ---- ETX byte -------------------------------------------- */
        if (b == S_ETX && parser->count == (parser->len - 1u)) {
            switch (parser->cmd) {
                case CMD_RX_MODE:
                    handle_rx_mode(port, bus, parser->buf, parser->count);
                    break;
                case CMD_SEND:
                    handle_send(port, bus, parser->buf, parser->count);
                    break;
                default:
                    send_nack(port);
                    break;
            }
        } else {
            send_nack(port);
        }

        parser->inFrame = 0;
    }
}
