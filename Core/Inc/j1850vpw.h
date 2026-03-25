/**
 * @file j1850vpw.h
 * @brief J1850 VPW driver for STM32 CubeIDE (pure C, HAL-based)
 *
 * Ported from the Arduino j1850 library by redheadedrod / GarnerM.
 *
 * Usage
 * -----
 *   #include "j1850vpw.h"
 *   #include "serial.h"
 *
 *   J1850 bus;
 *
 *   J1850_init(&bus, BUS_IN_GPIO_Port, BUS_IN_Pin,
 *                    BUS_OUT_GPIO_Port, BUS_OUT_Pin, &Serial1);
 *
 *   // In main loop:
 *   uint8_t msg[J1850_MAX_MSG_SIZE];
 *   if (J1850_accept(&bus, msg, true)) {
 *       // msg contains received frame (without CRC byte)
 *   }
 *
 *   // To send:
 *   uint8_t frame[] = { 0x68, 0x6A, 0xF1, 0x01, 0x00 };
 *   J1850_send(&bus, frame, 5);
 */

#ifndef J1850VPW_H
#define J1850VPW_H

#include "stm32c0xx_hal.h"
#include "serial.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  J1850 VPW timing constants (microseconds)                         */
/* ------------------------------------------------------------------ */
#define J1850_WAIT_100US      100u
#define J1850_TX_SHORT         64u
#define J1850_TX_LONG         128u
#define J1850_TX_SOF          200u
#define J1850_TX_EOF          280u
#define J1850_RX_SHORT_MIN     34u
#define J1850_RX_SHORT_MAX     96u
#define J1850_RX_LONG_MIN      96u
#define J1850_RX_LONG_MAX     163u
#define J1850_RX_SOF_MIN      163u
#define J1850_RX_SOF_MAX      239u
#define J1850_RX_EOD_MIN      163u
#define J1850_RX_IFS_MIN      280u
#define J1850_MAX_MSG_SIZE     12u   /* 12 bytes including CRC */

/* ------------------------------------------------------------------ */
/*  Status / error codes                                               */
/* ------------------------------------------------------------------ */
#define J1850_MSG_SEND_OK               1
#define J1850_MSG_ACCEPT_OK             2
#define J1850_ERR_MSG_TOO_LONG          3
#define J1850_ERR_NO_RESPONSE_100US     4
#define J1850_ERR_SOF_TIMEOUT           5
#define J1850_ERR_NOT_SOF               6
#define J1850_ERR_NOT_SHORT             7
#define J1850_ERR_CRC                   8

/* ------------------------------------------------------------------ */
/*  Driver state struct  — one per bus                                 */
/* ------------------------------------------------------------------ */
typedef struct {
    /* GPIO */
    GPIO_TypeDef *in_port;
    uint16_t      in_pin;
    GPIO_TypeDef *out_port;
    uint16_t      out_pin;

    /* Serial port for monitor output */
    SerialPort   *serial;

    /* State */
    int           rxEnabled;  /**< 1 = forward received frames to serial  */
    int           message;
    int           rx_nbyte;
    int           tx_nbyte;

    /* Pointers kept for monitor reporting */
    uint8_t      *rx_msg_buf;
    uint8_t      *tx_msg_buf;

    /* Internal TX staging buffer (used by J1850_send — holds data + CRC) */
    uint8_t       tx_buf[J1850_MAX_MSG_SIZE];

    /* DWT timer snapshot */
    uint32_t      time_tmp;
} J1850;

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

/**
 * Initialise the driver.
 * Call after SystemClock_Config() and MX_GPIO_Init().
 *
 * @param bus       pointer to a J1850 instance (zero-init before first use)
 * @param in_port   GPIO port for the bus RX pin,  e.g. BUS_IN_GPIO_Port
 * @param in_pin    GPIO pin  for the bus RX pin,  e.g. BUS_IN_Pin
 * @param out_port  GPIO port for the bus TX pin,  e.g. BUS_OUT_GPIO_Port
 * @param out_pin   GPIO pin  for the bus TX pin,  e.g. BUS_OUT_Pin
 * @param serial    SerialPort to use for monitor output (may be NULL to disable)
 */
void J1850_init(J1850       *bus,
                GPIO_TypeDef *in_port,  uint16_t in_pin,
                GPIO_TypeDef *out_port, uint16_t out_pin,
                SerialPort   *serial);

/**
 * Attempt to receive one J1850 frame.
 * Blocks until a frame is received or an error/timeout occurs.
 *
 * @param bus      pointer to J1850 instance
 * @param msg_buf  buffer to write received bytes into (must be >= J1850_MAX_MSG_SIZE)
 * @param check_crc  if true, validates the trailing CRC byte
 * @return true on successful receive, false on error (check bus->message)
 */
bool J1850_accept(J1850 *bus, uint8_t *msg_buf, bool check_crc);

/**
 * Transmit a J1850 frame.
 * Automatically appends a CRC byte before sending.
 *
 * @param bus      pointer to J1850 instance
 * @param msg_buf  data bytes to send (NOT including CRC — driver appends it)
 * @param nbytes   number of data bytes
 * @return true on success, false on error (check bus->message)
 */
bool J1850_send(J1850 *bus, uint8_t *msg_buf, int nbytes);

/**
 * Enable or disable forwarding of received J1850 frames to the serial port.
 * @param enabled  1 = forward RX frames, 0 = suppress RX frames
 */
void J1850_setRxEnabled(J1850 *bus, int enabled);

/**
 * Calculate J1850 CRC for a buffer.
 * Exposed publicly so the protocol layer can verify received frames if needed.
 */
uint8_t J1850_crc(uint8_t *msg_buf, int nbytes);

#ifdef __cplusplus
}
#endif

#endif /* J1850VPW_H */
