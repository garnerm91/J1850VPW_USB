/**
 * @file j1850vpw.c
 * @brief J1850 VPW driver for STM32 CubeIDE (pure C, HAL-based)
 *
 * Ported from the Arduino j1850 library by redheadedrod / GarnerM.
 *
 * Arduino -> STM32 mapping
 * ------------------------
 *  micros()              -> SysTick-based elapsed µs (Cortex-M0 compatible)
 *  delayMicroseconds()   -> NOP spin-loop
 *  digitalWrite(pin,HIGH)-> HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET)
 *  digitalRead(pin)      -> HAL_GPIO_ReadPin(port, pin)
 *  pinMode()             -> handled by CubeMX / MX_GPIO_Init()
 *  Serial.print()        -> Serial_print() from serial.h
 *
 * Timing notes (Cortex-M0)
 * ------------------------
 * Cortex-M0 has no DWT cycle counter, so we use two approaches:
 *
 *  delay_us()   — NOP spin-loop calibrated to SYSCLK_MHZ.
 *                 Each loop iteration is ~4 cycles on M0 (no pipeline).
 *                 Adjust SYSCLK_MHZ if your clock differs from 48 MHz.
 *
 *  start_timer() / read_timer_us()
 *               — SysTick snapshot for measuring pulse widths.
 *                 SysTick counts DOWN from LOAD to 0 at SYSCLK rate.
 *                 Handles the single wrap that can occur between
 *                 start_timer() and read_timer_us() for pulses up to
 *                 ~89 ms at 48 MHz (LOAD = 47999 for 1 ms tick).
 */
#include "protocol.h"
#include "j1850vpw.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/*  Clock configuration                                                */
/*  Set SYSCLK_MHZ to match your CubeMX clock configuration.         */
/* ------------------------------------------------------------------ */
#ifndef SYSCLK_MHZ
#define SYSCLK_MHZ      24u     /* default: 24 MHz - override in project defines if different */
#endif
#define CYCLES_PER_US   SYSCLK_MHZ

/* ------------------------------------------------------------------ */
/*  SysTick microsecond timer and delay                                */
/*                                                                     */
/*  SysTick->VAL counts DOWN from SysTick->LOAD to 0, then reloads.  */
/*  We use it for BOTH delay_us() and pulse-width measurement so that */
/*  everything is driven by the same hardware source and there is no  */
/*  NOP calibration problem regardless of clock speed or wait states. */
/*                                                                     */
/*  Maximum measurable interval before wrap ambiguity:                */
/*    At 24 MHz with 1 ms HAL tick: LOAD = 23999 cycles = 1000 us    */
/*  All J1850 timings are well under 1 ms so one wrap is all we need. */
/* ------------------------------------------------------------------ */

/** Internal: cycles elapsed since a SysTick snapshot value. */
static uint32_t systick_elapsed_cycles(uint32_t snapshot)
{
    uint32_t now  = SysTick->VAL;
    uint32_t load = SysTick->LOAD;
     //If snapshot < now the counter has wrapped once.
    if (snapshot >= now)
        return snapshot - now;
    else
        return (load - now) + snapshot;
}

/** Snapshot the SysTick counter as a start reference. */
static void start_timer(J1850 *bus)
{
    bus->time_tmp = SysTick->VAL;
}

/** Return microseconds elapsed since the last start_timer() call. */
static uint32_t read_timer_us(J1850 *bus)
{
    return systick_elapsed_cycles(bus->time_tmp) / CYCLES_PER_US;
}

static void delay_us(uint32_t us)
{
    uint32_t snapshot = SysTick->VAL;
    uint32_t target   = us * CYCLES_PER_US;
    while (systick_elapsed_cycles(snapshot) < target) {
    }
}

/* ------------------------------------------------------------------ */
/*  GPIO helpers                                                       */
/* ------------------------------------------------------------------ */

static void bus_active(J1850 *bus)
{
    HAL_GPIO_WritePin(bus->out_port, bus->out_pin, GPIO_PIN_SET);
}

static void bus_passive(J1850 *bus)
{
    HAL_GPIO_WritePin(bus->out_port, bus->out_pin, GPIO_PIN_RESET);
}

static bool bus_is_active(J1850 *bus)
{
    return (HAL_GPIO_ReadPin(bus->in_port, bus->in_pin) == GPIO_PIN_SET);
}

/* ------------------------------------------------------------------ */
/*  CRC                                                                */
/* ------------------------------------------------------------------ */

uint8_t J1850_crc(uint8_t *msg_buf, int nbytes)
{
    uint8_t crc_val = 0xFFu;
    while (nbytes--) {
        crc_val ^= *msg_buf++;
        for (int i = 0; i < 8; i++)
            crc_val = (crc_val & 0x80u) ? (uint8_t)((crc_val << 1u) ^ 0x1Du)
                                        : (uint8_t)(crc_val << 1u);
    }
    crc_val ^= 0xFFu;
    return crc_val;
}

/* ------------------------------------------------------------------ */
/*  Monitor / serial output                                            */
/* ------------------------------------------------------------------ */

/**
 * Send a received J1850 frame to the L4 over UART using the shared
 * STX/ETX framing.
 *
 * Format: [ STX | LEN | CMD_RECV | data bytes... | ETX ]
 *   LEN = 1 (CMD byte) + nbytes
 *   CMD = CMD_RECV (0x10) — tells the L4 this is an inbound J1850 frame
 *
 * The L4's CalCmd_ParseRxBuffer expects this exact layout.
 */
static void send_to_serial(J1850 *bus, int nbytes, uint8_t *buf)
{
    if (!bus->serial || nbytes <= 0) return;

    uint8_t header[3] = { S_STX, (uint8_t)(1u + nbytes), 0x10 };
    Serial_write(bus->serial, header, 3u);
    Serial_write(bus->serial, buf, (size_t)nbytes);
    uint8_t etx = S_ETX;
    Serial_write(bus->serial, &etx, 1u);
}

static void monitor(J1850 *bus)
{
    /* TX success — send ACK frame so host knows transmission completed */
    if (bus->message == J1850_MSG_SEND_OK);
    	//do nothing

    /* RX success — forward frame to host only if rxEnabled */
    if (bus->message == J1850_MSG_ACCEPT_OK)
        if (bus->rxEnabled)
            send_to_serial(bus, bus->rx_nbyte, bus->rx_msg_buf);
}

/* ------------------------------------------------------------------ */
/*  Low-level receive                                                  */
/* ------------------------------------------------------------------ */

static bool recv_msg(J1850 *bus, uint8_t *msg_buf)
{
    int     nbits, nbytes;
    bool    bit_state;

    bus->rx_msg_buf = msg_buf;

    /* Wait for bus to go active (SOF start) */
    start_timer(bus);
    while (!bus_is_active(bus)) {
        if (read_timer_us(bus) > J1850_WAIT_100US) {
            bus->message = J1850_ERR_NO_RESPONSE_100US;
            return false;
        }
    }

    /* Measure SOF pulse width */
    start_timer(bus);
    while (bus_is_active(bus)) {
        if (read_timer_us(bus) > J1850_RX_SOF_MAX) {
            bus->message = J1850_ERR_SOF_TIMEOUT;
            return false;
        }
    }

    if (read_timer_us(bus) < J1850_RX_SOF_MIN) {
        bus->message = J1850_ERR_NOT_SOF;
        return false;
    }

    bit_state = bus_is_active(bus);
    start_timer(bus);

    for (nbytes = 0; nbytes < J1850_MAX_MSG_SIZE; ++nbytes) {
        nbits = 8;
        do {
            *msg_buf <<= 1;

            /* Wait for edge or EOD */
            while (bus_is_active(bus) == bit_state) {
                if (read_timer_us(bus) > J1850_RX_EOD_MIN) {
                    bus->rx_nbyte = nbytes;
                    bus->message  = J1850_MSG_ACCEPT_OK;
                    return true;
                }
            }

            bit_state = bus_is_active(bus);
            uint32_t pulse_width = read_timer_us(bus);
            start_timer(bus);

            if (pulse_width < J1850_RX_SHORT_MIN) {
                bus->message = J1850_ERR_NOT_SHORT;
                return false;
            }

            /* Short passive pulse = logic 1 */
            if ((pulse_width < J1850_RX_SHORT_MAX) && !bus_is_active(bus))
                *msg_buf |= 1u;

            /* Long active pulse = logic 1 */
            if ((pulse_width > J1850_RX_LONG_MIN) &&
                (pulse_width < J1850_RX_LONG_MAX) &&
                bus_is_active(bus))
                *msg_buf |= 1u;

        } while (--nbits);
        ++msg_buf;
    }

    bus->rx_nbyte = nbytes;
    bus->message  = J1850_MSG_ACCEPT_OK;
    return true;
}

/* ------------------------------------------------------------------ */
/*  Low-level transmit                                                 */
/* ------------------------------------------------------------------ */

static bool send_msg(J1850 *bus, uint8_t *msg_buf, int nbytes)
{
    int     nbits;
    uint8_t temp_byte;

    bus->tx_msg_buf = msg_buf;
    bus->tx_nbyte   = nbytes;

    if (nbytes > J1850_MAX_MSG_SIZE) {
        bus->message = J1850_ERR_MSG_TOO_LONG;
        return false;
    }

    /* Wait for Inter-Frame Separation (IFS) */
    start_timer(bus);
    while (read_timer_us(bus) < J1850_RX_IFS_MIN) {
        if (bus_is_active(bus))
            start_timer(bus);   /* bus went active — reset IFS wait */
    }

    /* Start Of Frame */
    bus_active(bus);
    delay_us(J1850_TX_SOF);

    do {
        temp_byte = *msg_buf;
        nbits = 8;
        while (nbits--) {
            if (nbits & 1u) {
                bus_passive(bus);
                delay_us((temp_byte & 0x80u) ? J1850_TX_LONG : J1850_TX_SHORT);
            } else {
                bus_active(bus);
                delay_us((temp_byte & 0x80u) ? J1850_TX_SHORT : J1850_TX_LONG);
            }
            temp_byte <<= 1u;
        }
        ++msg_buf;
    } while (--nbytes);

    /* End Of Frame */
    bus_passive(bus);
    delay_us(J1850_TX_EOF);

    bus->message = J1850_MSG_SEND_OK;
    return true;
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

void J1850_init(J1850       *bus,
                GPIO_TypeDef *in_port,  uint16_t in_pin,
                GPIO_TypeDef *out_port, uint16_t out_pin,
                SerialPort   *serial)
{
    memset(bus, 0, sizeof(J1850));
    bus->in_port  = in_port;
    bus->in_pin   = in_pin;
    bus->out_port = out_port;
    bus->out_pin  = out_pin;
    bus->serial   = serial;
    bus->rxEnabled = 0;   /* RX forwarding disabled by default */

    bus_passive(bus);   /* ensure bus starts in passive (recessive) state */
}

bool J1850_accept(J1850 *bus, uint8_t *msg_buf, bool check_crc)
{
    bool ok = recv_msg(bus, msg_buf);

    if (check_crc && ok) {
        if (msg_buf[bus->rx_nbyte - 1] != J1850_crc(msg_buf, bus->rx_nbyte - 1)) {
            ok = false;
            bus->message = J1850_ERR_CRC;
        }
    }

    monitor(bus);
    return ok;
}

bool J1850_send(J1850 *bus, uint8_t *msg_buf, int nbytes)
{
    if (nbytes <= 0 || nbytes >= J1850_MAX_MSG_SIZE) {
        bus->message = J1850_ERR_MSG_TOO_LONG;
        return false;
    }

    /* Copy into internal buffer so we can append CRC without
     * modifying the caller's array */
    memcpy(bus->tx_buf, msg_buf, (size_t)nbytes);
    bus->tx_buf[nbytes] = J1850_crc(bus->tx_buf, nbytes);
    nbytes++;

    bool ok = send_msg(bus, bus->tx_buf, nbytes);
    monitor(bus);
    return ok;
}

void J1850_setRxEnabled(J1850 *bus, int enabled)
{
    bus->rxEnabled = enabled;
}
