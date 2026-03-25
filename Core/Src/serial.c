/**
 * @file serial.c
 * @brief Arduino-style UART driver for STM32 CubeIDE  (pure C, HAL-based)
 */
// Mostly AI
#include "serial.h"
#include <string.h>
#include <stdio.h>

/* ------------------------------------------------------------------ */
/*  Circular buffer helpers                                            */
/* ------------------------------------------------------------------ */
#define BUF_NEXT(idx, size)    (((idx) + 1u) % (size))
#define BUF_FULL(h, t, size)   (BUF_NEXT((h), (size)) == (t))
#define BUF_EMPTY(h, t)        ((h) == (t))
#define BUF_COUNT(h, t, size)  (((h) >= (t)) ? ((h) - (t)) : ((size) - (t) + (h)))

/* ------------------------------------------------------------------ */
/*  Internal helpers (forward declarations)                           */
/* ------------------------------------------------------------------ */
static int      _txBusy(SerialPort *port);
static void     _startTx(SerialPort *port);

/* ------------------------------------------------------------------ */
/*  Lifecycle                                                          */
/* ------------------------------------------------------------------ */

void Serial_attach(SerialPort *port, UART_HandleTypeDef *huart, IRQn_Type irq)
{
    port->huart       = huart;
    port->irq         = irq;
    port->rxHead      = 0;
    port->rxTail      = 0;
    port->txHead      = 0;
    port->txTail      = 0;
    port->recvByte    = 0;
    port->initialised = 1;

    /* Arm the first single-byte RX — the callback re-arms automatically */
    HAL_UART_Receive_IT(port->huart, &port->recvByte, 1u);
}

void Serial_end(SerialPort *port)
{
    if (!port->initialised) return;
    Serial_flush(port);
    HAL_NVIC_DisableIRQ(port->irq);
    HAL_UART_AbortReceive(port->huart);
    HAL_UART_AbortTransmit(port->huart);
    /* We do NOT call HAL_UART_DeInit — CubeMX owns the peripheral lifecycle */
    port->rxHead      = 0;
    port->rxTail      = 0;
    port->txHead      = 0;
    port->txTail      = 0;
    port->initialised = 0;
}

/* ------------------------------------------------------------------ */
/*  Core API                                                           */
/* ------------------------------------------------------------------ */

int Serial_available(SerialPort *port)
{
    return (int)BUF_COUNT(port->rxHead, port->rxTail, SERIAL_RX_BUFFER_SIZE);
}

int Serial_availableForWrite(SerialPort *port)
{
    return (int)(SERIAL_TX_BUFFER_SIZE - 1u
                 - BUF_COUNT(port->txHead, port->txTail, SERIAL_TX_BUFFER_SIZE));
}

int Serial_peek(SerialPort *port)
{
    if (BUF_EMPTY(port->rxHead, port->rxTail)) return -1;
    return (int)port->rxBuf[port->rxTail];
}

int Serial_read(SerialPort *port)
{
    if (BUF_EMPTY(port->rxHead, port->rxTail)) return -1;
    uint8_t byte = port->rxBuf[port->rxTail];
    port->rxTail = BUF_NEXT(port->rxTail, SERIAL_RX_BUFFER_SIZE);
    return (int)byte;
}

size_t Serial_writeByte(SerialPort *port, uint8_t byte)
{
    /* Spin-wait if buffer is full, unless interrupts are disabled */
    while (BUF_FULL(port->txHead, port->txTail, SERIAL_TX_BUFFER_SIZE)) {
        if (__get_PRIMASK() & 0x1u) return 0u;   /* IRQs off — cannot drain, bail */
    }

    port->txBuf[port->txHead] = byte;
    port->txHead = BUF_NEXT(port->txHead, SERIAL_TX_BUFFER_SIZE);

    if (!_txBusy(port)) _startTx(port);
    return 1u;
}

size_t Serial_write(SerialPort *port, const uint8_t *buf, size_t len)
{
    size_t written = 0u;
    for (size_t i = 0u; i < len; i++) written += Serial_writeByte(port, buf[i]);
    return written;
}

size_t Serial_writeStr(SerialPort *port, const char *str)
{
    if (!str) return 0u;
    size_t written = 0u;
    while (*str) written += Serial_writeByte(port, (uint8_t)(*str++));
    return written;
}

void Serial_flush(SerialPort *port)
{
    while (!BUF_EMPTY(port->txHead, port->txTail)) {
        if (__get_PRIMASK() & 0x1u) return;   /* can't wait with IRQs off */
    }
}

/* ------------------------------------------------------------------ */
/*  Print helpers                                                      */
/* ------------------------------------------------------------------ */

size_t Serial_print(SerialPort *port, const char *str)
{
    return Serial_writeStr(port, str);
}

size_t Serial_printInt(SerialPort *port, int32_t val, uint8_t base)
{
    char buf[34];
    if (base == 10u) {
        snprintf(buf, sizeof(buf), "%ld", (long)val);
    } else {
        /* Handle non-decimal bases using the unsigned path */
        return Serial_printUInt(port, (uint32_t)val, base);
    }
    return Serial_writeStr(port, buf);
}

size_t Serial_printUInt(SerialPort *port, uint32_t val, uint8_t base)
{
    char    buf[34];
    uint8_t digits[34];
    uint8_t n = 0u;

    if (base < 2u) base = 10u;

    /* Build digit string in reverse */
    do {
        digits[n++] = (uint8_t)(val % base);
        val /= base;
    } while (val && n < 33u);

    /* Reverse into buf */
    uint8_t j = 0u;
    for (int8_t k = (int8_t)(n - 1); k >= 0; k--)
        buf[j++] = (char)(digits[k] < 10u ? '0' + digits[k] : 'A' + digits[k] - 10u);
    buf[j] = '\0';

    return Serial_writeStr(port, buf);
}

size_t Serial_printFloat(SerialPort *port, double val, uint8_t decimals)
{
    char fmt[12], buf[32];
    snprintf(fmt, sizeof(fmt), "%%.%uf", (unsigned)decimals);
    snprintf(buf, sizeof(buf), fmt, val);
    return Serial_writeStr(port, buf);
}

size_t Serial_println(SerialPort *port, const char *str)
{
    return Serial_print(port, str) + Serial_writeStr(port, "\r\n");
}

size_t Serial_printlnInt(SerialPort *port, int32_t val, uint8_t base)
{
    return Serial_printInt(port, val, base) + Serial_writeStr(port, "\r\n");
}

size_t Serial_printlnUInt(SerialPort *port, uint32_t val, uint8_t base)
{
    return Serial_printUInt(port, val, base) + Serial_writeStr(port, "\r\n");
}

size_t Serial_printlnFloat(SerialPort *port, double val, uint8_t decimals)
{
    return Serial_printFloat(port, val, decimals) + Serial_writeStr(port, "\r\n");
}

/* ------------------------------------------------------------------ */
/*  HAL interrupt callbacks                                            */
/* ------------------------------------------------------------------ */

void Serial_rxCallback(SerialPort *port)
{
    /* Push received byte into RX ring buffer (silently drop if full) */
    if (!BUF_FULL(port->rxHead, port->rxTail, SERIAL_RX_BUFFER_SIZE)) {
        port->rxBuf[port->rxHead] = port->recvByte;
        port->rxHead = BUF_NEXT(port->rxHead, SERIAL_RX_BUFFER_SIZE);
    }
    /* Re-arm for the next byte */
    HAL_UART_Receive_IT(port->huart, &port->recvByte, 1u);
}

void Serial_txCallback(SerialPort *port)
{
    /* Advance tail past the byte that just finished transmitting */
    port->txTail = BUF_NEXT(port->txTail, SERIAL_TX_BUFFER_SIZE);
    /* If more bytes are waiting, send the next one */
    if (!BUF_EMPTY(port->txHead, port->txTail)) {
        HAL_UART_Transmit_IT(port->huart, &port->txBuf[port->txTail], 1u);
    }
}

void Serial_errorCallback(SerialPort *port)
{
    /* Clear all UART error flags */
    __HAL_UART_CLEAR_PEFLAG(port->huart);
    __HAL_UART_CLEAR_FEFLAG(port->huart);
    __HAL_UART_CLEAR_NEFLAG(port->huart);
    __HAL_UART_CLEAR_OREFLAG(port->huart);
    /* Resume reception */
    HAL_UART_Receive_IT(port->huart, &port->recvByte, 1u);
}

/* ------------------------------------------------------------------ */
/*  Private helpers                                                    */
/* ------------------------------------------------------------------ */

static int _txBusy(SerialPort *port)
{
    return ((HAL_UART_GetState(port->huart) & HAL_UART_STATE_BUSY_TX)
            == HAL_UART_STATE_BUSY_TX);
}

static void _startTx(SerialPort *port)
{
    if (BUF_EMPTY(port->txHead, port->txTail)) return;
    /* Disable IRQ briefly to prevent a race between write() and the TxCplt callback */
    HAL_NVIC_DisableIRQ(port->irq);
    __DSB(); __ISB();
    if (!_txBusy(port)) {
        HAL_UART_Transmit_IT(port->huart, &port->txBuf[port->txTail], 1u);
    }
    HAL_NVIC_EnableIRQ(port->irq);
}
