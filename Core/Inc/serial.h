/**
 * @file serial.h
 * @brief Arduino-style UART driver for STM32 CubeIDE  (pure C, HAL-based)
 *
 * Supports: Serial_attach(), Serial_available(), Serial_read(), Serial_write(),
 *           Serial_print(), Serial_println(), Serial_flush(), Serial_end()
 *
 * Design
 * ------
 * CubeMX / .ioc owns ALL hardware initialisation (pins, clocks, baud rate, NVIC).
 * This driver sits on top.  Call Serial_attach() once after MX_USARTx_UART_Init()
 * to reset the ring buffers and arm the first RX interrupt.  Nothing else is touched.
 *
 * Quick-start
 * -----------
 *  1. Adjust the HAL include below to match your MCU family.
 *  2. Configure USART2 (or any UART) in CubeMX, enable its global interrupt, regenerate.
 *  3. In a shared globals file declare one SerialPort per UART you need:
 *
 *       #include "serial.h"
 *       extern UART_HandleTypeDef huart2;   // CubeMX-generated
 *
 *  4. In main.c, after MX_USART2_UART_Init(), inside USER CODE BEGIN 2:
 *
 *       Serial_attach(&Serial2, &huart2, USART2_IRQn);
 *
 *  5. In stm32xxxx_it.c, inside USART2_IRQHandler(), USER CODE BEGIN block:
 *
 *       extern SerialPort Serial2;
 *       HAL_UART_IRQHandler(Serial2.huart);
 *
 *  6. In a new file serial_callbacks.c, override the three HAL weak callbacks:
 *
 *       extern SerialPort Serial2;
 *       void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h) {
 *           if (h->Instance == USART2) Serial_rxCallback(&Serial2);
 *       }
 *       void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h) {
 *           if (h->Instance == USART2) Serial_txCallback(&Serial2);
 *       }
 *       void HAL_UART_ErrorCallback(UART_HandleTypeDef *h) {
 *           if (h->Instance == USART2) Serial_errorCallback(&Serial2);
 *       }
 *
 *  7. Use anywhere:
 *       Serial_println(&Serial2, "Hello STM32!");
 *       if (Serial_available(&Serial2)) { int c = Serial_read(&Serial2); }
 */

#ifndef SERIAL_H
#define SERIAL_H

/* ---------- Adjust this include for your MCU family --------------- */
#include "stm32c0xx_hal.h"  /* e.g. "stm32f4xx_hal.h"            */
/* ------------------------------------------------------------------ */

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  Buffer sizes — override with a project-level #define if needed    */
/* ------------------------------------------------------------------ */
#ifndef SERIAL_RX_BUFFER_SIZE
#define SERIAL_RX_BUFFER_SIZE   256u
#endif
#ifndef SERIAL_TX_BUFFER_SIZE
#define SERIAL_TX_BUFFER_SIZE   256u
#endif

/* ------------------------------------------------------------------ */
/*  Serial mode constants (same encoding as Arduino)                  */
/* ------------------------------------------------------------------ */
#define SERIAL_8N1   0x06u
#define SERIAL_8N2   0x0Eu
#define SERIAL_8E1   0x26u
#define SERIAL_8O1   0x36u
#define SERIAL_8E2   0x2Eu
#define SERIAL_8O2   0x3Eu
#define SERIAL_7E1   0x24u
#define SERIAL_7O1   0x34u
#define SERIAL_7E2   0x2Cu
#define SERIAL_7O2   0x3Cu

/* ------------------------------------------------------------------ */
/*  Driver state struct  — declare one per UART                       */
/*  Zero-initialise at startup:  SerialPort Serial2 = {0};            */
/* ------------------------------------------------------------------ */
typedef struct {
    UART_HandleTypeDef *huart;      /**< pointer to CubeMX HAL handle        */
    IRQn_Type           irq;        /**< IRQ number, e.g. USART2_IRQn        */

    uint8_t  recvByte;              /**< single-byte staging for HAL_IT      */

    /* RX ring buffer */
    volatile uint16_t rxHead;
    volatile uint16_t rxTail;
    uint8_t  rxBuf[SERIAL_RX_BUFFER_SIZE];

    /* TX ring buffer */
    volatile uint16_t txHead;
    volatile uint16_t txTail;
    uint8_t  txBuf[SERIAL_TX_BUFFER_SIZE];

    uint8_t  initialised;
} SerialPort;

/* ------------------------------------------------------------------ */
/*  Lifecycle                                                          */
/* ------------------------------------------------------------------ */

/**
 * Serial_attach  — PREFERRED for CubeIDE projects.
 * Call once after MX_USARTx_UART_Init().  Resets buffers, arms first
 * RX interrupt.  Does NOT touch baud rate, pins, clocks, or NVIC.
 *
 * @param port   pointer to a SerialPort instance (zero-init before first use)
 * @param huart  pointer to the CubeMX-generated HAL handle, e.g. &huart2
 * @param irq    IRQ number for this peripheral,              e.g. USART2_IRQn
 */
void Serial_attach(SerialPort *port, UART_HandleTypeDef *huart, IRQn_Type irq);

/**
 * Serial_end  — abort transfers and stop the driver.
 * Does NOT de-init the peripheral (CubeMX owns that).
 */
void Serial_end(SerialPort *port);

/* ------------------------------------------------------------------ */
/*  Core API                                                           */
/* ------------------------------------------------------------------ */

/** Returns number of bytes waiting in the RX buffer. */
int  Serial_available(SerialPort *port);

/** Returns number of bytes that can be written without blocking. */
int  Serial_availableForWrite(SerialPort *port);

/** Read one byte from the RX buffer. Returns -1 if empty. */
int  Serial_read(SerialPort *port);

/** Peek at the next byte without consuming it. Returns -1 if empty. */
int  Serial_peek(SerialPort *port);

/** Enqueue one byte for transmission. Blocks if TX buffer is full. */
size_t Serial_writeByte(SerialPort *port, uint8_t byte);

/** Enqueue a raw byte buffer for transmission. */
size_t Serial_write(SerialPort *port, const uint8_t *buf, size_t len);

/** Enqueue a null-terminated string for transmission. */
size_t Serial_writeStr(SerialPort *port, const char *str);

/** Block until the TX buffer is fully drained. */
void Serial_flush(SerialPort *port);

/* ------------------------------------------------------------------ */
/*  Print helpers                                                      */
/* ------------------------------------------------------------------ */

/** Print a null-terminated string (no line ending). */
size_t Serial_print(SerialPort *port, const char *str);

/** Print a signed integer in the given base (2, 8, 10, 16). */
size_t Serial_printInt(SerialPort *port, int32_t val, uint8_t base);

/** Print an unsigned integer in the given base. */
size_t Serial_printUInt(SerialPort *port, uint32_t val, uint8_t base);

/** Print a double with the given number of decimal places. */
size_t Serial_printFloat(SerialPort *port, double val, uint8_t decimals);

/** Print a string followed by CR+LF. */
size_t Serial_println(SerialPort *port, const char *str);

/** Print a signed integer followed by CR+LF. */
size_t Serial_printlnInt(SerialPort *port, int32_t val, uint8_t base);

/** Print an unsigned integer followed by CR+LF. */
size_t Serial_printlnUInt(SerialPort *port, uint32_t val, uint8_t base);

/** Print a double followed by CR+LF. */
size_t Serial_printlnFloat(SerialPort *port, double val, uint8_t decimals);

/* ------------------------------------------------------------------ */
/*  HAL interrupt callbacks — call these from your callback overrides */
/* ------------------------------------------------------------------ */
void Serial_rxCallback(SerialPort *port);
void Serial_txCallback(SerialPort *port);
void Serial_errorCallback(SerialPort *port);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_H */
