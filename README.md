# J1850VPW_USB
STM32C0 firmware that acts as a serial-to-J1850 VPW bridge.
Built with STM32 CubeIDE, pure C, HAL-based. Based on Redheadedrod's

## Hardware
- STM32C0 @ 24 MHz
- USART1 @ 115200 baud (TX/RX)
- PB0 - LED
- PB1 - Power Enable for 7v
- PB3 - J1850 bus TX
- PB4 - J1850 bus RX

## Serial Protocol
Frames use STX(0x02)/ETX(0x03) framing: `[ 0x02 | LEN | CMD | data... | 0x03 ]`
LEN includes the CMD byte.
The device will respond with an ACK = `0x06` or NACK = `0x15`.

| CMD  | Description | Data |
|------|-------------|------|
| 0x01 | RX Mode     | `0x01` = enable, `0x00` = disable |
| 0x02 | Send frame  | J1850 bytes (CRC appended automatically) |

This is all in hex not ascii

## Examples for external device interfacing with this
Example of how to put in and out of RX mode
```c
 uint8_t BuildRxModeFrame(uint8_t *dst, uint8_t enable)
{
    dst[0] = S_STX;
    dst[1] = 0x02u;          /* LEN: CMD(1) + data(1) */
    dst[2] = CMD_RX_MODE;
    dst[3] = enable ? 0x01u : 0x00u;
    dst[4] = S_ETX;
    return 5u;
}
```
Example of a Parser for a received frame
```c
void ParseRxBuffer(uint8_t *buf, uint16_t len)
{
    uint16_t i = 0;

    while (i + 3u < len)   /* need at least STX+LEN+CMD+ETX */
    {
        /* Scan for STX */
        if (buf[i] != S_STX) { i++; continue; }

        uint8_t  frameLen  = buf[i + 1u];          /* LEN field            */
        uint16_t etx_pos   = i + 2u + frameLen;    /* expected ETX offset  */

        /* Bounds check */
        if (etx_pos >= len) return;                 /* incomplete, wait     */

        /* ETX check */
        if (buf[etx_pos] != S_ETX) { i++; continue; }

        /* LEN=0 is invalid (C0 rejects it too) */
        if (frameLen == 0u) { i++; continue; }

        /* Extract CMD and data */
        uint8_t  cmd      = buf[i + 2u];
        uint8_t *data     = &buf[i + 3u];
        uint8_t  data_len = frameLen - 1u;          /* LEN minus CMD byte   */

        HandleFrame(cmd, data, data_len);           /* Do something useful with the frame   */

        i = etx_pos + 1u;   /* advance past the ETX */
    }
}
```
Example of how to transmit
```c
uint8_t CalCmd_BuildSendFrame(uint8_t *dst, const uint8_t *j1850_data, uint8_t data_len)
{
    uint8_t i = 0;
    dst[i++] = S_STX;
    dst[i++] = 1u + data_len;   /* LEN: CMD(1) + data */
    dst[i++] = CMD_SEND;
    for (uint8_t d = 0; d < data_len; d++) dst[i++] = j1850_data[d];
    dst[i++] = S_ETX;
    return i;
}

```

## Credit
A significant amount of the code is based on redheadedrod's J1850VPW firmware: https://github.com/redheadedrod/j1850/tree/master/M2_J1850_VPW
