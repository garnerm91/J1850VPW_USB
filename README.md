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

## Credit
A significant amount of the code is based on redheadedrod's J1850VPW firmware: https://github.com/redheadedrod/j1850/tree/master/M2_J1850_VPW
