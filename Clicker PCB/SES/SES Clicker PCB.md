
# MCU Pinout

| Pin      | Type | Protocol | Function                                  |
| -------- | ---- | -------- | ----------------------------------------- |
| A0       | I    | -        | DW1000 TXE signal                         |
| A0       | O    | -        | DW1000 SPI polarity setting (setup phase) |
| A1       | I    | -        | DW1000 RXE signal                         |
| A1       | O    | -        | DW1000 SPI phase setting (setup phase)    |
| A2       | IO   | SWD      | SWDIO                                     |
| A3       | IO   | SWD      | SWCLK                                     |
| A4       | O    | -        | DW1000 NRST                               |
| A5       | O    | -        | DW1000 WKUP                               |
| A6       | O    | -        | DW1000 Force on                           |
| A8       | IO   | UART     | RX docked timestamp update                |
| A9       | O    | -        | Status LED                                |
| A10      | I    | BOOT     | BOOT0                                     |
| A12      | O    | SPI      | SPI NSS                                   |
| A13      | O    | SPI      | SPI CLK                                   |
| A14      | I    | SPI      | SPI MISO                                  |
| A15      | O    | SPI      | SPI MOSI                                  |
| B0       | I    | -        | Clicker (IRQ)                             |
| B1       | I    | -        | UWB (IRQ)                                 |
| B2       | I    | -        | Battery (IRQ)                             |
| B3       | I    | ADC      | Battery (voltage)                         |
| B4       | I    | -        | UWB sync                                  |
| B8 - B11 | O    | -        | Battery status (4-bit)                    |

# PCB Pinout
## SW1
| Pin | Net     |
| --- | ------- |
| 1   | Clicker |
| 2   | GND     |
## SW2
| Pin | Net         |
| --- | ----------- |
| 1   | Battery_IRQ |
| 2   | GND         |
## J1
| Pin | Net   |
| --- | ----- |
| 1   | VIN   |
| 2   | RX    |
| 3   | SWDIO |
| 4   | SWCLK |
| 5   | GND   |
## J2
| Pin | Net  |
| --- | ---- |
| 1   | BAT+ |
| 2   | GND  |

# PCB Dimensions
Zero Point: TOP-LEFT (outside rounded corner)
## Edge cuts

| Dimension     | Measurement (mm) |
| ------------- | ---------------- |
| Width         | 30               |
| Height        | 85.5             |
| Corner Radius | 0.75             |

## Mounting
| Dimension | Measurement (mm) |
| --------- | ---------------- |
| Width     | 22               |
| Height    | 60               |

## LEDs

| LED                        | x (mm)  | y (mm) |
| -------------------------- | ------- | ------ |
| PWR                        | 2       | 31.15  |
| Status                     | 2       | 32.7   |
| Battery_status[3]          | 2       | 34.25  |
| Battery_status[2]          | 2       | 35.8   |
| Battery_status[1]          | 2       | 37.35  |
| Battery_status[0]          | 2       | 38.9   |
| UWB RX_OK                  | 28.725  | 61.825 |
| UWB SFD                    | 27.175  | 61.825 |
| UWB RX                     | 25.625  | 61.825 |
| UWB TX                     | 24.075  | 61.825 |
| Battery charger PG         | 19.4375 | 26.225 |
| Battery charger STAT 1     | 20.9875 | 26.225 |
| Battery charger STAT 2<br> | 22.5375 | 26.225 |
