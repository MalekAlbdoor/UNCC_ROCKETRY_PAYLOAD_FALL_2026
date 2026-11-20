# Hardware notes

What is on the Fall 2026 payload board, how it is wired, and where the connectors go. The full schematic is in [Schematic.pdf](../Schematic.pdf). If this page and the schematic ever disagree, the schematic wins.

## Parts

| Job | Part | Talks to the MCU over | Notes |
|---|---|---|---|
| Microcontroller | STM32F405RGT6 | | 168 MHz Cortex-M4, 8 MHz crystal, native USB |
| IMU | LSM6DSO32 | SPI1 | 3-axis accel (up to 32 g) and gyro (up to 2000 dps) |
| High-g accelerometer | H3LIS331DL | SPI1 | up to 400 g, for the motor burn and landing |
| Barometer | MS5611-01BA03 | SPI1 | pressure and temperature, used for altitude |
| Magnetometer | MMC5983MA | I2C1 | 3-axis, 8 gauss range |
| Humidity / temperature | SHT45 | I2C1 | version with the PTFE membrane |
| GPS | u-blox SAM-M10Q | USART2 | antenna is built into the module |
| Telemetry radio | EBYTE E22-900M30S | SPI2 | SX1262 LoRa, 915 MHz, 1 W |
| Flight log | W25Q512JV | SPI3 | 512 Mbit (64 MB) NOR flash |
| USB protection | USBLC6-2SC6 | | ESD on the USB-C data lines |

The three SPI1 sensors each have their own chip select, and the IMU and the accelerometer also have an interrupt line back to the MCU.

## MCU pin map

| Signal | Pins |
|---|---|
| SPI1 (IMU, accel, baro) | SCK PA5, MISO PA6, MOSI PA7. CS: IMU PA4, accel PC4, baro PC5. INT: IMU PB0, accel PB1 |
| SPI2 (radio) | SCK PB13, MISO PB14, MOSI PB15, CS PB12. BUSY PC8, DIO1 PC9, NRST PA8, RXEN PC0 |
| SPI3 (flash) | SCK PC10, MISO PC11, MOSI PC12, CS PA15 |
| I2C1 | SCL PB6, SDA PB7 (4.7k pull-ups) |
| GPS | USART2 on PA2/PA3. Reset PB4, timepulse PB9 |
| Debug UART | USART3 on PB10/PB11 |
| Camera 1 / 2 | USART6 on PC6/PC7, USART1 on PA9/PA10 |
| USB | PA11 (D-), PA12 (D+) |
| Misc | PC13 camera power enable, PB5 camera select, PB8 buzzer, PA1 battery voltage, PA0 VTX SmartAudio |

## Power

```
battery -> J2 -> fuse (4 A) -> arming switch (J3) -> P-FET (reverse polarity) -> +BATT
+BATT -> TPS54302 buck -> 5 V -> AP2112K LDO -> 3.3 V
```

- The 5 V rail runs the radio, the buzzer, the cameras and the 3.3 V LDO. 3.3 V runs the MCU, sensors, flash and (through a ferrite bead) the GPS.
- USB-C VBUS feeds the 5 V rail through a diode, so the board can be powered from a laptop on the bench.
- Cameras are switched with a TPS22917 load switch, so the MCU can turn them off.
- A 15 V TVS sits on the battery input. The buck itself takes 4.5 to 28 V, but the TVS and the battery sense divider (100k / 33k, full scale about 13.3 V) make a 3S LiPo the practical maximum.
- The VTX connector is fed straight from the battery after the arming switch.

Test points: TP1 battery, TP2 GND, TP3 after the P-FET, TP4 5 V, TP5 3.3 V, TP6 GPS safeboot.

## Connectors

Pin numbers below are the schematic pin numbers.

| Ref | Label | Type | Pins |
|---|---|---|---|
| J1 | SWD | 2x7 1.27 mm header (STDC14) | SWD, SWO, reset and a UART, for an ST-LINK |
| J2 | BATT IN | JST XH, 2 pin | 1 GND, 2 battery + (goes through the fuse) |
| J3 | ARM SW | JST XH, 2 pin | arming switch, in series with the battery |
| J4 | CAM 1 | JST PH, 4 pin | 1 5V, 2 TX, 3 RX, 4 GND |
| J5 | USB-C | USB-C receptacle | USB 2.0 device |
| J6 | VTX | JST XH, 3 pin | 1 battery +, 2 SmartAudio, 3 GND |
| J7 | CAM SW | JST PH, 3 pin | 1 5V, 2 select, 3 GND |
| J8 | CAM 2 | JST PH, 4 pin | 1 5V, 2 TX, 3 RX, 4 GND |
| J9 | GPIO | screw terminal, 4 pos | 1 PC3, 2 PC1, 3 PC15, 4 PC14 |
| J12 | I2C | screw terminal, 4 pos | 1 3V3, 2 SDA, 3 SCL, 4 GND (same bus as the magnetometer and SHT45) |
| J13 | UART DEBUG | screw terminal, 4 pos | 1 3V3, 2 TX, 3 RX, 4 GND, 115200 8N1 |

SW1 is the BOOT0 button, for the USB bootloader. The green LED is a status light and the red one shows power.

## Ordering

The `fabrication/` folder has what JLCPCB needs: the gerber and drill zip, a BOM and a placement file for assembly. It also has `bom_full.csv`, which lists every part with manufacturer part number and LCSC number.

- 4 layers, 1.6 mm, standard JLC04161H-7628 stackup.
- The sensors and the GPS are LGA parts that only JLC's Standard assembly tier can place. Connectors, USB-C and J1 are not in the JLC BOM and get soldered by hand.
- Part numbers live in the schematic symbol fields (`MPN`, `LCSC`), so the BOM comes straight out of KiCad. Change them there, not in the CSVs.

## Firmware

PlatformIO projects are in `firmware/`: `UNCC_PAYLOAD_FIRMWARE_PLATFORMIO` for this board and `UNCC_BASESTATION_PLATFORMIO` for the ground station.
