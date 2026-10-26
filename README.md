# UNCC Rocketry Payload, Fall 2026

Custom avionics board for the UNC Charlotte rocketry club's transonic payload. It logs flight data, tracks position, sends live telemetry and pad status to a ground station, and controls an onboard camera.

## Layout

The schematic is split into hierarchical sheets, all tied together on the root sheet:

| Sheet | Contents |
|---|---|
| POWER | 5V buck, 3.3V regulator, power LED, bring-up jumpers and camera power switch |
| MCU | Microcontroller, clock, reset/boot, battery voltage sense, buzzer and status LED |
| SENSORS | Inertial, pressure and environmental sensors |
| RF | LoRa telemetry radio and GPS |
| FLASH | Onboard flight data storage |
| CONNECTORS | Battery input protection, arming switch, debug header, camera and USB-C |

## Tools

Designed in KiCad 10. Open `UNCC_ROCKETRY_PAYLOAD_FALL_2026.kicad_pro` to get started.

No KiCad? Download [Schematic.pdf](Schematic.pdf) to view the full schematic.

## Dashboard
Take a look at live telemetry at telemetry.malekalbdoor.com

## Status

Schematic in progress. Firmware is being developed on breadboarded breakout boards in parallel. PCB layout has not started.
