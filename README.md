# UNCC Rocketry Payload, Fall 2026

Custom avionics board for the UNC Charlotte rocketry club's transonic payload. It logs flight data, tracks position, sends live telemetry and pad status to a ground station, and controls an onboard camera.

## Layout

The schematic is split into hierarchical sheets, all tied together on the root sheet:

| Sheet | Contents |
|---|---|
| POWER | Battery input, 5V buck and 3.3V regulator |
| MCU | Microcontroller, clock, reset/boot, debug |
| SENSORS | Inertial, pressure and environmental sensors |
| RF | GPS |
| FLASH | Onboard flight data storage |
| CAM | Camera interface |
| CONNECTORS | Battery, arming switch, debug header, camera and USB-C |

## Tools

Designed in KiCad 10. Open `UNCC_ROCKETRY_PAYLOAD_FALL_2026.kicad_pro` to get started.

## Status

Schematic in progress. Firmware is being developed on breadboarded breakout boards in parallel. PCB layout has not started.
