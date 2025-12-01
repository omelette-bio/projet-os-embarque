# projet-os-embarque

## Arduino_FreeRTOS Pin Mapping

This project targets an Arduino Uno (ATmega328P) and uses direct port/register access. The relevant pin-to-port mappings are:

- Door LED: `PD2` (Arduino D2, output)
- Green LED: `PD4` (Arduino D4, output)
- Ultrasound sensor echo/input: `PD6` (Arduino D6, input)
- RFID reader input: `PB0` (Arduino D8, input)
- Master device input: `PB2` (Arduino D10, input)

Note: Arduino Uno does not have `PD8`/`PD10`; if you intend to reference Arduino digital pins 8 and 10 via registers, use `PB0` and `PB2` respectively.

## Build

From `Arduino_FreeRTOS/`:

```bash
make clean
make
```

Artifacts are generated under `Arduino_FreeRTOS/Build/` (`.elf` and `.hex`).
