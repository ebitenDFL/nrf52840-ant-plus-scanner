# ANT Scanner PCA10059 v0.1

Target: nRF52840 Dongle (PCA10059), nRF5 SDK 17.1.0, S340 6.1.1, GCC 9.3.1.

## 1. Copy project

Copy this folder to `~/Projects/ANT_Scanner`.

## 2. Add your ANT+ network key

```bash
cp ant_plus_key.h.example ant_plus_key.h
nano ant_plus_key.h
```

Replace the `#error` line with a local definition of the 8-byte ANT+ network key supplied through your authorized ANT+/Garmin developer access.
Do not commit or publish the key.

The S340 evaluation license key in `nrf_sdm.h` is separate from the ANT+ network key.

## 3. Build

```bash
make clean
make
```

Expected files include `_build/nrf52840_xxaa.hex` and `_build/nrf52840_xxaa.bin`.

## Output

When the USB CDC port is open:

`ANTSCANNER,READY`

Received packets:

`ANT,<device_number>,<device_type>,<transmission_type>,<rssi_dbm>,<payload_hex>`

Example shape:

`ANT,12345,120,1,-52,0011223344556677`

The scan uses ANT+ RF channel 57 (2457 MHz), RX-only background scanning, and wildcard device number/type/transmission type.

## Important

Do not flash the raw application HEX yet. Keep the PCA10059 factory Open DFU Bootloader intact. Once this compiles, the next step is to make a DFU package containing S340 6.1.1 plus the application and program it using the bootloader.
