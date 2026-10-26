# ANT+ USB Wheel Speed Dongle — Linux / Jetson Integration Guide

## 1. Purpose

This document describes how to receive and decode wheel-speed data from the custom ANT+ USB dongle on a Linux computer, NVIDIA Jetson, Raspberry Pi, or another Linux-based embedded device.

The USB dongle performs the ANT+ radio communication internally using an nRF52840 and the Nordic S340 ANT SoftDevice.

The host computer **does not need to implement the ANT protocol**.

The host only needs to:

1. Open the USB CDC serial device.
2. Read newline-terminated ASCII messages.
3. Parse `ANT,...` records.
4. For ANT+ Bike Speed devices (`device_type = 123`), decode the wheel event time and cumulative revolution count.
5. Calculate vehicle speed using a wheel circumference of **2170 mm**.
6. Force speed to **0 km/h** if no new wheel revolution is detected for **5 seconds**.

---

## 2. USB Interface

When the dongle application is running, Linux identifies it as:

```text
VID:PID = 1915:520f
Product = ANT+ Scanner
```

Typical Linux device:

```text
/dev/ttyACM0
```

Check with:

```bash
lsusb
ls -l /dev/ttyACM*
```

Example:

```text
Bus 003 Device 024: ID 1915:520f Nordic Semiconductor ASA ANT+ Scanner
```

The exact `/dev/ttyACM*` number can change between boots or depending on other connected USB devices.

For a production application, prefer discovering the port by USB VID/PID, USB product name, or a persistent `/dev/serial/by-id/` path rather than assuming `/dev/ttyACM0`.

---

## 3. Linux Permissions

The user running the application normally needs access to the `dialout` group:

```bash
sudo usermod -aG dialout $USER
```

Log out and log back in after changing group membership.

For a temporary shell session:

```bash
newgrp dialout
```

---

## 4. Serial Configuration

The dongle uses USB CDC ACM.

A conventional host configuration is:

```text
Baud rate: 115200
Data bits: 8
Parity: none
Stop bits: 1
Flow control: none
```

Because this is USB CDC rather than a physical UART, the baud rate is essentially nominal, but configuring the port as 115200 8N1 is recommended for compatibility with normal serial libraries.

Each record is ASCII text terminated by a newline.

Always call `strip()` / remove `\r\n` before parsing.

---

## 5. Startup Message

After the application starts, the dongle sends:

```text
ANTSCANNER,READY
```

The host may use this as confirmation that the scanner application is alive.

It is not an ANT sensor packet and should not be passed to the packet decoder.

---

## 6. ANT Packet Format

Received ANT packets are sent as:

```text
ANT,<device_number>,<device_type>,<transmission_type>,<rssi_dbm>,<payload>
```

Example:

```text
ANT,37907,123,1,-58,80FFFFFFA3180300
```

Fields:

| Field | Example | Description |
|---|---:|---|
| Prefix | `ANT` | Identifies an ANT packet |
| Device number | `37907` | ANT device ID |
| Device type | `123` | ANT+ device profile/type |
| Transmission type | `1` | ANT transmission type |
| RSSI | `-58` | Received signal strength in dBm |
| Payload | `80FFFFFFA3180300` | 8-byte ANT payload as 16 hexadecimal characters |

The payload is always represented as:

```text
Byte 0 Byte 1 Byte 2 Byte 3 Byte 4 Byte 5 Byte 6 Byte 7
```

For:

```text
80FFFFFFA3180300
```

the bytes are:

```text
80 FF FF FF A3 18 03 00
```

---

# 7. Bike Speed Device

The wheel sensor currently used by this project reports:

```text
device_type = 123
```

This identifies an ANT+ **Bike Speed** sensor.

For this profile, the fields required for speed calculation are:

```text
payload[4] = wheel event time LSB
payload[5] = wheel event time MSB

payload[6] = cumulative wheel revolutions LSB
payload[7] = cumulative wheel revolutions MSB
```

Both values are unsigned 16-bit little-endian integers.

For example:

```text
Payload = 80 FF FF FF A3 18 03 00
```

Wheel event time:

```text
0x18A3 = 6307
```

Cumulative wheel revolutions:

```text
0x0003 = 3
```

The first four bytes are not required for basic wheel-speed calculation.

---

## 8. Wheel Circumference

Use:

```text
2170 mm
```

or:

```text
2.170 m
```

This corresponds to the 700×35C wheel/tire configuration used on the bike.

Define:

```text
WHEEL_CIRCUMFERENCE_M = 2.170
```

---

# 9. Speed Calculation

The wheel sensor transmits:

- cumulative wheel revolution count
- wheel-event timestamp

The timestamp increments at:

```text
1 / 1024 second
```

Speed must be calculated from the difference between two valid wheel events.

For two packets:

```text
delta_revolutions = new_revolutions - previous_revolutions
delta_time        = new_event_time - previous_event_time
```

Because both counters are 16-bit values, rollover must be handled.

Use:

```text
delta = (new_value - old_value) & 0xFFFF
```

Then:

```text
speed_m_per_s =
    delta_revolutions
    × 2.170
    × 1024
    / delta_time
```

Convert to km/h:

```text
speed_km_h = speed_m_per_s × 3.6
```

Combined:

```text
speed_km_h =
    delta_revolutions
    × 2.170
    × 1024
    × 3.6
    / delta_time
```

Only update the calculated speed when:

```text
delta_time > 0
AND
delta_revolutions > 0
```

---

# 10. Important: Repeated Packets

When the wheel stops or when no new revolution has occurred, the ANT sensor may continue transmitting packets.

These packets can contain the same:

```text
wheel_event_time
cumulative_revolutions
```

as the previous packet.

Example:

```text
ANT,37907,123,1,-58,80FFFFFFA3180300
ANT,37907,123,1,-56,00FFFFFFA3180300
ANT,37907,123,1,-55,00FFFFFFA3180300
```

The first byte changes, but the speed fields remain:

```text
A3 18 03 00
A3 18 03 00
A3 18 03 00
```

Therefore:

**Do not interpret every received ANT packet as a new wheel measurement.**

A new wheel-speed event exists only when the wheel event time and/or cumulative revolution count advances.

---

# 11. Zero-Speed Timeout

The system should return speed to zero when no new wheel revolution has occurred.

Use:

```text
SPEED_TIMEOUT_SECONDS = 5.0
```

Whenever a valid new revolution is detected:

```text
last_speed_event_host_time = current_monotonic_time
```

During normal application processing:

```text
if current_monotonic_time - last_speed_event_host_time >= 5.0:
    speed_km_h = 0.0
```

Use a **monotonic clock**, not wall-clock/date time.

Examples:

Python:

```python
time.monotonic()
```

C/C++ on Linux:

```c
clock_gettime(CLOCK_MONOTONIC, ...)
```

The timeout is based on the arrival of a **new wheel revolution**, not merely the arrival of ANT packets.

This distinction is important because the sensor can continue transmitting identical packets while stationary.

---

# 12. Complete Decoding Algorithm

For every received line:

```text
1. Remove CR/LF.
2. Ignore empty lines.
3. If the line is "ANTSCANNER,READY":
       scanner is online
       continue

4. Split the line by comma.

5. Verify:
       field[0] == "ANT"
       number of fields == 6

6. Parse:
       device_number
       device_type
       transmission_type
       RSSI
       16-character hexadecimal payload

7. Convert payload into exactly 8 bytes.

8. If device_type == 123:
       event_time = payload[4] | payload[5] << 8
       revolutions = payload[6] | payload[7] << 8

9. If previous values exist:
       dt = (event_time - previous_event_time) & 0xFFFF
       dr = (revolutions - previous_revolutions) & 0xFFFF

10. If dt > 0 and dr > 0:
       speed =
           dr * 2.170 * 1024 * 3.6 / dt

       update last_speed_event_host_time

11. Store:
       previous_event_time = event_time
       previous_revolutions = revolutions

12. Independently:
       if no new revolution for >= 5 seconds:
           speed = 0.0
```

---

# 13. Minimal Python Example

Install PySerial if required:

```bash
python3 -m pip install pyserial
```

Example implementation:

```python
import time
import serial

PORT = "/dev/ttyACM0"
BAUD = 115200

WHEEL_CIRCUMFERENCE_M = 2.170
SPEED_TIMEOUT_SECONDS = 5.0

previous_event_time = None
previous_revolutions = None

speed_kmh = 0.0
last_speed_event_time = None


def delta_u16(new, old):
    return (new - old) & 0xFFFF


ser = serial.Serial(PORT, BAUD, timeout=0.1)

while True:

    line = ser.readline().decode("ascii", errors="ignore").strip()

    now = time.monotonic()

    if line:

        if line == "ANTSCANNER,READY":
            print("ANT+ scanner ready")
            continue

        parts = line.split(",")

        if len(parts) == 6 and parts[0] == "ANT":

            try:
                device_number = int(parts[1])
                device_type = int(parts[2])
                transmission_type = int(parts[3])
                rssi_dbm = int(parts[4])

                payload_hex = parts[5]

                if len(payload_hex) != 16:
                    continue

                payload = bytes.fromhex(payload_hex)

            except ValueError:
                continue

            if device_type == 123:

                event_time = (
                    payload[4]
                    | (payload[5] << 8)
                )

                revolutions = (
                    payload[6]
                    | (payload[7] << 8)
                )

                if (
                    previous_event_time is not None
                    and previous_revolutions is not None
                ):

                    dt = delta_u16(
                        event_time,
                        previous_event_time
                    )

                    dr = delta_u16(
                        revolutions,
                        previous_revolutions
                    )

                    if dt > 0 and dr > 0:

                        speed_kmh = (
                            dr
                            * WHEEL_CIRCUMFERENCE_M
                            * 1024.0
                            * 3.6
                            / dt
                        )

                        last_speed_event_time = now

                previous_event_time = event_time
                previous_revolutions = revolutions

    # Zero-speed logic runs even if no serial packet arrives.
    if (
        last_speed_event_time is not None
        and now - last_speed_event_time >= SPEED_TIMEOUT_SECONDS
    ):
        speed_kmh = 0.0

    print(
        f"\rSpeed: {speed_kmh:6.2f} km/h",
        end="",
        flush=True,
    )
```

---

# 14. C/C++ Decoding Example

The same payload fields can be decoded in C as:

```c
#include <stdint.h>

#define WHEEL_CIRCUMFERENCE_M 2.170f

static uint16_t previous_event_time;
static uint16_t previous_revolutions;
static int previous_valid = 0;

static float speed_kmh = 0.0f;

static uint16_t delta_u16(uint16_t new_value,
                          uint16_t old_value)
{
    return (uint16_t)(new_value - old_value);
}

void process_bike_speed_payload(const uint8_t payload[8])
{
    uint16_t event_time =
        (uint16_t)payload[4] |
        ((uint16_t)payload[5] << 8);

    uint16_t revolutions =
        (uint16_t)payload[6] |
        ((uint16_t)payload[7] << 8);

    if (previous_valid)
    {
        uint16_t dt =
            delta_u16(event_time,
                      previous_event_time);

        uint16_t dr =
            delta_u16(revolutions,
                      previous_revolutions);

        if ((dt > 0U) && (dr > 0U))
        {
            speed_kmh =
                ((float)dr
                 * WHEEL_CIRCUMFERENCE_M
                 * 1024.0f
                 * 3.6f)
                / (float)dt;

            /*
             * Also record CLOCK_MONOTONIC here.
             * If no new revolution is seen for 5 seconds,
             * set speed_kmh = 0.
             */
        }
    }

    previous_event_time = event_time;
    previous_revolutions = revolutions;
    previous_valid = 1;
}
```

---

# 15. Multiple Sensors

The ANT scanner can receive multiple devices.

Do not use one global previous-event state if more than one speed sensor may be present.

Maintain state per unique ANT device.

Recommended key:

```text
(device_number, device_type, transmission_type)
```

For example:

```text
(37907, 123, 1)
```

Each device should maintain its own:

```text
previous_event_time
previous_revolutions
last_speed_event_host_time
speed_kmh
RSSI
last_packet_time
```

---

# 16. Device Presence vs Vehicle Speed

These are two different concepts.

## ANT device presence

A device is considered present when ANT packets are still being received.

This can be tracked using:

```text
last_packet_time
```

## Wheel movement

The wheel is considered moving only when its wheel-event counters advance.

This is tracked using:

```text
last_speed_event_host_time
```

Therefore it is completely valid to have:

```text
Sensor connected: YES
RSSI: -45 dBm
Speed: 0.0 km/h
```

because the stationary sensor can continue broadcasting.

---

# 17. Recommended Host State

A useful application-side structure is:

```text
ANTSpeedSensor
├── device_number
├── device_type
├── transmission_type
├── rssi_dbm
├── previous_event_time
├── previous_revolutions
├── last_packet_monotonic
├── last_speed_event_monotonic
└── speed_kmh
```

---

# 18. Input Validation

A robust parser should reject malformed data.

Recommended checks:

```text
Line starts with "ANT,"
Exactly 6 CSV fields
device_number parses as integer
device_type parses as integer
transmission_type parses as integer
RSSI parses as signed integer
payload contains exactly 16 hexadecimal characters
payload converts to exactly 8 bytes
```

Do not allow malformed serial lines to terminate the application.

---

# 19. Example Input and Expected Interpretation

Input:

```text
ANT,37907,123,1,-58,80FFFFFFA3180300
```

Interpretation:

```text
Device number      = 37907
Device type        = 123
Profile            = ANT+ Bike Speed
Transmission type  = 1
RSSI               = -58 dBm

Payload:
80 FF FF FF A3 18 03 00

Wheel event time   = 0x18A3 = 6307
Wheel revolutions  = 0x0003 = 3
```

A single packet is not enough to determine speed.

A later packet with an advanced event time/revolution counter provides the delta required for speed calculation.

---

# 20. Example Speed Calculation

Assume two consecutive valid wheel events produce:

```text
Previous:
event time = 6307
revolutions = 3

New:
event time = 7229
revolutions = 4
```

Then:

```text
delta_time = 922
delta_revolutions = 1
```

Speed:

```text
speed =
    1
    × 2.170
    × 1024
    × 3.6
    / 922

≈ 8.68 km/h
```

---

# 21. Recommended Constants

For this bike:

```text
ANT_BIKE_SPEED_DEVICE_TYPE = 123

WHEEL_CIRCUMFERENCE_MM = 2170
WHEEL_CIRCUMFERENCE_M  = 2.170

SPEED_TIMEOUT_SECONDS = 5.0
```

---

# 22. USB Disconnect / Reconnect Handling

Embedded Linux software should expect the USB dongle to be removed or reset.

Recommended behavior:

```text
1. If serial read/open fails:
       mark ANT interface offline
       speed = 0

2. Periodically search for the dongle again.

3. Identify it by:
       VID 0x1915
       PID 0x520F

4. Reopen the CDC port.

5. Reset previous ANT delta state.

6. Wait for new packets before calculating speed again.
```

Do not retain the previous wheel timestamp/revolution values across a dongle reconnect.

---

# 23. Summary

The host receives already-decoded ANT metadata over USB CDC:

```text
ANT,<device_number>,<device_type>,<transmission_type>,<rssi>,<8-byte-payload>
```

For the current wheel sensor:

```text
device_type = 123
```

Decode:

```text
event_time  = little-endian payload bytes 4..5
wheel_revs  = little-endian payload bytes 6..7
```

Use:

```text
wheel circumference = 2.170 m
```

Calculate:

```text
speed_km_h =
    delta_revolutions
    × 2.170
    × 1024
    × 3.6
    / delta_event_time
```

Handle both ANT counters as wrapping 16-bit unsigned values.

Update speed only when a **new wheel revolution** is observed.

If no new wheel revolution occurs for:

```text
5 seconds
```

force:

```text
speed = 0.0 km/h
```

This host-side interface does not require the application developer to implement ANT or ANT+ radio communication.
