#!/usr/bin/env python3
"""
ANT+ Scanner terminal monitor for the custom PCA10059 firmware.

Input format from dongle:
    ANT,<device_number>,<device_type>,<transmission_type>,<rssi_dbm>,<16 hex chars>

Example:
    ANT,37907,123,1,-58,80FFFFFFA3180300

No dependencies beyond pyserial.
"""

import argparse
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import serial


DEVICE_TYPES = {
    1:   "ANT-FS",
    11:  "Bike Power",
    12:  "Environment (legacy)",
    15:  "Multi-sport Speed/Distance",
    16:  "Control",
    17:  "Fitness Equipment",
    18:  "Blood Pressure",
    19:  "Geocache",
    20:  "Light Electric Vehicle",
    25:  "Environment",
    26:  "Racquet",
    27:  "Control Hub",
    31:  "Muscle Oxygen",
    35:  "Bike Light",
    36:  "Bike Light Shared",
    38:  "Extended Display",
    40:  "Bike Radar",
    119: "Weight Scale",
    120: "Heart Rate",
    121: "Bike Speed + Cadence",
    122: "Bike Cadence",
    123: "Bike Speed",
    124: "Stride Speed/Distance",
}


def u16le(data: bytes, offset: int) -> int:
    return data[offset] | (data[offset + 1] << 8)


def delta_u16(new: int, old: int) -> int:
    return (new - old) & 0xFFFF


@dataclass
class DeviceState:
    device_number: int
    device_type: int
    transmission_type: int
    rssi: int = -127
    last_seen: float = 0.0
    packets: int = 0
    raw: str = ""
    summary: str = "waiting"
    page: Optional[int] = None

    # speed-only / cadence-only state
    last_event_time: Optional[int] = None
    last_revs: Optional[int] = None
    last_speed_event_wall: Optional[float] = None

    # combined speed/cadence state
    last_cad_time: Optional[int] = None
    last_cad_revs: Optional[int] = None
    last_speed_time: Optional[int] = None
    last_speed_revs: Optional[int] = None

    speed_kmh: Optional[float] = None
    cadence_rpm: Optional[float] = None
    power_w: Optional[int] = None
    heart_rate_bpm: Optional[int] = None


def decode_speed_only(state: DeviceState, payload: bytes, wheel_m: float) -> str:
    # ANT+ Bike Speed:
    # bytes 4..5 = event time (1/1024 s)
    # bytes 6..7 = cumulative wheel revolutions.
    event_time = u16le(payload, 4)
    revs = u16le(payload, 6)

    if state.last_event_time is not None and state.last_revs is not None:
        dt = delta_u16(event_time, state.last_event_time)
        dr = delta_u16(revs, state.last_revs)

        if dt and dr:
            speed_ms = dr * wheel_m * 1024.0 / dt
            state.speed_kmh = speed_ms * 3.6
            state.last_speed_event_wall = time.monotonic()

    state.last_event_time = event_time
    state.last_revs = revs

    if state.speed_kmh is None:
        return f"wheel revs={revs}, event={event_time}"

    return f"{state.speed_kmh:.1f} km/h  wheel_revs={revs}"

def decode_cadence_only(state: DeviceState, payload: bytes) -> str:
    # ANT+ Bike Cadence: bytes 4..5 event time (1/1024 s), 6..7 cumulative crank revs.
    event_time = u16le(payload, 4)
    revs = u16le(payload, 6)

    if state.last_event_time is not None and state.last_revs is not None:
        dt = delta_u16(event_time, state.last_event_time)
        dr = delta_u16(revs, state.last_revs)

        if dt:
            state.cadence_rpm = 60.0 * dr * 1024.0 / dt

    state.last_event_time = event_time
    state.last_revs = revs

    if state.cadence_rpm is None:
        return f"crank revs={revs}, event={event_time}"
    return f"{state.cadence_rpm:.1f} rpm  crank_revs={revs}"


def decode_speed_cadence(state: DeviceState, payload: bytes, wheel_m: float) -> str:
    cad_time = u16le(payload, 0)
    cad_revs = u16le(payload, 2)
    speed_time = u16le(payload, 4)
    speed_revs = u16le(payload, 6)

    if state.last_cad_time is not None and state.last_cad_revs is not None:
        dt = delta_u16(cad_time, state.last_cad_time)
        dr = delta_u16(cad_revs, state.last_cad_revs)
        if dt:
            state.cadence_rpm = 60.0 * dr * 1024.0 / dt

    if state.last_speed_time is not None and state.last_speed_revs is not None:
        dt = delta_u16(speed_time, state.last_speed_time)
        dr = delta_u16(speed_revs, state.last_speed_revs)
        if dt:
            state.speed_kmh = dr * wheel_m * 1024.0 / dt * 3.6

    state.last_cad_time = cad_time
    state.last_cad_revs = cad_revs
    state.last_speed_time = speed_time
    state.last_speed_revs = speed_revs

    parts = []
    if state.speed_kmh is not None:
        parts.append(f"{state.speed_kmh:.1f} km/h")
    if state.cadence_rpm is not None:
        parts.append(f"{state.cadence_rpm:.1f} rpm")
    if not parts:
        parts.append(f"speed_revs={speed_revs} cad_revs={cad_revs}")
    return "  ".join(parts)


def decode_hr(state: DeviceState, payload: bytes) -> str:
    # ANT+ HRM common pages: low 7 bits of byte 0 = page; byte 7 = computed heart rate.
    state.page = payload[0] & 0x7F
    hr = payload[7]
    state.heart_rate_bpm = hr
    beat_time = u16le(payload, 4)
    beat_count = payload[6]
    return f"{hr} bpm  page={state.page} beat_count={beat_count} event={beat_time}"


def decode_power(state: DeviceState, payload: bytes) -> str:
    # Bicycle Power page 16 ("standard power-only main data page").
    page = payload[0] & 0x7F
    state.page = page
    if page == 16:
        event_count = payload[1]
        pedal_power = payload[2]
        cadence = payload[3]
        accum_power = u16le(payload, 4)
        inst_power = u16le(payload, 6)
        state.power_w = inst_power
        if cadence != 0xFF:
            state.cadence_rpm = float(cadence)

        cadence_txt = "n/a" if cadence == 0xFF else f"{cadence} rpm"
        balance_txt = "n/a" if pedal_power == 0xFF else f"0x{pedal_power:02X}"
        return (
            f"{inst_power} W  cadence={cadence_txt}  "
            f"event={event_count} accum={accum_power} pedal={balance_txt}"
        )

    return f"page={page}  raw={payload.hex().upper()}"


def decode_fitness_equipment(state: DeviceState, payload: bytes) -> str:
    page = payload[0] & 0x7F
    state.page = page
    # Keep FE decoding conservative for now. We identify the ANT+ FE page and retain raw data.
    return f"FE page={page}  raw={payload.hex().upper()}"


def decode_payload(state: DeviceState, payload: bytes, wheel_m: float) -> str:
    if state.device_type == 123:
        return decode_speed_only(state, payload, wheel_m)
    if state.device_type == 122:
        return decode_cadence_only(state, payload)
    if state.device_type == 121:
        return decode_speed_cadence(state, payload, wheel_m)
    if state.device_type == 120:
        return decode_hr(state, payload)
    if state.device_type == 11:
        return decode_power(state, payload)
    if state.device_type == 17:
        return decode_fitness_equipment(state, payload)

    page = payload[0] & 0x7F
    state.page = page
    return f"page={page}  raw={payload.hex().upper()}"


def parse_ant_line(line: str) -> Optional[Tuple[int, int, int, int, bytes, str]]:
    line = line.strip()
    if not line.startswith("ANT,"):
        return None

    parts = line.split(",")
    if len(parts) != 6:
        return None

    try:
        devnum = int(parts[1])
        devtype = int(parts[2])
        tx = int(parts[3])
        rssi = int(parts[4])
        raw = parts[5].strip().upper()
        if len(raw) != 16:
            return None
        payload = bytes.fromhex(raw)
    except (ValueError, TypeError):
        return None

    return devnum, devtype, tx, rssi, payload, raw


def clear_screen() -> None:
    sys.stdout.write("\033[2J\033[H")


def strength_bar(rssi: int) -> str:
    # Deliberately coarse indication, not a calibrated distance estimate.
    if rssi >= -45:
        return "████"
    if rssi >= -60:
        return "███ "
    if rssi >= -75:
        return "██  "
    return "█   "


def render(devices: Dict[Tuple[int, int, int], DeviceState], port: str, wheel_m: float, speed_timeout: float) -> None:
    now = time.monotonic()

    # Force bike speed to zero if no new wheel revolution has occurred
    # for the configured timeout. This also works if the ANT sensor stops transmitting.
    for d in devices.values():
        if (
            d.device_type == 123
            and d.speed_kmh is not None
            and d.last_speed_event_wall is not None
            and (now - d.last_speed_event_wall) >= speed_timeout
        ):
            d.speed_kmh = 0.0
            d.summary = f"0.0 km/h  wheel_revs={d.last_revs}"
    clear_screen()

    print("ANT+ Scanner Monitor")
    print(f"Port: {port}   Wheel circumference: {wheel_m*1000:.0f} mm   Speed timeout: {speed_timeout:.1f} s")
    print("Ctrl+C to exit")
    print()
    print(
        f"{'Device':>6}  {'Type':>4}  {'Profile':<24} "
        f"{'Tx':>3}  {'RSSI':>5}  {'Sig':<4}  {'Age':>5}  {'Packets':>7}  Data"
    )
    print("-" * 125)

    active = sorted(
        devices.values(),
        key=lambda d: (now - d.last_seen, d.device_type, d.device_number),
    )

    if not active:
        print("Waiting for ANT+ traffic...")

    for d in active:
        age = now - d.last_seen
        profile = DEVICE_TYPES.get(d.device_type, "Unknown")
        stale = " [STALE]" if age > 5.0 else ""
        print(
            f"{d.device_number:6d}  {d.device_type:4d}  {profile:<24.24} "
            f"{d.transmission_type:3d}  {d.rssi:5d}  {strength_bar(d.rssi)}  "
            f"{age:4.1f}s  {d.packets:7d}  {d.summary}{stale}"
        )

    print()
    print("Known type IDs: HR=120, Speed+Cadence=121, Cadence=122, Speed=123, Power=11, FE=17")


def main() -> int:
    ap = argparse.ArgumentParser(description="Monitor/decode the custom PCA10059 ANT+ scanner")
    ap.add_argument("port", nargs="?", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200,
                    help="CDC baud setting (mostly nominal for USB CDC; default 115200)")
    ap.add_argument("--wheel-mm", type=float, default=2100.0,
                    help="Wheel circumference in mm for speed calculation (default 2100)")
    ap.add_argument("--raw", action="store_true",
                    help="Print every incoming raw ANT line instead of live table")
    ap.add_argument("--refresh", type=float, default=0.25,
                    help="Live table refresh interval in seconds")
    ap.add_argument("--speed-timeout", type=float, default=5.0,
                    help="Seconds without a new wheel revolution before speed is forced to 0 (default 5.0)")
    args = ap.parse_args()

    wheel_m = args.wheel_mm / 1000.0
    devices: Dict[Tuple[int, int, int], DeviceState] = {}

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"Could not open {args.port}: {e}", file=sys.stderr)
        return 2

    print(f"Opened {args.port}. Waiting for ANT+ scanner...")
    last_render = 0.0

    try:
        while True:
            raw_line = ser.readline()
            now = time.monotonic()

            if raw_line:
                line = raw_line.decode("ascii", errors="replace").strip()

                if args.raw:
                    print(line)
                    continue

                parsed = parse_ant_line(line)
                if parsed:
                    devnum, devtype, tx, rssi, payload, raw = parsed
                    key = (devnum, devtype, tx)

                    state = devices.get(key)
                    if state is None:
                        state = DeviceState(
                            device_number=devnum,
                            device_type=devtype,
                            transmission_type=tx,
                        )
                        devices[key] = state

                    state.rssi = rssi
                    state.last_seen = now
                    state.packets += 1
                    state.raw = raw
                    state.summary = decode_payload(state, payload, wheel_m)

            if not args.raw and now - last_render >= args.refresh:
                render(devices, args.port, wheel_m, args.speed_timeout)
                last_render = now

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

