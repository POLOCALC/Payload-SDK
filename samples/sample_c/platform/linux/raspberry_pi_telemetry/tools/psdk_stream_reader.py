#!/usr/bin/env python3
"""
psdk_stream_reader.py - decode the binary stream written by psdk_telemetry (for testing).

Usage:
  psdk_telemetry | python3 psdk_stream_reader.py             # live, from the program's stdout
  python3 psdk_stream_reader.py capture.bin                  # from a file written with --output
  python3 psdk_stream_reader.py capture.bin --quiet          # only status, topic table, stats and a summary

Record format: see application/telemetry_stream.h.
"""
import argparse
import datetime
import math
import struct
import sys
from collections import Counter

SYNC = b"\xA5\x5A"
HEADER = struct.Struct("<BBHHqII")          # version, type, topic, length, host ns, aircraft ms, aircraft us
HEADER_SIZE = 2 + HEADER.size               # 24 with the sync bytes
MAX_PAYLOAD = 2048                          # TELEMETRY_STREAM_MAX_PAYLOAD

RECORD_DATA, RECORD_STATUS, RECORD_TOPIC_TABLE, RECORD_STATS = 1, 2, 3, 4
STATES = {1: "STARTING", 2: "CONNECTING", 3: "CONNECTED", 4: "SUBSCRIBED", 5: "STOPPING",
          10: "INFO", 11: "WARNING", 100: "ERROR"}

# packed PSDK structs, by topic name (see psdk_lib/include/dji_fc_subscription.h)
DECODERS = {
    # imu group
    "quaternion":            ("<4f",     ("q0", "q1", "q2", "q3")),
    "velocity":              ("<3fB",    ("vx", "vy", "vz", "health")),
    # units seen on the Matrice 400 (the header says m/s^2 for all three):
    "acceleration_ground":   ("<3f",     ("x", "y", "z")),         # gravity removed, ~0 at rest
    "acceleration_body":     ("<3f",     ("x_g", "y_g", "z_g")),   # g, gravity included (z ~ -1 at rest)
    "acceleration_raw":      ("<3f",     ("x", "y", "z")),         # gravity removed, ~0 at rest
    "angular_rate_fused":    ("<3f",     ("x", "y", "z")),
    "angular_rate_raw":      ("<3f",     ("x", "y", "z")),
    "compass":               ("<3h",     ("x", "y", "z")),
    "altitude_fused":        ("<f",      ("m",)),
    "altitude_barometer":    ("<f",      ("m",)),
    "height_fusion":         ("<f",      ("height_m",)),
    # fast group
    "gps_position":          ("<3i",     ("x", "y", "z")),
    "gps_velocity":          ("<3f",     ("x_cms", "y_cms", "z_cms")),
    "rtk_position":          ("<ddf",    ("lon_deg", "lat_deg", "hfsl_m")),
    "rtk_velocity":          ("<3f",     ("x_cms", "y_cms", "z_cms")),
    "rtk_yaw":               ("<f",      ("yaw_deg",)),            # float deg on the M400 (header: int16)
    "rtk_position_info":     ("<B",      ("info",)),
    "rtk_yaw_info":          ("<B",      ("info",)),
    "position_fused":        ("<ddfHBB", ("lon_rad", "lat_rad", "alt_m", "sats", "flag1", "flag2")),  # 24 B on the M400
    "position_vo":           ("<3fB",    ("x_m", "y_m", "z_m", "health")),
    "avoid_data":            ("<6fB",    ("down", "front", "right", "back", "left", "up", "health")),
    "rc":                    ("<6h",     ("roll", "pitch", "yaw", "throttle", "mode", "gear")),
    "rc_with_flag":          ("<4fB",    ("pitch", "roll", "yaw", "throttle", "flags")),
    "gimbal_angles":         ("<3f",     ("pitch", "roll", "yaw")),
    "gimbal_status":         ("<I",      ("flags",)),
    "three_gimbal_data":     ("<9f",     ("g1_roll", "g1_pitch", "g1_yaw", "g2_roll", "g2_pitch", "g2_yaw",
                                          "g3_roll", "g3_pitch", "g3_yaw")),
    # status group
    "gps_details":           ("<6f2I2H", ("hdop", "pdop", "fix", "vacc_mm", "hacc_mm", "sacc_cms",
                                          "gps_sats", "glonass_sats", "total_sats", "counter")),
    "gps_signal_level":      ("<B",      ("level",)),
    "gps_control_level":     ("<B",      ("level",)),
    "flight_status":         ("<B",      ("status",)),
    "display_mode":          ("<B",      ("mode",)),
    "landing_gear":          ("<B",      ("state",)),
    "motor_start_error":     ("<H",      ("error",)),
    "control_device":        ("<BB",     ("byte0", "byte1")),
    "rtk_connect_status":    ("<H",      ("flags",)),
    "gimbal_control_mode":   ("<B",      ("mode",)),
    "flight_anomaly":        ("<I",      ("flags",)),
    "imu_atti_navi":         ("<HH9f4fHHI", ("version", "flag", "pn_x", "pn_y", "pn_z", "vn_x", "vn_y", "vn_z",
                                             "an_x", "an_y", "an_z", "q0", "q1", "q2", "q3", "resv", "cnt", "timestamp")),
    # slow group
    "gps_date":              ("<I",      ("yyyymmdd",)),
    "gps_time":              ("<I",      ("hhmmss",)),
    "altitude_of_homepoint": ("<f",      ("m",)),
    "height_relative":       ("<f",      ("m",)),
    "home_point_set_status": ("<B",      ("status",)),
    "home_point_info":       ("<dd",     ("lat_rad", "lon_rad")),
    "hard_sync":             ("<IIIHB4f3f3f", ("t_2p5ms", "t_1ns", "reset_2p5ms", "index", "flag", "q0", "q1", "q2",
                                               "q3", "ax_g", "ay_g", "az_g", "gx", "gy", "gz")),
    "battery_info":          ("<IiiB",   ("capacity_mah", "voltage_mv", "current_ma", "percentage")),
}

BATTERY = struct.Struct("<BBiiIIhBB8sBBB")      # T_DjiFcSubscriptionSingleBatteryInfo, 33 bytes
ESC = struct.Struct("<hhHhH")                   # ESCStatusIndividual, 10 bytes; ESC data = 8 of them
ESC_FLAGS = ("stall", "no_load", "unbalanced", "disconnected", "temp_high")
ANOMALY_FLAGS = ("impact_in_air", "random_fly", "height_ctrl_fail", "tilt_ctrl_fail", "yaw_ctrl_fail",
                 "falling", "wind_level1", "wind_level2", "compass_install_error", "imu_install_error",
                 "esc_temp_high", "esc_disconnected", "gps_yaw_error")


def decode_battery(payload):
    (_, index, mv, ma, full, remained, temp, cells, percent, state, _, _, sop) = BATTERY.unpack(payload)
    bits = int.from_bytes(state[:4], "little")
    return (f"index={index} {percent}% {mv / 1000:.2f}V {ma / 1000:.2f}A {remained}/{full}mAh "
            f"{temp / 10:.1f}C cells={cells} sop={sop} cell_break={(bits >> 12) & 0x1F} "
            f"self_check={(bits >> 17) & 0x7} closed_reason={(bits >> 27) & 0x1F}")


def decode_position_fused(payload):
    lon, lat, alt, sats, flag1, flag2 = struct.unpack("<ddfHBB", payload)
    return (f"lat_deg={math.degrees(lat):.8f}  lon_deg={math.degrees(lon):.8f}  alt_m={alt:.3f}  sats={sats}  "
            f"flags={flag1},{flag2}")


def decode_esc(payload):
    parts = []
    for i in range(8):
        ma, rpm, mv, temp, flags = ESC.unpack_from(payload, i * ESC.size)
        if not (ma or rpm or mv or temp):
            continue                            # slot not used by this aircraft
        names = [n for b, n in enumerate(ESC_FLAGS) if flags >> b & 1]
        parts.append(f"esc{i + 1}:{rpm}rpm/{ma / 1000:.2f}A/{mv / 1000:.2f}V/{temp / 10:.1f}C" +   # 0.1 C on the M400
                     (f"[{','.join(names)}]" if names else ""))
    return " ".join(parts) or "all zero"


def decode_anomaly(payload):
    (flags,) = struct.unpack("<I", payload)
    return f"flags=0x{flags:05X} " + (",".join(n for b, n in enumerate(ANOMALY_FLAGS) if flags >> b & 1) or "none")


# decoders that need more than a flat struct: name -> (size, function)
SPECIAL = {
    "battery1": (BATTERY.size, decode_battery),
    "battery2": (BATTERY.size, decode_battery),
    "esc_data": (8 * ESC.size, decode_esc),
    "position_fused": (24, decode_position_fused),
    "flight_anomaly": (4, decode_anomaly),
}


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def records(stream):
    """Yield (type, topic, host_ns, aircraft_ms, aircraft_us, payload); resynchronise on bad data."""
    buf = b""
    while True:
        chunk = stream.read(4096)
        if not chunk:
            return
        buf += chunk
        while True:
            start = buf.find(SYNC)
            if start < 0:
                buf = buf[-1:]                    # keep a possible half sync byte
                break
            if len(buf) - start < HEADER_SIZE:
                buf = buf[start:]
                break
            version, rtype, topic, length, host_ns, ac_ms, ac_us = HEADER.unpack_from(buf, start + 2)
            if length > MAX_PAYLOAD:
                buf = buf[start + 1:]             # not a real header
                continue
            end = start + HEADER_SIZE + length + 2
            if len(buf) < end:
                buf = buf[start:]
                break
            body = buf[start + 2:end - 2]
            (crc,) = struct.unpack_from("<H", buf, end - 2)
            if crc16(body) != crc:
                print("  [bad CRC, resyncing]", file=sys.stderr)
                buf = buf[start + 1:]
                continue
            yield rtype, topic, host_ns, ac_ms, ac_us, buf[start + HEADER_SIZE:end - 2]
            buf = buf[end:]


def main():
    parser = argparse.ArgumentParser(description="Decode the psdk_telemetry binary stream")
    parser.add_argument("input", nargs="?", default="-", help="file written with --output, or '-' for stdin")
    parser.add_argument("--quiet", action="store_true", help="don't print every data record")
    args = parser.parse_args()

    stream = sys.stdin.buffer if args.input == "-" else open(args.input, "rb")
    names = {}                                   # topic id -> name, from the TOPIC_TABLE record
    counts = Counter()

    for rtype, topic, host_ns, ac_ms, ac_us, payload in records(stream):
        t = datetime.datetime.fromtimestamp(host_ns / 1e9).strftime("%H:%M:%S.%f")[:-3]
        if rtype == RECORD_STATUS:
            print(f"{t} STATUS {STATES.get(payload[0], payload[0])}: {payload[1:].decode('utf-8', 'replace')}")
        elif rtype == RECORD_TOPIC_TABLE:
            (count,) = struct.unpack_from("<H", payload, 0)
            print(f"{t} TOPIC TABLE ({count} topics)")
            for i in range(count):
                tid, hz, size, sub = struct.unpack_from("<HHHB", payload, 2 + i * 39)
                name = payload[2 + i * 39 + 7: 2 + (i + 1) * 39].split(b"\0")[0].decode()
                names[tid] = name
                print(f"    {tid:3d} {name:<18} {hz:3d} Hz  {size:3d} B  {'subscribed' if sub else 'NOT subscribed'}")
        elif rtype == RECORD_STATS:
            dropped, count = struct.unpack_from("<IH", payload, 0)
            received = [struct.unpack_from("<HI", payload, 6 + i * 6) for i in range(count)]
            print(f"{t} STATS dropped={dropped} " + " ".join(f"{names.get(tid, tid)}={n}" for tid, n in received))
        elif rtype == RECORD_DATA:
            name = names.get(topic, f"topic{topic}")
            counts[name] += 1
            if args.quiet:
                continue
            fmt = DECODERS.get(name)
            special = SPECIAL.get(name)
            if special and special[0] == len(payload):
                text = special[1](payload)
            elif fmt and struct.calcsize(fmt[0]) == len(payload):
                values = dict(zip(fmt[1], struct.unpack(fmt[0], payload)))
                text = "  ".join(f"{k}={v:.6g}" if isinstance(v, float) else f"{k}={v}" for k, v in values.items())
            else:
                text = payload.hex()
            print(f"{t} [{ac_ms / 1000:10.3f}] {name:<18} {text}")

    print("\nData records per topic: " + (", ".join(f"{k}={v}" for k, v in sorted(counts.items())) or "none"))


if __name__ == "__main__":
    main()
