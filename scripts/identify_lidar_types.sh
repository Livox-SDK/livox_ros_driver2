#!/usr/bin/env bash

if ! command -v python3 >/dev/null 2>&1; then
  printf 'Error: python3 is required.\n' >&2
  exit 1
fi

exec python3 - "$@" <<'PY'
import argparse
import binascii
import ipaddress
import math
import os
import socket
import struct
import sys
import time


def discovery_request():
    header = struct.pack("<BBHIHBB6s", 0xAA, 0, 24, 1, 0, 0, 0, bytes(6))
    return header + struct.pack("<HI", binascii.crc_hqx(header, 0xFFFF), 0)


def parse_reply(packet, source_ip):
    if len(packet) < 48:
        return None
    sof, version, length, sequence, command, kind, sender = struct.unpack_from(
        "<BBHIHBB", packet
    )
    if (sof, version, length, command, kind, sender) != (0xAA, 0, len(packet), 0, 1, 1):
        return None
    header_crc, payload_crc = struct.unpack_from("<HI", packet, 18)
    payload = packet[24:]
    if header_crc != binascii.crc_hqx(packet[:18], 0xFFFF):
        return None
    if payload_crc != binascii.crc32(payload):
        return None
    status, device_type, serial, lidar_ip, command_port = struct.unpack_from("<BB16s4sH", payload)
    if status != 0 or socket.inet_ntoa(lidar_ip) != source_ip:
        return None
    return device_type, serial.split(b"\0", 1)[0].decode("ascii", errors="replace")


def main():
    parser = argparse.ArgumentParser(
        description="Run on the robot to identify Livox models at FRONT_LASER_IP "
        "and BACK_LASER_IP using read-only UDP discovery."
    )
    parser.add_argument("--timeout", type=float, default=5.0, help="Discovery timeout in seconds (default: 5)")
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be a finite positive number")
    positions = {}
    for position in ("FRONT", "BACK"):
        variable = f"{position}_LASER_IP"
        value = os.environ.get(variable, "")
        try:
            address = ipaddress.IPv4Address(value)
        except ipaddress.AddressValueError:
            parser.error(f"{variable} must be an exported IPv4 address (got {value!r})")
        if address.is_unspecified or address.is_multicast or value == "255.255.255.255":
            parser.error(f"{variable} must be a unicast lidar IPv4 address")
        positions[position] = str(address)
    if positions["FRONT"] == positions["BACK"]:
        parser.error("FRONT_LASER_IP and BACK_LASER_IP must identify different lidars")

    pending = set(positions.values())
    detected = {}
    request = discovery_request()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        try:
            probe.bind(("0.0.0.0", 56000))
        except OSError as error:
            print(f"Cannot bind Livox discovery UDP port 56000: {error}. "
                  "Stop any Livox driver/SDK using that port and retry.", file=sys.stderr)
            return 1
        deadline = time.monotonic() + args.timeout
        next_request = 0.0
        while pending:
            now = time.monotonic()
            if now >= deadline:
                break
            if now >= next_request:
                for address in sorted(pending):
                    try:
                        probe.sendto(request, (address, 56000))
                    except OSError as error:
                        print(f"Discovery request to {address} failed: {error}", file=sys.stderr)
                next_request = now + 0.5
            probe.settimeout(min(deadline - now, max(0.001, next_request - now)))
            try:
                packet, (source_ip, source_port) = probe.recvfrom(65535)
            except socket.timeout:
                continue
            if source_ip not in pending or source_port != 56000:
                continue
            result = parse_reply(packet, source_ip)
            if result is not None:
                detected[source_ip] = result
                pending.remove(source_ip)

    successful = True
    for position, address in positions.items():
        if address not in detected:
            print(f"{position} ({address}): no Livox discovery reply within {args.timeout:g}s")
            successful = False
            continue
        device_type, serial = detected[address]
        model = {9: "Mid-360", 35: "Mid-360S"}.get(device_type)
        print(f"{position} ({address}): {model or 'Unknown Livox model'} "
              f"(device_type={device_type}, serial={serial})")
        if model is None:
            successful = False
    return 0 if successful else 1


try:
    sys.exit(main())
except KeyboardInterrupt:
    sys.exit(130)
except OSError as error:
    print(f"Livox discovery failed: {error}", file=sys.stderr)
    sys.exit(1)
PY