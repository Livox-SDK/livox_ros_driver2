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
from pathlib import Path
from tempfile import TemporaryDirectory


def update_config_link(config_dir, back_model, front_model):
    if config_dir is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            config_dir = Path(get_package_share_directory("thorvald_core")) / "config"
        except (ImportError, LookupError) as error:
            print(f"Cannot locate thorvald_core: {error}. Source your ROS 2 workspace "
                  "or specify --config-dir.", file=sys.stderr)
            return 1
    config_dir = config_dir.expanduser().resolve()
    target = config_dir / f"{back_model}_{front_model}_multi.json"
    link = config_dir / "livox_multi.json"
    if not target.is_file():
        print(f"Lidar configuration does not exist: {target}. "
              "Existing livox_multi.json was not changed.", file=sys.stderr)
        return 1
    if link.exists() and not link.is_symlink():
        print(f"Refusing to replace non-symlink: {link}", file=sys.stderr)
        return 1
    with TemporaryDirectory(prefix=".livox_config_", dir=config_dir) as temporary_dir:
        replacement = Path(temporary_dir) / "livox_multi.json"
        replacement.symlink_to(target.name)
        os.replace(replacement, link)
    print(f"Configuration: {link} -> {target.name}")
    return 0


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
        "and BACK_LASER_IP using read-only UDP discovery, then select the matching "
        "thorvald_core/config/livox_multi.json symlink."
    )
    parser.add_argument("--timeout", type=float, default=5.0, help="Discovery timeout in seconds (default: 5)")
    parser.add_argument("--host-ip", default=os.environ.get("ENO_HOST_IP") or "0.0.0.0",
                        help="Local lidar-facing IPv4 address (default: ENO_HOST_IP or all interfaces)")
    parser.add_argument("--config-dir", type=Path, help="Override thorvald_core's ROS package config directory")
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be a finite positive number")
    try:
        host_address = ipaddress.IPv4Address(args.host_ip)
    except ipaddress.AddressValueError:
        parser.error("--host-ip must be a local IPv4 address")
    if host_address.is_multicast or args.host_ip == "255.255.255.255":
        parser.error("--host-ip must be a local IPv4 address")
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
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        try:
            probe.bind((str(host_address), 56000))
        except OSError as error:
            print(f"Cannot bind Livox discovery {host_address}:56000: {error}. "
                  "Check the local host IP and stop any Livox driver/SDK using "
                  "that port before retrying.", file=sys.stderr)
            return 1
        print(f"Livox discovery on {host_address}:56000 (unicast and broadcast)", flush=True)
        deadline = time.monotonic() + args.timeout
        next_request = 0.0
        while pending:
            now = time.monotonic()
            if now >= deadline:
                break
            if now >= next_request:
                for address in sorted(pending) + ["255.255.255.255"]:
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
    models = {}
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
        else:
            models[position] = model
    if not successful:
        return 1
    return update_config_link(args.config_dir, models["BACK"], models["FRONT"])


try:
    sys.exit(main())
except KeyboardInterrupt:
    sys.exit(130)
except OSError as error:
    print(f"Lidar identification/configuration failed: {error}", file=sys.stderr)
    sys.exit(1)
PY