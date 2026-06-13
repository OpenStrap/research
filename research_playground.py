#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
the reference client — The complete WHOOP 4.0 (Harvard) BLE client & protocol library.
================================================================================

ONE FILE that covers *everything* in the WHOOP band ecosystem that has been
reverse-engineered: the BLE link, command/response, events, live sensor
streams, historical sync + acknowledgement, on-disk storage, offline replay,
the charging puck / battery pack, ship-mode/reboot, optical tuning, alarms,
haptics, firmware-update opcodes, and the analytics you have to compute
yourself because the strap never does.

It is BOTH:
  • a library  ->  `from whoop import WhoopClient, build_command, decode_frame, ...`
  • a CLI      ->  `python the reference client <subcommand>`   (see `python the reference client --help`)

It runs WITHOUT any hardware for the protocol/decode/replay/self-test paths
(pure stdlib). Live BLE needs `bleak`; the charger serial-reset needs `pyserial`.
Both are imported lazily so the rest of the module works regardless.

────────────────────────────────────────────────────────────────────────────
HOW THE APP <-> BAND COMMUNICATION WORKS  (the whole picture in one breath)
────────────────────────────────────────────────────────────────────────────
The WHOOP 4.0 ("Harvard", firmware "Puffin") is a *dumb sensor pipe*. It does
NO analytics. It exposes a proprietary GATT service with five characteristics:

    61080001-…  service
    61080002-…  cmdToStrap     WRITE (no response)   app -> strap   (commands)
    61080003-…  cmdFromStrap   NOTIFY               strap -> app   (cmd responses)
    61080004-…  eventFromStrap NOTIFY               strap -> app   (system events)
    61080005-…  dataFromStrap  NOTIFY               strap -> app   (sensor + sync)
    61080007-…  memfault       NOTIFY               strap -> app   (crash logs — noise)

Every byte on every characteristic is wrapped in one framing envelope:

    [0xAA] [u16 LE frame_size] [CRC8(size bytes)] [inner, 0-padded to /4] [u32 LE CRC32(inner)]
            └ frame_size = len(padded_inner) + 4 (the +4 is the trailing CRC32)
    CRC8 = custom poly-0x07 LUT over the 2 size bytes only.
    CRC32 = standard zlib, over the *padded* inner content only.

The inner content is:  [packet_type] [seq] [opcode/event_id_lo] [body…]

Talking to the band is a fixed dance (verified on real hw, fw 41.17.6.0):
  1. Pair/bond (Android createBond / Linux bluetoothctl pair; macOS works on
     newer fw even though CoreBluetooth has no pair() API).
  2. Subscribe to 0003/0004/0005(/0007). Request MTU 247.
  3. Fire the 5-packet INIT (GET_HELLO, GET_ADV_NAME, GET_DATA_RANGE,
     GET_ALARM_TIME, SEND_HISTORICAL_DATA) one-at-a-time.
  4. The strap floods 0x2F (data) + 0x30 (events), punctuated by 0x31 METADATA
     markers. You MUST acknowledge each "HistoryEnd" marker (cmd 0x17) or the
     strap re-streams that batch forever ("Groundhog Day" bug) and burns battery.
        HistoryStart (sub 1) -> ignore
        HistoryEnd   (sub 2) -> ACK with 0x17 + [0x01]+token(8B), KEEP GOING
        HistoryComplete(sub 3)-> STOP (do NOT ack)
  5. Enable live streams (HR 0x03, IMU 0x3F/0x6A, optical 0x6B/0x6C/0x9A).
     Optical toggles need a TWO-byte [revision=0x01, enable=0x01] payload.
  6. Heartbeat LINK_VALID (0x01) every ~10 s; poll battery (0x1A) every ~20 s
     (the standard 0x180F battery service is bugged — always 100%).
  7. On shutdown, turn the persistent/high-frequency toggles back OFF or the
     LEDs keep blasting and drain the pack.

The charging puck ("battery pack") is its own thing: it appears over BLE as
events (7/8 charging, 21/22 pack connected/removed, 109 pack serial) and over
USB as a serial port — a dead pack is revived by opening it at 9600-8N1 and
writing the ASCII string "Reboot" (see `reset_charger_via_serial`).

────────────────────────────────────────────────────────────────────────────
Every value here is the one observed on a live WHOOP 4.0. PROTOCOL.md is the
field-level reference; this file is the working implementation of it.
================================================================================
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import struct
import sys
import time
import zlib
from dataclasses import dataclass, field, asdict
from enum import IntEnum
from typing import Any, Callable, Iterable, Optional

# Optional, lazily-required heavy deps. The protocol/decode/replay/self-test
# layers never need them; only live BLE / serial does.
try:  # pragma: no cover - import guard
    from bleak import BleakClient, BleakScanner  # type: ignore
    _HAVE_BLEAK = True
except Exception:  # pragma: no cover
    BleakClient = BleakScanner = None  # type: ignore
    _HAVE_BLEAK = False


# ════════════════════════════════════════════════════════════════════════════
# SECTION 1 — CONSTANTS  (UUIDs, packet types, commands, events, records, …)
# ════════════════════════════════════════════════════════════════════════════

# ── 1.1 BLE service / characteristic UUIDs, per generation ───────────────────
# + (device generation enum).
class Gen(IntEnum):
    """Device generation. Decides UUID family + frame header length."""
    HARVARD = 4   # WHOOP 4.0  — 4-byte frame header (the one you most likely have)
    MAVERICK = 3  # WHOOP 3.0  — fd4b family, 8-byte header
    PUFFIN = 5    # WHOOP 5.0  — fd4b family (real devices), 8-byte header


# Each family shares the suffix pattern: 0001=service, 0002=cmd_to, 0003=cmd_from,
# 0004=events, 0005=data, 0007=memfault.
UUID_FAMILIES = {
    Gen.HARVARD: "61080{n:03x}-8d6d-82b8-614a-1c8cb0f8dcc6",
    # NB: real WHOOP 5.0 *and* the old "Maverick/Goose" both advertise fd4b…
    # (verified against goose/openwhoop). The 11500001-… in some APK enums is
    # an internal placeholder that may never have shipped — kept below for ref.
    Gen.MAVERICK: "fd4b{n:04x}-cce1-4033-93ce-002d5875f58a",
    Gen.PUFFIN: "fd4b{n:04x}-cce1-4033-93ce-002d5875f58a",
}
# Internal-only Puffin UUID seen in APK enums (likely never shipped):
PUFFIN_INTERNAL_SERVICE = "11500001-6215-11ee-8c99-0242ac120002"


def uuids_for(gen: Gen) -> dict[str, str]:
    """Return {role: full-uuid} for a generation."""
    fam = UUID_FAMILIES[gen]
    if gen == Gen.HARVARD:
        mk = lambda n: fam.format(n=n)            # 61080001 etc. (3 hex digits + leading 6108)
        return {
            "service": "61080001-8d6d-82b8-614a-1c8cb0f8dcc6",
            "cmd_to":  "61080002-8d6d-82b8-614a-1c8cb0f8dcc6",
            "cmd_from":"61080003-8d6d-82b8-614a-1c8cb0f8dcc6",
            "events":  "61080004-8d6d-82b8-614a-1c8cb0f8dcc6",
            "data":    "61080005-8d6d-82b8-614a-1c8cb0f8dcc6",
            "memfault":"61080007-8d6d-82b8-614a-1c8cb0f8dcc6",
        }
    mk = lambda n: fam.format(n=n)
    return {
        "service":  mk(0x0001), "cmd_to": mk(0x0002), "cmd_from": mk(0x0003),
        "events":   mk(0x0004), "data":   mk(0x0005), "memfault": mk(0x0007),
    }


# Default = Harvard / WHOOP 4.0.
UUID = uuids_for(Gen.HARVARD)
SERVICE_UUID  = UUID["service"]
CMD_TO_UUID   = UUID["cmd_to"]
CMD_FROM_UUID = UUID["cmd_from"]
EVENTS_UUID   = UUID["events"]
DATA_UUID     = UUID["data"]
MEMFAULT_UUID = UUID["memfault"]

# Standard BLE services (mostly useless on WHOOP):
HR_SERVICE_UUID     = "0000180d-0000-1000-8000-00805f9b34fb"  # works, but proprietary is richer
HR_MEASUREMENT_UUID = "00002a37-0000-1000-8000-00805f9b34fb"
BATTERY_SERVICE_UUID = "0000180f-0000-1000-8000-00805f9b34fb"  # BUGGED — always 100%, ignore
DEVICE_INFO_UUID     = "0000180a-0000-1000-8000-00805f9b34fb"

SOF = 0xAA  # start of frame


# ── 1.2 Packet type byte (inner[0]) ──────────────────────────────────────────
# enum.
class PacketType(IntEnum):
    COMMAND                            = 0x23  # app -> strap
    COMMAND_RESPONSE                   = 0x24  # strap -> app
    PUFFIN_COMMAND                     = 0x25  # app -> puffin (5.0)
    PUFFIN_COMMAND_RESPONSE            = 0x26
    REALTIME_DATA                      = 0x28  # live sensor records
    REALTIME_RAW_DATA                  = 0x2B
    HISTORICAL_DATA                    = 0x2F  # bulk flash records during sync
    EVENT                              = 0x30  # discrete system events
    METADATA                           = 0x31  # sync markers (HistoryStart/End/Complete)
    CONSOLE_LOGS                       = 0x32  # firmware log lines
    REALTIME_IMU_DATA_STREAM           = 0x33
    HISTORICAL_IMU_DATA_STREAM         = 0x34
    RELATIVE_PUFFIN_EVENTS             = 0x35
    PUFFIN_EVENTS_FROM_STRAP           = 0x36
    RELATIVE_BATTERY_PACK_CONSOLE_LOGS = 0x37
    PUFFIN_METADATA                    = 0x38


# ── 1.3 Command opcodes (sent inside a 0x23 COMMAND packet) ───────────────────
# (Harvard) + poohw cross-check. This is the FULL set.
class Cmd(IntEnum):
    LINK_VALID                       = 0x01  # heartbeat (send every ~10s)
    GET_MAX_PROTOCOL_VERSION         = 0x02
    TOGGLE_REALTIME_HR               = 0x03  # payload [01]/[00] -> 1Hz HR via 0x28
    REPORT_VERSION_INFO              = 0x07
    SET_CLOCK                        = 0x0A  # [u32 epoch, u32 pad]
    GET_CLOCK                        = 0x0B
    TOGGLE_GENERIC_HR_PROFILE        = 0x0E  # standard 0x180D HR profile
    TOGGLE_R7_DATA_COLLECTION        = 0x10
    RUN_HAPTIC_PATTERN_MAVERICK      = 0x13  # 12-byte pattern (3.0)
    ABORT_HISTORICAL_TRANSMITS       = 0x14  # cancel sync (saves battery)
    SEND_HISTORICAL_DATA             = 0x16  # start flash drain
    HISTORICAL_DATA_RESULT           = 0x17  # the BATCH ACK (12-byte inner)
    FORCE_TRIM                       = 0x19  # flash defrag
    GET_BATTERY_LEVEL                = 0x1A  # ALWAYS poll battery this way
    REBOOT_STRAP                     = 0x1D  # hard reset (drops BLE)
    POWER_CYCLE_STRAP                = 0x20  # deeper reset
    SET_READ_POINTER                 = 0x21  # u32 offset
    GET_DATA_RANGE                   = 0x22  # available history size
    GET_HELLO_HARVARD                = 0x23  # identity (4.0)
    START_FIRMWARE_LOAD              = 0x24
    LOAD_FIRMWARE_DATA               = 0x25
    PROCESS_FIRMWARE_IMAGE           = 0x26
    SET_LED_DRIVE                    = 0x27  # optical LED current
    GET_LED_DRIVE                    = 0x28
    SET_TIA_GAIN                     = 0x29  # optical amplifier gain
    GET_TIA_GAIN                     = 0x2A
    SET_BIAS_OFFSET                  = 0x2B
    GET_BIAS_OFFSET                  = 0x2C
    ENTER_BLE_DFU                    = 0x2D  # reboot to Nordic Secure DFU
    SET_DP_TYPE                      = 0x34
    FORCE_DP_TYPE                    = 0x35
    SEND_R10_R11_REALTIME            = 0x3F  # live R10/R11 IMU stream
    SET_ALARM_TIME                   = 0x42  # [u32 epoch, 0,0,0,0]
    GET_ALARM_TIME                   = 0x43
    RUN_ALARM                        = 0x44  # test-fire alarm
    DISABLE_ALARM                    = 0x45
    GET_ADVERTISING_NAME_HARVARD     = 0x4C
    SET_ADVERTISING_NAME_HARVARD     = 0x4D  # rename strap
    RUN_HAPTICS_PATTERN              = 0x4F  # [pattern_id, loops, 0,0,0]
    GET_ALL_HAPTICS_PATTERN          = 0x50
    START_RAW_DATA                   = 0x51
    STOP_RAW_DATA                    = 0x52
    VERIFY_FIRMWARE_IMAGE            = 0x53
    GET_BODY_LOCATION_AND_STATUS     = 0x54
    ENTER_HIGH_FREQ_SYNC             = 0x60  # turbo radio (DANGER: battery)
    EXIT_HIGH_FREQ_SYNC              = 0x61
    GET_EXTENDED_BATTERY_INFO        = 0x62
    RESET_FUEL_GAUGE                 = 0x63
    CALIBRATE_CAPSENSE               = 0x64
    TOGGLE_IMU_MODE_HISTORICAL       = 0x69
    TOGGLE_IMU_MODE                  = 0x6A  # 6-axis accel+gyro
    ENABLE_OPTICAL_DATA              = 0x6B  # 2-byte [01,01]
    TOGGLE_OPTICAL_MODE              = 0x6C  # 2-byte [01,01]
    START_DEVICE_CONFIG_KEY_EXCHANGE = 0x73
    SEND_NEXT_DEVICE_CONFIG          = 0x74
    START_FF_KEY_EXCHANGE            = 0x75
    SEND_NEXT_FF                     = 0x76
    SET_DEVICE_CONFIG_VALUE          = 0x77
    SET_FF_VALUE                     = 0x78
    GET_DEVICE_CONFIG_VALUE          = 0x79
    STOP_HAPTICS                     = 0x7A
    SELECT_WRIST                     = 0x7B  # [R/L, Bicep/Wrist, Inside/Outside]
    TOGGLE_LABRADOR_DATA_GENERATION  = 0x7C  # internal ECG subsystem
    TOGGLE_LABRADOR_RAW_SAVE         = 0x7D
    GET_FF_VALUE                     = 0x80
    SET_RESEARCH_PACKET              = 0x83
    GET_RESEARCH_PACKET              = 0x84
    TOGGLE_LABRADOR_FILTERED         = 0x8B
    SET_ADVERTISING_NAME             = 0x8C
    GET_ADVERTISING_NAME             = 0x8D
    START_FIRMWARE_LOAD_NEW          = 0x8E  # new-style Puffin OTA
    LOAD_FIRMWARE_DATA_NEW           = 0x8F
    PROCESS_FIRMWARE_IMAGE_NEW       = 0x90
    GET_HELLO                        = 0x91  # Puffin/new-style hello
    GET_BATTERY_PACK_INFO            = 0x97  # charging puck info
    TOGGLE_PERSISTENT_R20            = 0x99  # 2-byte [01,01] (Maverick optical)
    TOGGLE_PERSISTENT_R21            = 0x9A  # 2-byte [01,01] (Harvard PPG) — DANGER

    # ── Doc-sourced, NOT present in the 5.445.0 command enum. Power-state ops.
    # Treat as EXPERIMENTAL: confirm on your firmware before relying on them.
    SHIP_MODE                        = 0x37  # deep sleep (wake only on charger)
    SHIP_OFF                         = 0xCC  # wake from ship mode


# Puffin-only opcodes (sent inside a 0x25 PUFFIN_COMMAND).
class PuffinCmd(IntEnum):
    PUFFIN_GET_HELLO        = 0x02
    PUFFIN_START_FW_LOAD    = 0x03
    PUFFIN_LOAD_FW_DATA     = 0x04
    PUFFIN_PROCESS_FW       = 0x06
    HISTORICAL_PULL_ABORT   = 0x14
    SEND_HISTORICAL_DATA    = 0x16
    HISTORICAL_PULL_RESULT  = 0x17


# Opcodes that take the special TWO-byte [REVISION_1=0x01, enable] payload.
# Sending a single [0x01] is read as [revision, <missing enable>] -> no data flows.
TWO_BYTE_TOGGLES = {Cmd.ENABLE_OPTICAL_DATA, Cmd.TOGGLE_OPTICAL_MODE,
                    Cmd.TOGGLE_PERSISTENT_R20, Cmd.TOGGLE_PERSISTENT_R21}

# Commands that can burn battery / brick the link if left on. Never auto-fire.
DANGEROUS_CMDS = {Cmd.TOGGLE_PERSISTENT_R21, Cmd.ENTER_HIGH_FREQ_SYNC,
                  Cmd.REBOOT_STRAP, Cmd.POWER_CYCLE_STRAP, Cmd.ENTER_BLE_DFU,
                  Cmd.SHIP_MODE, Cmd.SHIP_OFF, Cmd.START_FIRMWARE_LOAD,
                  Cmd.PROCESS_FIRMWARE_IMAGE}


# ── 1.4 Event IDs (inner[2:4] of a 0x30 EVENT packet) ─────────────────────────
# enum. The complete set.
class Event(IntEnum):
    UNDEFINED                         = 0
    ERROR                             = 1
    CONSOLE_OUTPUT                    = 2
    BATTERY_LEVEL                     = 3   # format unstable across fw — poll 0x1A instead
    SYSTEM_CONTROL                    = 4
    CHARGING_ON                       = 7
    CHARGING_OFF                      = 8
    WRIST_ON                          = 9
    WRIST_OFF                         = 10
    BLE_CONNECTION_UP                 = 11
    BLE_CONNECTION_DOWN               = 12
    RTC_LOST                          = 13  # battery died completely
    DOUBLE_TAP                        = 14  # physical gesture
    BOOT                              = 15
    SET_RTC                           = 16
    TEMPERATURE_LEVEL                 = 17  # live extraction unreliable; skin temp via R24
    PAIRING_MODE                      = 18
    SERIAL_HEAD_CONNECTED             = 19
    SERIAL_HEAD_REMOVED               = 20
    BATTERY_PACK_CONNECTED            = 21  # charging puck attached
    BATTERY_PACK_REMOVED              = 22  # charging puck removed
    BLE_BONDED                        = 23  # pair success
    BLE_HR_PROFILE_ENABLED            = 24
    BLE_HR_PROFILE_DISABLED           = 25
    TRIM_ALL_DATA                     = 26
    TRIM_ALL_DATA_ENDED               = 27
    FLASH_INIT_COMPLETE               = 28
    STRAP_CONDITION_REPORT            = 29
    BOOT_REPORT                       = 30
    EXIT_VIRGIN_MODE                  = 31
    CAPTOUCH_AUTOTHRESHOLD_ACTION     = 32
    BLE_REALTIME_HR_ON                = 33
    BLE_REALTIME_HR_OFF               = 34
    ACCELEROMETER_RESET               = 35
    AFE_RESET                         = 36
    SHIP_MODE_ENABLED                 = 37
    SHIP_MODE_DISABLED                = 38
    SHIP_MODE_BOOT                    = 39
    CH1_SATURATION_DETECTED           = 40  # optical blinded
    CH2_SATURATION_DETECTED           = 41
    ACCELEROMETER_SATURATION_DETECTED = 42
    BLE_SYSTEM_RESET                  = 43
    BLE_SYSTEM_ON                     = 44
    BLE_SYSTEM_INITIALIZED            = 45
    RAW_DATA_COLLECTION_ON            = 46
    RAW_DATA_COLLECTION_OFF           = 47
    STRAP_DRIVEN_ALARM_SET            = 56
    STRAP_DRIVEN_ALARM_EXECUTED       = 57
    APP_DRIVEN_ALARM_EXECUTED         = 58
    STRAP_DRIVEN_ALARM_DISABLED       = 59
    HAPTICS_FIRED                     = 60
    EXTENDED_BATTERY_INFORMATION      = 63  # u32 LE / 10 = %
    HIGH_FREQ_SYNC_PROMPT             = 96  # flash filling — strap begs for 0x60
    HIGH_FREQ_SYNC_ENABLED            = 97
    HIGH_FREQ_SYNC_DISABLED           = 98
    HAPTICS_TERMINATED                = 100
    PPG_SEARCH_ON                     = 102  # LEDs hunting for a pulse
    PPG_SEARCH_OFF                    = 103  # locked, or gave up
    BATTERY_PACK_INFO                 = 109  # ASCII puck serial in payload


# ── 1.5 Sensor record types (inner[1] of a data packet) ──────────────────────
class Record(IntEnum):
    R2  = 2    # sparse PPG fallback (big-endian!)
    R7  = 7    # HR realtime variant (1936 B)
    R9  = 9
    R10 = 10   # HR + 6-axis IMU + GSR (Harvard 1928 B)
    R11 = 11   # companion IMU (R10 + 4B tail)
    R12 = 12
    R18 = 18
    R20 = 20   # Maverick optical (2140 B, not Harvard)
    R21 = 21   # Harvard optical / 6-channel PPG (1244 B)
    R24 = 24   # 1 Hz telemetry: HR + tri-axial accel (sync/historical; app relays raw to cloud)
    R25 = 25   # RR-interval time series (sync-only)
    COMPREHENSIVE = 0x5C  # newer firmware combined HR+temp+SpO2 record


# ── 1.6 Metadata (sync) sub-types — inner[2] of a 0x31 METADATA packet ────────
class SyncMeta(IntEnum):
    HISTORY_START    = 1  # informational — ignore
    HISTORY_END      = 2  # ACK with 0x17, then KEEP listening
    HISTORY_COMPLETE = 3  # sync finished — STOP, do not ACK


# ── 1.7 Misc enums for command payloads ──────────────────────────────────────
class Wrist(IntEnum):
    RIGHT = 1
    LEFT = 2


class BodyLimb(IntEnum):
    BICEP = 1
    WRIST = 2


class BodySide(IntEnum):
    INSIDE = 1
    OUTSIDE = 2


HAPTIC_SHORT_PULSE = 2  # pattern id 2 = a single short buzz (good for "I'm alive")
REVISION_1 = 0x01       # the magic first byte for *_HARVARD / 2-byte toggle payloads


# ════════════════════════════════════════════════════════════════════════════
# SECTION 2 — CRC PRIMITIVES
# ════════════════════════════════════════════════════════════════════════════
#. CRC8 (poly 0x07) over the 2 length bytes; CRC32
# (zlib) over the padded inner; CRC16-modbus only used by the Gen5 8-byte header.

CRC8_TABLE = bytes([
    0,7,14,9,28,27,18,21,56,63,54,49,36,35,42,45,
    112,119,126,121,108,107,98,101,72,79,70,65,84,83,90,93,
    224,231,238,233,252,251,242,245,216,223,214,209,196,195,202,205,
    144,151,158,153,140,139,130,133,168,175,166,161,180,179,186,189,
    199,192,201,206,219,220,213,210,255,248,241,246,227,228,237,234,
    183,176,185,190,171,172,165,162,143,136,129,134,147,148,157,154,
    39,32,41,46,59,60,53,50,31,24,17,22,3,4,13,10,
    87,80,89,94,75,76,69,66,111,104,97,102,115,116,125,122,
    137,142,135,128,149,146,155,156,177,182,191,184,173,170,163,164,
    249,254,247,240,229,226,235,236,193,198,207,200,221,218,211,212,
    105,110,103,96,117,114,123,124,81,86,95,88,77,74,67,68,
    25,30,23,16,5,2,11,12,33,38,47,40,61,58,51,52,
    78,73,64,71,82,85,92,91,118,113,120,127,106,109,100,99,
    62,57,48,55,34,37,44,43,6,1,8,15,26,29,20,19,
    174,169,160,167,178,181,188,187,150,145,152,159,138,141,132,131,
    222,217,208,215,194,197,204,203,230,225,232,239,250,253,244,243,
])


def crc8(data: bytes) -> int:
    """Custom CRC-8 (poly 0x07), applied ONLY to the 2-byte length field."""
    crc = 0
    for b in data:
        crc = CRC8_TABLE[(crc ^ b) & 0xFF]
    return crc


def crc32(data: bytes) -> int:
    """Standard zlib CRC-32 over the (padded) inner content."""
    return zlib.crc32(bytes(data)) & 0xFFFFFFFF


def crc16_modbus(data: bytes) -> int:
    """CRC-16/MODBUS — only used inside the Gen5 (WHOOP 5.0/3.0) 8-byte header."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


# ════════════════════════════════════════════════════════════════════════════
# SECTION 3 — FRAMING  (build / parse / reassemble) for Gen4 and Gen5
# ════════════════════════════════════════════════════════════════════════════

def pad4(data: bytes) -> bytes:
    """Zero-pad to a 4-byte boundary (CRC32 is computed over the padded form)."""
    return data + b"\x00" * ((-len(data)) % 4)


def build_frame(inner: bytes, gen: Gen = Gen.HARVARD) -> bytes:
    """Wrap inner content in the WHOOP frame envelope for the given generation."""
    inner_p = pad4(inner)
    crc_tail = struct.pack("<I", crc32(inner_p))
    declared = len(inner_p) + 4  # +4 = trailing CRC32, what the size field counts
    if gen == Gen.HARVARD:
        # [0xAA][u16 size][crc8(size)][inner_p][crc32]
        len_b = struct.pack("<H", declared)
        return bytes([SOF]) + len_b + bytes([crc8(len_b)]) + inner_p + crc_tail
    # Gen5 / Maverick: [0xAA][0x01][u16 size @2..4][0x00][0x01][crc16(head6)][inner_p][crc32]
    head = bytes([SOF, 0x01]) + struct.pack("<H", declared) + bytes([0x00, 0x01])
    head += struct.pack("<H", crc16_modbus(head))
    return head + inner_p + crc_tail


def header_len(gen: Gen) -> int:
    return 4 if gen == Gen.HARVARD else 8


@dataclass
class Frame:
    """A fully-parsed, validated frame envelope."""
    raw: bytes
    gen: Gen
    inner: bytes            # padded inner content (type, seq, opcode, body…)
    crc8_ok: bool
    crc32_ok: bool

    @property
    def packet_type(self) -> int: return self.inner[0] if self.inner else -1
    @property
    def seq(self) -> int:        return self.inner[1] if len(self.inner) > 1 else -1
    @property
    def opcode(self) -> int:     return self.inner[2] if len(self.inner) > 2 else -1
    @property
    def body(self) -> bytes:     return self.inner[3:]


def parse_frame(raw: bytes, gen: Gen = Gen.HARVARD) -> Optional[Frame]:
    """Parse a single complete frame. Returns None if too short / bad SOF."""
    hl = header_len(gen)
    if len(raw) < hl + 4 or raw[0] != SOF:
        return None
    if gen == Gen.HARVARD:
        declared = struct.unpack_from("<H", raw, 1)[0]
        crc8_ok = raw[3] == crc8(raw[1:3])
        inner_start = 4
    else:
        if raw[1] != 0x01:
            return None
        declared = struct.unpack_from("<H", raw, 2)[0]
        crc8_ok = struct.unpack_from("<H", raw, 6)[0] == crc16_modbus(raw[0:6])
        inner_start = 8
    total = hl + declared
    if len(raw) < total:
        return None
    inner = raw[inner_start: inner_start + declared - 4]
    stored = struct.unpack_from("<I", raw, inner_start + declared - 4)[0]
    return Frame(raw=raw[:total], gen=gen, inner=inner,
                 crc8_ok=crc8_ok, crc32_ok=(stored == crc32(inner)))


class FrameReassembler:
    """
    BLE notifications are MTU-sized chunks; a big record (R10 ~1936 B) spans many.
    A NEW frame always begins with 0xAA; continuation chunks do not. The strap
    also inserts 0x00 padding between consecutive records. feed() returns every
    complete Frame it can carve out of the running buffer.
    """
    def __init__(self, gen: Gen = Gen.HARVARD):
        self.gen = gen
        self.buf = bytearray()

    def feed(self, chunk: bytes) -> list[Frame]:
        """
        Append the chunk and extract every complete frame by its DECLARED length.

        Critical: we do NOT treat "chunk starts with 0xAA" as a new-frame signal —
        the int16 sensor payload of a big record (R10/R21) routinely contains 0xAA
        bytes, and a BLE notification boundary can land on one. Instead we read the
        length field at a true frame boundary and consume exactly that many bytes.
        If the buffer head isn't a SOF (e.g. we subscribed mid-record), we resync
        forward to the next 0xAA.
        """
        out: list[Frame] = []
        self.buf += chunk
        hl = header_len(self.gen)

        def resync() -> bool:
            nxt = self.buf.find(SOF, 1)
            if nxt < 0:
                self.buf.clear()
                return False
            del self.buf[:nxt]
            return True

        while len(self.buf) >= hl + 4:
            if self.buf[0] != SOF:
                if not resync():
                    break
                continue
            if self.gen == Gen.HARVARD:
                declared = struct.unpack_from("<H", self.buf, 1)[0]
            else:
                if self.buf[1] != 0x01:
                    if not resync():
                        break
                    continue
                declared = struct.unpack_from("<H", self.buf, 2)[0]
            total = hl + declared
            if declared < 4 or total > 4096:        # implausible length → spurious SOF
                if not resync():
                    break
                continue
            if len(self.buf) < total:
                break                                # wait for the rest of this frame
            frame = parse_frame(bytes(self.buf[:total]), self.gen)
            if frame:
                out.append(frame)
            del self.buf[:total]
            # skip inter-record null padding
            i = 0
            while i < len(self.buf) and self.buf[i] == 0x00:
                i += 1
            if i:
                del self.buf[:i]
        if len(self.buf) > 8192:                     # safety: never grow unbounded
            self.buf.clear()
        return out


# ════════════════════════════════════════════════════════════════════════════
# SECTION 4 — COMMAND BUILDERS  (every opcode worth sending)
# ════════════════════════════════════════════════════════════════════════════

def build_command(seq: int, opcode: int, payload: bytes = b"\x00",
                  gen: Gen = Gen.HARVARD,
                  packet_type: int = PacketType.COMMAND) -> bytes:
    """Build a framed command packet: [type][seq][opcode][payload]."""
    inner = bytes([packet_type, seq & 0xFF, opcode & 0xFF]) + bytes(payload)
    return build_frame(inner, gen)


def build_puffin_command(seq: int, opcode: int, payload: bytes = b"\x00",
                         gen: Gen = Gen.PUFFIN) -> bytes:
    return build_command(seq, opcode, payload, gen, PacketType.PUFFIN_COMMAND)


def build_batch_ack(seq: int, token: bytes, gen: Gen = Gen.HARVARD) -> bytes:
    """
    The historical-sync acknowledgement (cmd 0x17).
    Inner = [0x23][seq][0x17][0x01] + token(8B)  -> 12-byte inner, 20-byte frame.
    `token` is bytes inner[13:21] of the HistoryEnd METADATA marker.
    Verified byte-identical across goose, whoomp, and two working Python clients.
    """
    assert len(token) == 8, "batch token must be 8 bytes"
    inner = bytes([PacketType.COMMAND, seq & 0xFF, Cmd.HISTORICAL_DATA_RESULT, REVISION_1]) + bytes(token)
    return build_frame(inner, gen)


# ── Convenience builders for common ops (seq defaults to 0 for one-shots) ─────
def cmd_link_valid(seq=0):           return build_command(seq, Cmd.LINK_VALID, b"\x00")
def cmd_get_hello_harvard(seq=0):    return build_command(seq, Cmd.GET_HELLO_HARVARD, b"\x00")
def cmd_get_hello(seq=0):            return build_command(seq, Cmd.GET_HELLO, bytes([REVISION_1]))
def cmd_get_battery(seq=0):          return build_command(seq, Cmd.GET_BATTERY_LEVEL, b"")
def cmd_get_extended_battery(seq=0): return build_command(seq, Cmd.GET_EXTENDED_BATTERY_INFO, b"\x00")
def cmd_get_battery_pack(seq=0):     return build_command(seq, Cmd.GET_BATTERY_PACK_INFO, b"\x00")
def cmd_get_clock(seq=0):            return build_command(seq, Cmd.GET_CLOCK, b"\x00")
def cmd_set_clock(epoch, seq=0):     return build_command(seq, Cmd.SET_CLOCK, struct.pack("<II", int(epoch), 0))
def cmd_get_data_range(seq=0):       return build_command(seq, Cmd.GET_DATA_RANGE, b"\x00")
def cmd_set_read_pointer(off, seq=0):return build_command(seq, Cmd.SET_READ_POINTER, struct.pack("<I", off))
def cmd_send_historical(seq=0):      return build_command(seq, Cmd.SEND_HISTORICAL_DATA, b"\x00")
def cmd_abort_historical(seq=0):     return build_command(seq, Cmd.ABORT_HISTORICAL_TRANSMITS, b"\x00")
def cmd_get_adv_name(seq=0):         return build_command(seq, Cmd.GET_ADVERTISING_NAME_HARVARD, b"\x00")
def cmd_set_adv_name(name, seq=0):
    # Payload: [0x01][len][ascii name][u32 0].
    b = name.encode("ascii", "ignore")[:20]
    return build_command(seq, Cmd.SET_ADVERTISING_NAME_HARVARD,
                         bytes([0x01, len(b)]) + b + b"\x00\x00\x00\x00")
def cmd_get_alarm(seq=0):            return build_command(seq, Cmd.GET_ALARM_TIME, bytes([REVISION_1]))
def cmd_get_body_location(seq=0):    return build_command(seq, Cmd.GET_BODY_LOCATION_AND_STATUS, b"\x00")
def cmd_report_version(seq=0):       return build_command(seq, Cmd.REPORT_VERSION_INFO, b"\x00")

# live-stream toggles
def cmd_toggle_hr(on=True, seq=0):       return build_command(seq, Cmd.TOGGLE_REALTIME_HR, bytes([0x01 if on else 0x00]))
def cmd_toggle_imu(on=True, seq=0):      return build_command(seq, Cmd.TOGGLE_IMU_MODE, bytes([0x01 if on else 0x00]))
def cmd_toggle_imu_hist(on=True, seq=0): return build_command(seq, Cmd.TOGGLE_IMU_MODE_HISTORICAL, bytes([0x01 if on else 0x00]))
def cmd_send_r10_r11(on=True, seq=0):    return build_command(seq, Cmd.SEND_R10_R11_REALTIME, bytes([0x01 if on else 0x00]))
def cmd_toggle_r7(on=True, seq=0):       return build_command(seq, Cmd.TOGGLE_R7_DATA_COLLECTION, bytes([0x01 if on else 0x00]))
def cmd_enable_optical(on=True, seq=0):  return build_command(seq, Cmd.ENABLE_OPTICAL_DATA, bytes([REVISION_1, 0x01 if on else 0x00]))
def cmd_toggle_optical(on=True, seq=0):  return build_command(seq, Cmd.TOGGLE_OPTICAL_MODE, bytes([REVISION_1, 0x01 if on else 0x00]))
def cmd_persistent_r20(on=True, seq=0):  return build_command(seq, Cmd.TOGGLE_PERSISTENT_R20, bytes([REVISION_1, 0x01 if on else 0x00]))
def cmd_persistent_r21(on=True, seq=0):  return build_command(seq, Cmd.TOGGLE_PERSISTENT_R21, bytes([REVISION_1, 0x01 if on else 0x00]))
def cmd_start_raw(seq=0):                return build_command(seq, Cmd.START_RAW_DATA, b"\x01")
def cmd_stop_raw(seq=0):                 return build_command(seq, Cmd.STOP_RAW_DATA, b"\x00")
def cmd_enter_hifreq(seq=0):             return build_command(seq, Cmd.ENTER_HIGH_FREQ_SYNC, b"\x00")
def cmd_exit_hifreq(seq=0):              return build_command(seq, Cmd.EXIT_HIGH_FREQ_SYNC, b"\x00")

# optical analog front-end tuning
def cmd_set_led_drive(level=0xFF, seq=0): return build_command(seq, Cmd.SET_LED_DRIVE, bytes([level & 0xFF, level & 0xFF]))
def cmd_get_led_drive(seq=0):             return build_command(seq, Cmd.GET_LED_DRIVE, bytes([REVISION_1]))
def cmd_set_tia_gain(gain=0xFF, seq=0):   return build_command(seq, Cmd.SET_TIA_GAIN, bytes([REVISION_1, gain & 0xFF]))

# alarms & haptics
def cmd_set_alarm(epoch, seq=0):     # [0x01][u32 epoch LE][u16 subsec]
    return build_command(seq, Cmd.SET_ALARM_TIME, b"\x01" + struct.pack("<I", int(epoch)) + b"\x00\x00")
def cmd_disable_alarm(seq=0):        return build_command(seq, Cmd.DISABLE_ALARM, b"\x00")
def cmd_run_alarm(seq=0):            return build_command(seq, Cmd.RUN_ALARM, b"\x00")
def cmd_run_haptic(pattern=HAPTIC_SHORT_PULSE, loops=0, seq=0):
    return build_command(seq, Cmd.RUN_HAPTICS_PATTERN, bytes([pattern & 0xFF, loops & 0xFF, 0, 0, 0]))
def cmd_run_haptic_maverick(seq=0):
    return build_command(seq, Cmd.RUN_HAPTIC_PATTERN_MAVERICK,
                         bytes([0x01, 0x2F, 0x98, 0, 0, 0, 0, 0, 0, 0, 0, 0x01]))
def cmd_stop_haptics(seq=0):         return build_command(seq, Cmd.STOP_HAPTICS, b"\x00")

# body placement
def cmd_select_wrist(side=Wrist.LEFT, limb=BodyLimb.WRIST, face=BodySide.OUTSIDE, seq=0):
    return build_command(seq, Cmd.SELECT_WRIST, bytes([int(side), int(limb), int(face)]))

# power state — EXPERIMENTAL / dangerous (see DANGEROUS_CMDS). Not auto-used.
def cmd_reboot(seq=0):       return build_command(seq, Cmd.REBOOT_STRAP, b"\x00")
def cmd_power_cycle(seq=0):  return build_command(seq, Cmd.POWER_CYCLE_STRAP, b"\x00")
def cmd_ship_mode(seq=0):    return build_command(seq, Cmd.SHIP_MODE, bytes([0x01]))
def cmd_ship_off(seq=0):     return build_command(seq, Cmd.SHIP_OFF, bytes([0x01]))


# ── 4.1 The 5-packet INIT handshake (HCI-snoop verbatim, seq 0..4) ────────────
# These are pre-computed; build_command() regenerates them byte-for-byte (the
# self-test asserts it). Send one-at-a-time, advancing on each write-ack.
INIT_PACKETS = [
    bytes.fromhex("aa0800a823002300ada86a2d"),  # seq0 GET_HELLO_HARVARD      0x23 [00]
    bytes.fromhex("aa0800a823014c00f2b5cdce"),  # seq1 GET_ADVERTISING_NAME   0x4C [00]
    bytes.fromhex("aa0800a823022200824df537"),  # seq2 GET_DATA_RANGE         0x22 [00]
    bytes.fromhex("aa0800a823034301c54dd63d"),  # seq3 GET_ALARM_TIME         0x43 [01]
    bytes.fromhex("aa0800a823041600c7c25288"),  # seq4 SEND_HISTORICAL_DATA   0x16 [00]
]


# ════════════════════════════════════════════════════════════════════════════
# SECTION 5 — DECODERS  (responses, events, metadata, sensor records)
# ════════════════════════════════════════════════════════════════════════════

def _u16(b, o):  return struct.unpack_from("<H", b, o)[0]
def _i16(b, o):  return struct.unpack_from("<h", b, o)[0]
def _u32(b, o):  return struct.unpack_from("<I", b, o)[0]
def _i32(b, o):  return struct.unpack_from("<i", b, o)[0]
def _f32(b, o):  return struct.unpack_from("<f", b, o)[0]
def _u16be(b, o):return struct.unpack_from(">H", b, o)[0]


def _arr_stats(arr: list[int]) -> dict:
    if not arr:
        return {"n": 0}
    return {"n": len(arr), "min": min(arr), "max": max(arr), "avg": sum(arr) // len(arr)}


# ── 5.1 GET_HELLO_HARVARD response ────────────────────────────────────────────
@dataclass
class HelloInfo:
    battery_pct: Optional[float] = None
    charging: Optional[bool] = None
    serial: Optional[str] = None
    commit: Optional[str] = None        # firmware git commit hash (ASCII)
    wrist_on: Optional[bool] = None     # best-effort; events 9/10 are authoritative
    raw_hex: str = ""


def _ascii_runs(data: bytes, start: int = 0, minlen: int = 4) -> list[str]:
    """Extract printable-ASCII substrings (used to find serial/commit by content)."""
    runs, cur = [], []
    for i in range(start, len(data)):
        b = data[i]
        if 0x20 <= b < 0x7F:
            cur.append(chr(b))
        else:
            if len(cur) >= minlen:
                runs.append("".join(cur))
            cur = []
    if len(cur) >= minlen:
        runs.append("".join(cur))
    return runs


def parse_hello(payload: bytes) -> HelloInfo:
    """
    Decode the GET_HELLO_HARVARD response *body* (bytes after [0x24,seq,0x23]).

    The field offsets DRIFT across firmware revisions (verified empirically on a
    live band: serial @17 not @14, battery @3 not @1). So we parse by CONTENT
    rather than fixed offsets: battery = the u16 that yields a sane 1–100%;
    serial = the first short alphanumeric ASCII run; commit = the long hex run.
    `charging` (byte 5) is stable. Authoritative battery still comes from the
    0x1A poll; wrist state from the WRIST_ON/OFF events (9/10).
    """
    info = HelloInfo(raw_hex=payload.hex())
    if len(payload) < 10:
        return info

    # battery is reported ×10 — find the first u16 LE that maps to 1–100%
    for off in range(1, 10):
        if off + 2 <= len(payload):
            v = _u16(payload, off)
            if 10 <= v <= 1009:           # 1.0% .. 100.9%
                info.battery_pct = round(v / 10.0, 1)
                break

    info.charging = bool(payload[5]) if len(payload) > 5 else None

    # wrist flag: empirically body[116] (= inner[119]) is 1 on-wrist / 0 off-wrist
    # (verified worn-vs-off byte-diff, 2026-06). Offset may drift across fw — the
    # WRIST_ON/OFF events (9/10) remain authoritative; this is a poll-time best-effort.
    if len(payload) > 116:
        info.wrist_on = bool(payload[116])

    # serial (short alnum run, e.g. "4C2248092") + firmware commit (long hex run)
    hexset = set("0123456789abcdefABCDEF")
    runs = _ascii_runs(payload, start=6, minlen=6)
    for r in runs:
        if info.serial is None and 6 <= len(r) <= 13:
            info.serial = r
        elif info.commit is None and len(r) >= 16 and all(c in hexset for c in r):
            info.commit = r
    return info


# ── 5.2 EVENT (0x30) ──────────────────────────────────────────────────────────
@dataclass
class EventInfo:
    event_id: int
    name: str
    ts_epoch: int
    payload: bytes
    decoded: dict = field(default_factory=dict)


def parse_event(inner: bytes) -> Optional[EventInfo]:
    """
    EVENT inner layout (verified vs device):
      [0]=0x30 [1]=seq [2:4]=event_id u16 [4:8]=ts_sec u32 [8:10]=subsec u16 [12:]=payload
    (the app reads sub-seconds as a u16 @8, not u32; we don't use subsec downstream.)
    """
    if len(inner) < 4 or inner[0] != PacketType.EVENT:
        return None
    eid = _u16(inner, 2)
    name = Event(eid).name if eid in Event._value2member_map_ else f"EVENT_{eid}"
    ts = _u32(inner, 4) if len(inner) >= 8 else 0
    payload = inner[12:] if len(inner) > 12 else inner[10:]
    dec: dict = {}

    if eid in (Event.BATTERY_LEVEL, Event.EXTENDED_BATTERY_INFORMATION):
        # The battery EVENT payload format is UNSTABLE across firmware revisions.
        # The canonical, reliable source is the 0x1A poll (see
        # parse_command_response). So we expose only a best-effort TENTATIVE value
        # plus the raw bytes, and never present it as authoritative.
        dec["unreliable"] = True
        dec["battery_raw_hex"] = payload.hex()
        for base in (payload, inner):
            for off in (0, 2, 4, 12):
                for size, fn in ((2, _u16), (4, _u32)):
                    if off + size <= len(base):
                        pct = fn(base, off) / 10.0
                        if 0 < pct <= 100:
                            dec["battery_pct_tentative"] = round(pct, 1)
                            break
                if "battery_pct_tentative" in dec:
                    break
            if "battery_pct_tentative" in dec:
                break
    elif eid == Event.TEMPERATURE_LEVEL and len(payload) >= 2:
        # NOTE: the APK parser for live temp is an empty constructor — treat as
        # tentative. Some firmware does put i16/10 °C here.
        t = _i16(payload, 0) / 10.0
        if 10 <= t <= 50:
            dec["skin_temp_c"] = round(t, 1)
    elif eid == Event.BATTERY_PACK_INFO and payload:
        dec["puck_serial"] = payload.split(b"\x00")[0].decode("ascii", "replace")
    elif eid in (Event.CHARGING_ON, Event.CHARGING_OFF):
        dec["charging"] = (eid == Event.CHARGING_ON)
    elif eid in (Event.WRIST_ON, Event.WRIST_OFF):
        dec["on_wrist"] = (eid == Event.WRIST_ON)
    elif eid in (Event.BATTERY_PACK_CONNECTED, Event.BATTERY_PACK_REMOVED):
        dec["pack_connected"] = (eid == Event.BATTERY_PACK_CONNECTED)
    return EventInfo(eid, name, ts, payload, dec)


# ── 5.3 COMMAND_RESPONSE (0x24) ───────────────────────────────────────────────
@dataclass
class CmdResponse:
    opcode: int
    name: str
    payload: bytes
    decoded: dict = field(default_factory=dict)


def parse_command_response(inner: bytes) -> Optional[CmdResponse]:
    if len(inner) < 3 or inner[0] != PacketType.COMMAND_RESPONSE:
        return None
    op = inner[2]
    name = Cmd(op).name if op in Cmd._value2member_map_ else f"CMD_{op:#x}"
    payload = inner[3:]
    dec: dict = {}
    if op == Cmd.GET_BATTERY_LEVEL and len(inner) >= 7:
        dec["battery_pct"] = round(_u16(inner, 5) / 10.0, 1)   # u16 LE @ inner[5:7] / 10
    elif op == Cmd.GET_HELLO_HARVARD:
        dec["hello"] = asdict(parse_hello(payload))
    elif op == Cmd.GET_ADVERTISING_NAME_HARVARD:
        dec["name"] = payload.split(b"\x00")[0].decode("ascii", "replace")
    elif op == Cmd.GET_BATTERY_PACK_INFO and len(payload) >= 2:
        dec["pack_level"] = payload[0]
        dec["pack_charging"] = bool(payload[1])
        if len(payload) >= 24:
            dec["pack_serial"] = payload[8:24].split(b"\x00")[0].decode("ascii", "replace")
    elif op in (Cmd.GET_CLOCK,) and len(payload) >= 4:
        dec["epoch"] = _u32(payload, 0)
    elif op == Cmd.GET_DATA_RANGE:
        dec["raw"] = payload.hex()
    return CmdResponse(op, name, payload, dec)


# ── 5.4 METADATA (0x31) sync markers ──────────────────────────────────────────
@dataclass
class MetaMarker:
    sub: int
    name: str
    token: Optional[bytes] = None   # 8-byte batch token (HistoryEnd only)
    batch_id: Optional[int] = None


def parse_metadata(inner: bytes) -> Optional[MetaMarker]:
    if len(inner) < 3 or inner[0] != PacketType.METADATA:
        return None
    sub = inner[2]
    name = SyncMeta(sub).name if sub in SyncMeta._value2member_map_ else f"META_{sub}"
    token = batch_id = None
    if sub == SyncMeta.HISTORY_END and len(inner) >= 21:
        token = inner[13:21]                 # the 8 bytes the ACK echoes
        batch_id = _u32(inner, 17)           # 2nd u32 of the token = batch id
    return MetaMarker(sub, name, token, batch_id)


# ── 5.5 Harvard "big" sensor records (R10 / R21 / R24 / R25 / R2) ─────────────
# All offsets are into `inner` (offset 0 = the packet_type byte). Verified
# against (R10) and (R21) + the protocol notes.
@dataclass
class R10:  # HR + 6-axis IMU (Harvard, ~1928 B)
    ts_epoch: int; hr: int; gsr: Optional[int]
    accel: dict; gyro: dict
    accel_x: list = field(default_factory=list, repr=False)
    accel_y: list = field(default_factory=list, repr=False)
    accel_z: list = field(default_factory=list, repr=False)
    gyro_x: list = field(default_factory=list, repr=False)
    gyro_y: list = field(default_factory=list, repr=False)
    gyro_z: list = field(default_factory=list, repr=False)


def parse_r10(inner: bytes) -> Optional[R10]:
    if len(inner) < 1287:
        return None
    def arr(off): return [ _i16(inner, off + 2*i) for i in range(100) ]
    ts  = _u32(inner, 7) if len(inner) >= 11 else 0
    hr  = inner[17] if len(inner) > 17 else 0
    # GSR @154 is EMPIRICAL — the device does NOT read it.
    # Treat as unverified; do not rely on it.
    gsr = _u16(inner, 154) if len(inner) >= 156 else None
    ax, ay, az = arr(85), arr(285), arr(485)
    gx, gy, gz = arr(688), arr(888), arr(1088)
    return R10(ts, hr, gsr,
               {"x": _arr_stats(ax), "y": _arr_stats(ay), "z": _arr_stats(az)},
               {"x": _arr_stats(gx), "y": _arr_stats(gy), "z": _arr_stats(gz)},
               ax, ay, az, gx, gy, gz)


def parse_r17(inner: bytes) -> Optional[dict]:
    """R17 "Labrador Filtered" — REALTIME_RAW_DATA (0x2B) record type 17. THE RR/HRV
    carrier, per device (LE, offsets into full inner):
      [3:7] u32 ts seconds, [24:26] u16 RR count, [26:] count×int16 RR intervals.
    RR UNIT IS UNCONFIRMED (ms vs 1/1024s ticks) — validate on hardware before trusting."""
    if len(inner) < 26 or inner[1] != 17:
        return None
    ts = _u32(inner, 3)
    n = _u16(inner, 24)
    rr = [_i16(inner, 26 + 2 * i) for i in range(n) if 26 + 2 * i + 2 <= len(inner)]
    return {"ts_epoch": ts, "rr_count": n, "rr_raw": rr,
            "rmssd_if_ms": rmssd(rr) if len(rr) >= 2 else None}


@dataclass
class R21:  # 6-channel optical PPG (Harvard, ~1244 B)
    ts_epoch: int; led_drive: int; sample_count: int
    channels: dict
    ch_a: list = field(default_factory=list, repr=False)
    ch_b: list = field(default_factory=list, repr=False)
    ch_c: list = field(default_factory=list, repr=False)
    ch_d: list = field(default_factory=list, repr=False)
    ch_e: list = field(default_factory=list, repr=False)
    ch_f: list = field(default_factory=list, repr=False)


def parse_r21(inner: bytes) -> Optional[R21]:
    if len(inner) < 620:
        return None
    def arr(off, n=100): return [ _u16(inner, off + 2*i) for i in range(n) if off + 2*i + 2 <= len(inner) ]
    ts   = _u32(inner, 7) if len(inner) >= 11 else 0
    led  = _u16(inner, 14)
    cnt  = _u16(inner, 16)
    a, b, c = arr(20), arr(220), arr(420)              # green1, green2(often 0), IR
    d = arr(632) if len(inner) >= 832 else []
    e = arr(832) if len(inner) >= 1032 else []
    f = arr(1032) if len(inner) >= 1232 else []        # red (SpO2 with C)
    return R21(ts, led, cnt,
               {k: _arr_stats(v) for k, v in
                (("a", a), ("b", b), ("c", c), ("d", d), ("e", e), ("f", f))},
               a, b, c, d, e, f)


@dataclass
class R24:  # type-24 data record, 1 Hz (historical / sync-only).
    # Header [3:13] + HR[17] are confirmed (HR matches the live stream within a beat).
    # The rest of the payload is relayed to WHOOP's cloud uncalibrated. The offsets
    # below were verified on 127,971 of our own stored records and cross-checked
    # against an independent implementation; only fields that survived are decoded.
    # Raw ADCs (spo2_red_raw, skin_temp_raw) are RELATIVE — WHOOP computes SpO2 % and
    # °C server-side, never on the wire. Everything else stays in raw_tail.
    ts_epoch: int            # u32 @[7:11]  unix seconds
    ts_subsec: int           # u16 @[11:13] sub-seconds
    counter: int             # u32 @[3:7]   record counter (+1/rec)
    hr: int                  # u8  @[17]    bpm; 0 = no reading
    rr_count: int            # u8  @[18]    R-R intervals in this record (0–4)
    rr_intervals_ms: list    # i16 LE from [19], rr_count of them — ms (the HRV source)
    ppg_green: int           # u16 @[29]    raw green-LED PPG ADC (pulsatile)
    accel_g: tuple           # (x,y,z) float32 g @[36:48]; |g|≈1 at rest (corpus mean 1.012)
    spo2_red_raw: int        # u16 @[64]    raw red ADC — RELATIVE (SpO2 % computed in cloud)
    skin_temp_raw: int       # u16 @[68]    raw temp ADC — RELATIVE (°C computed in cloud)
    raw_tail: str            # [13:] hex — kept so records can be re-decoded later


def parse_r24(inner: bytes) -> Optional[R24]:
    if len(inner) < 89:
        return None
    n = inner[18]
    rr = []
    o = 19
    for _ in range(n):
        if o + 2 > len(inner):
            break
        v = _i16(inner, o)
        if v > 0:
            rr.append(v)
        o += 2
    return R24(
        ts_epoch=_u32(inner, 7),
        ts_subsec=_u16(inner, 11),
        counter=_u32(inner, 3),
        hr=inner[17],
        rr_count=n,
        rr_intervals_ms=rr,
        ppg_green=_u16(inner, 29),
        accel_g=(round(_f32(inner, 36), 4), round(_f32(inner, 40), 4),
                 round(_f32(inner, 44), 4)),
        spo2_red_raw=_u16(inner, 64),
        skin_temp_raw=_u16(inner, 68),
        raw_tail=inner[13:].hex(),
    )


def parse_r25(inner: bytes) -> list[int]:
    """RR-interval time series: i16 LE array from inner[15:]. Each = ms between beats."""
    out = []
    o = 15
    while o + 2 <= len(inner):
        v = _i16(inner, o)
        if v != 0:
            out.append(v)
        o += 2
    return out


@dataclass
class R2:  # sparse PPG fallback (big-endian!)
    ir: int; green: int; red: int


def parse_r2(inner: bytes) -> Optional[R2]:
    if len(inner) < 13:
        return None
    return R2(ir=_u16be(inner, 7), green=_u16be(inner, 9), red=_u16be(inner, 11))


# ── 5.6 Compact realtime decoders ─────────────────────────────────────────────
# REALTIME_DATA (0x28) layout VERIFIED against the device,
# little-endian, offsets into the FULL inner packet (offset 0 = the 0x28 byte):
#   [1] u8 revision  [2:6] u32 ts SECONDS  [6:8] u16 sub-seconds (/32768)
#   [8] u8 HEART RATE  [18] off-wrist flag (0 = on-wrist)  [19] body location.
# NOTE: 0x28 carries NO RR-intervals (the earlier poohw/WG50 "ts@0,HR/256,rr…"
# layout was WRONG for this firmware). RR/IBI lives in REALTIME_RAW_DATA (0x2B).
@dataclass
class RealtimeHR:
    hr_bpm: int; ts_epoch: int; subsec: int; on_wrist: bool


def parse_realtime_hr(inner: bytes) -> Optional[RealtimeHR]:
    """Parse a 0x28 REALTIME_DATA packet (FULL inner, not stripped body)."""
    if len(inner) < 20 or inner[0] != PacketType.REALTIME_DATA:
        return None
    ts = _u32(inner, 2)
    subsec = _u16(inner, 6)
    hr = inner[8]
    on_wrist = inner[18] == 0
    return RealtimeHR(hr, ts, subsec, on_wrist)


def parse_realtime_accel(body: bytes, scale: float = 1/4096) -> list[tuple]:  # ±8g, 4096 LSB/g (1g at rest verified)
    """0x33 IMU: skip 1 cmd byte, then int16 LE x,y,z triplets -> g."""
    data = body[1:] if body else b""
    out = []
    o = 0
    while o + 6 <= len(data):
        x, y, z = struct.unpack_from("<hhh", data, o)
        out.append((round(x*scale, 4), round(y*scale, 4), round(z*scale, 4)))
        o += 6
    return out


def parse_realtime_temp(body: bytes) -> Optional[float]:
    """0x28 temp variants: u16/100, i16/10, or raw byte, validated to [25,45]°C."""
    if len(body) < 3:
        return None
    for val in (_u16(body, 1) / 100.0, _i16(body, 1) / 10.0, float(body[1])):
        if 25.0 <= val <= 45.0:
            return round(val, 2)
    return None


def parse_comprehensive_5c(body: bytes) -> Optional[dict]:
    """0x5C record: [0:4]ts [4]HR [5]rr_count [6:6+2N]RR [22:34]temp/1e5 [34:84]spo2-raw."""
    if len(body) < 6:
        return None
    ts = _u32(body, 0)
    hr = body[4]
    n = body[5]
    rr = [ _u16(body, 6 + 2*i) for i in range(n) if 6 + 2*i + 2 <= len(body) ]
    out = {"ts_epoch": ts, "hr": hr, "rr_ms": rr, "hrv_rmssd_ms": rmssd(rr)}
    if len(body) >= 34:
        raw = int.from_bytes(body[22:34], "little")
        t = raw / 100000.0
        if 25 <= t <= 45:
            out["skin_temp_c"] = round(t, 4)
    if len(body) >= 50:
        out["spo2_raw_hex"] = body[34:84].hex()
    return out


# ── 5.7 One-shot frame decoder used everywhere (CLI, replay, client) ──────────
def decode_frame(frame: Frame) -> dict:
    """Turn a parsed Frame into a structured dict describing its contents."""
    inner = frame.inner
    pt = frame.packet_type
    out: dict = {
        "packet_type": pt,
        "packet_name": PacketType(pt).name if pt in PacketType._value2member_map_ else f"0x{pt:02x}",
        "seq": frame.seq,
        "crc_ok": frame.crc8_ok and frame.crc32_ok,
    }
    try:
        if pt == PacketType.COMMAND_RESPONSE:
            r = parse_command_response(inner)
            if r: out.update(kind="cmd_response", opcode=r.name, **r.decoded)
        elif pt == PacketType.EVENT:
            e = parse_event(inner)
            if e: out.update(kind="event", event=e.name, event_id=e.event_id,
                             ts_epoch=e.ts_epoch, **e.decoded)
        elif pt == PacketType.METADATA:
            m = parse_metadata(inner)
            if m: out.update(kind="metadata", sub=m.name,
                             token=m.token.hex() if m.token else None, batch_id=m.batch_id)
        elif pt in (PacketType.HISTORICAL_DATA, PacketType.REALTIME_DATA, PacketType.REALTIME_RAW_DATA):
            out.update(_decode_data_record(inner))
        elif pt in (PacketType.REALTIME_IMU_DATA_STREAM, PacketType.HISTORICAL_IMU_DATA_STREAM):
            samples = parse_realtime_accel(frame.body)
            out.update(kind="imu_stream", n_samples=len(samples),
                       sample0=samples[0] if samples else None)
        elif pt == PacketType.CONSOLE_LOGS:
            out.update(kind="console", text=frame.body.split(b"\x00")[0].decode("ascii", "replace"))
        else:
            out.update(kind="other", body_hex=frame.body[:32].hex())
    except Exception as exc:  # never let one bad packet kill the stream
        out.update(kind="decode_error", error=str(exc), inner_hex=inner[:48].hex())
    return out


def _decode_data_record(inner: bytes) -> dict:
    """Route a data packet to the right sensor-record decoder by type byte + length."""
    rec_type = inner[1] if len(inner) > 1 else -1
    # R17 "Labrador Filtered" — RR/HRV carrier. Check FIRST (it can be short, which
    # would otherwise fall into the compact branch). Accept ANY packet type with
    # rec_type 17 (0x2B live OR 0x2F historical/flash-saved via Labrador raw-save).
    if rec_type == 17:
        r = parse_r17(inner)
        if r: return {"kind": "r17_rr", "rec_type": 17, "pkt": inner[0] if inner else -1, **r}
    # Compact realtime stream (small packet) — use the WG50/newer decoders.
    if len(inner) < 64:
        hr = parse_realtime_hr(inner)  # 0x28 — absolute offsets per the app
        if hr:
            return {"kind": "realtime_hr", "rec_type": rec_type, "hr": hr.hr_bpm,
                    "ts_epoch": hr.ts_epoch, "subsec": hr.subsec, "on_wrist": hr.on_wrist}
        body = inner[3:]
        t = parse_realtime_temp(body)
        if t is not None:
            return {"kind": "realtime_temp", "rec_type": rec_type, "skin_temp_c": t}
        return {"kind": "realtime_small", "rec_type": rec_type, "body_hex": body.hex()}
    # Big Harvard records by type byte.
    if rec_type == Record.R10:
        r = parse_r10(inner)
        if r: return {"kind": "R10", "ts_epoch": r.ts_epoch, "hr": r.hr, "gsr": r.gsr,
                      "accel": r.accel, "gyro": r.gyro}
    elif rec_type == Record.R21:
        r = parse_r21(inner)
        if r: return {"kind": "R21", "ts_epoch": r.ts_epoch, "led_drive": r.led_drive,
                      "sample_count": r.sample_count, "channels": r.channels}
    elif rec_type == Record.R24:
        r = parse_r24(inner)
        if r: return {"kind": "R24_telemetry", "ts_epoch": r.ts_epoch,
                      "ts_subsec": r.ts_subsec, "counter": r.counter, "hr": r.hr,
                      "rr_count": r.rr_count, "rr_intervals_ms": r.rr_intervals_ms,
                      "ppg_green": r.ppg_green, "accel_g": r.accel_g,
                      "spo2_red_raw": r.spo2_red_raw, "skin_temp_raw": r.skin_temp_raw,
                      "raw_tail": r.raw_tail}
    elif rec_type == Record.R25:
        rr = parse_r25(inner)
        return {"kind": "R25_rr", "n": len(rr), "rmssd_ms": rmssd(rr)}
    elif rec_type == Record.R2:
        r = parse_r2(inner)
        if r: return {"kind": "R2", **asdict(r)}
    elif rec_type == Record.COMPREHENSIVE:
        c = parse_comprehensive_5c(inner[3:])
        if c: return {"kind": "comprehensive_0x5c", **c}
    rname = Record(rec_type).name if rec_type in Record._value2member_map_ else f"R{rec_type}"
    return {"kind": "data", "rec_type": rec_type, "rec_name": rname,
            "len": len(inner), "body_hex": inner[3:35].hex(), "raw_inner": inner.hex()}


# ════════════════════════════════════════════════════════════════════════════
# SECTION 6 — ANALYTICS  (the strap computes NONE of this — you do)
# ════════════════════════════════════════════════════════════════════════════

def spo2_from_ratio(r: float) -> float:
    """Empirical pulse-ox curve. R = (AC_red/DC_red)/(AC_ir/DC_ir)."""
    return max(85.0, min(100.0, 110.0 - 25.0 * r))


def rmssd(rr_ms: list[float]) -> Optional[float]:
    """HRV: root-mean-square of successive RR-interval differences (ms)."""
    if len(rr_ms) < 2:
        return None
    d = [rr_ms[i+1] - rr_ms[i] for i in range(len(rr_ms) - 1)]
    return round((sum(x*x for x in d) / len(d)) ** 0.5, 2)


def strain_from_hr_series(hr_series: list[int], rhr: int = 50, hr_max: int = 190) -> float:
    """Banister TRIMP-based strain on WHOOP's 0–21 scale."""
    trimp = 0.0
    for hr in hr_series:
        ratio = (hr - rhr) / max(1, (hr_max - rhr))
        ratio = max(0.0, min(1.0, ratio))
        trimp += ratio * 0.64 * math.exp(1.92 * ratio)
    return round(math.log(trimp + 1) / math.log(1.5), 2) if trimp > 0 else 0.0


def recovery_score(hrv_ms: float, rhr_bpm: float) -> int:
    """0–100 recovery: 70% HRV (20ms->0, 80ms->100) + 30% RHR (80bpm->0, 40bpm->100)."""
    hrv_c = max(0.0, min(1.0, (hrv_ms - 20) / 60.0))
    rhr_c = max(0.0, min(1.0, (80 - rhr_bpm) / 40.0))
    return int(round(100 * (0.7 * hrv_c + 0.3 * rhr_c)))


# ════════════════════════════════════════════════════════════════════════════
# SECTION 7 — STORAGE  ("storing": SQLite tables + a raw JSONL capture)
# ════════════════════════════════════════════════════════════════════════════

class WhoopStore:
    """
    Persists everything the band sends. Two layers:
      • A raw capture file (JSONL): one line per BLE frame {t, dir, char, hex}.
        This is the canonical record and is exactly what `replay_capture` reads.
      • A SQLite DB with decoded, query-friendly tables.
    Both are optional and independent; either can be disabled.
    """
    def __init__(self, db_path: Optional[str] = "whoop.db",
                 capture_path: Optional[str] = "whoop_capture.jsonl"):
        self.capture_path = capture_path
        self._cap = open(capture_path, "a", buffering=1) if capture_path else None
        self.db = None
        if db_path:
            import sqlite3
            self.db = sqlite3.connect(db_path)
            self._init_db()

    def _init_db(self):
        c = self.db.cursor()
        c.executescript("""
        CREATE TABLE IF NOT EXISTS frames(
            id INTEGER PRIMARY KEY, t REAL, dir TEXT, char TEXT,
            packet_type INTEGER, seq INTEGER, crc_ok INTEGER, hex TEXT);
        CREATE TABLE IF NOT EXISTS events(
            id INTEGER PRIMARY KEY, t REAL, event_id INTEGER, name TEXT,
            ts_device INTEGER, decoded TEXT);
        CREATE TABLE IF NOT EXISTS records(
            id INTEGER PRIMARY KEY, t REAL, rec_type INTEGER, kind TEXT,
            ts_device INTEGER, decoded TEXT);
        CREATE TABLE IF NOT EXISTS battery(
            id INTEGER PRIMARY KEY, t REAL, source TEXT, pct REAL, charging INTEGER);
        CREATE TABLE IF NOT EXISTS hello(
            id INTEGER PRIMARY KEY, t REAL, serial TEXT, fw TEXT, hw TEXT,
            battery REAL, charging INTEGER, wrist INTEGER, raw_hex TEXT);
        CREATE TABLE IF NOT EXISTS sync_batches(
            id INTEGER PRIMARY KEY, t REAL, token TEXT, batch_id INTEGER,
            n_records INTEGER, n_events INTEGER, complete INTEGER);
        """)
        self.db.commit()

    def capture(self, direction: str, char: str, raw: bytes):
        if self._cap:
            self._cap.write(json.dumps({"t": round(time.time(), 3), "dir": direction,
                                        "char": char, "hex": raw.hex()}) + "\n")

    def record_frame(self, direction: str, char: str, frame: Frame):
        if not self.db:
            return
        self.db.execute(
            "INSERT INTO frames(t,dir,char,packet_type,seq,crc_ok,hex) VALUES(?,?,?,?,?,?,?)",
            (time.time(), direction, char, frame.packet_type, frame.seq,
             int(frame.crc8_ok and frame.crc32_ok), frame.raw.hex()))
        self.db.commit()

    def record_decoded(self, decoded: dict):
        if not self.db:
            return
        t = time.time()
        kind = decoded.get("kind")
        if kind == "event":
            self.db.execute("INSERT INTO events(t,event_id,name,ts_device,decoded) VALUES(?,?,?,?,?)",
                            (t, decoded.get("event_id"), decoded.get("event"),
                             decoded.get("ts_epoch"), json.dumps(decoded)))
            if "battery_pct_tentative" in decoded:
                self.db.execute("INSERT INTO battery(t,source,pct,charging) VALUES(?,?,?,?)",
                                (t, "event_tentative", decoded["battery_pct_tentative"],
                                 1 if decoded.get("charging") else None))
        elif kind in ("R10", "R21", "R24_recovery", "R25_rr", "R2", "comprehensive_0x5c", "data",
                      "realtime_hr", "realtime_temp", "imu_stream"):
            self.db.execute("INSERT INTO records(t,rec_type,kind,ts_device,decoded) VALUES(?,?,?,?,?)",
                            (t, decoded.get("rec_type", -1), kind,
                             decoded.get("ts_epoch"), json.dumps(decoded)))
        elif kind == "cmd_response" and "battery_pct" in decoded:
            self.db.execute("INSERT INTO battery(t,source,pct,charging) VALUES(?,?,?,?)",
                            (t, "poll", decoded["battery_pct"], None))
        elif kind == "cmd_response" and "hello" in decoded:
            h = decoded["hello"]
            self.db.execute(
                "INSERT INTO hello(t,serial,fw,hw,battery,charging,wrist,raw_hex) VALUES(?,?,?,?,?,?,?,?)",
                (t, h.get("serial"), h.get("fw_version"), h.get("hw_version"),
                 h.get("battery_pct"), int(bool(h.get("charging"))),
                 int(bool(h.get("wrist_on"))), h.get("raw_hex")))
        self.db.commit()

    def record_batch(self, token: Optional[bytes], batch_id, n_rec, n_evt, complete):
        if self.db:
            self.db.execute(
                "INSERT INTO sync_batches(t,token,batch_id,n_records,n_events,complete) VALUES(?,?,?,?,?,?)",
                (time.time(), token.hex() if token else None, batch_id, n_rec, n_evt, int(complete)))
            self.db.commit()

    def close(self):
        if self._cap:
            self._cap.close()
        if self.db:
            self.db.close()


# ════════════════════════════════════════════════════════════════════════════
# SECTION 8 — REPLAY  ("replaying": re-run a capture through the decoders offline)
# ════════════════════════════════════════════════════════════════════════════

def replay_capture(path: str, gen: Gen = Gen.HARVARD,
                   on_decode: Optional[Callable[[dict], None]] = None,
                   only_rx: bool = True) -> list[dict]:
    """
    Read a capture and decode every frame WITHOUT hardware. Accepts either:
      • a JSONL capture written by WhoopStore (lines of {t,dir,char,hex}), or
      • a plain text file with one hex frame per line (e.g. an HCI-snoop dump,
        like reference-repos/reverse-engineering-whoop/data/*.txt).
    Returns the list of decoded dicts (and calls on_decode for each).
    """
    results: list[dict] = []
    asm = FrameReassembler(gen)

    def handle(raw: bytes):
        for fr in asm.feed(raw):
            d = decode_frame(fr)
            results.append(d)
            if on_decode:
                on_decode(d)

    with open(path, "r") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            if line[0] == "{":  # JSONL capture
                try:
                    rec = json.loads(line)
                except Exception:
                    continue
                if only_rx and rec.get("dir") not in (None, "rx"):
                    continue
                try:
                    handle(bytes.fromhex(rec["hex"]))
                except Exception:
                    pass
            else:               # raw hex-per-line
                hexs = line.split()[0]
                try:
                    handle(bytes.fromhex(hexs))
                except Exception:
                    pass
    return results


# ════════════════════════════════════════════════════════════════════════════
# SECTION 9 — LIVE BLE CLIENT  (bleak; the actual talk-to-the-band layer)
# ════════════════════════════════════════════════════════════════════════════

class WhoopClient:
    """
    Async WHOOP client built on bleak. Handles: scan/connect/bond, MTU, notify
    subscription, the 5-packet INIT, the full historical sync with correct
    3-state acknowledgement, live stream enable/disable, heartbeat + battery
    poll, capture + storage, and a graceful shutdown that turns the dangerous
    persistent/high-frequency toggles back off.

    Sequence-number discipline (critical): live commands use a HIGH counter
    (0xA0+) and sync ACKs use a LOW counter (5+, continuing from INIT 0..4) so
    the two streams never collide.
    """
    def __init__(self, address: Optional[str] = None, gen: Gen = Gen.HARVARD,
                 store: Optional[WhoopStore] = None,
                 on_decode: Optional[Callable[[dict], None]] = None,
                 verbose: bool = True):
        if not _HAVE_BLEAK:
            raise RuntimeError("bleak is required for live BLE — `pip install bleak`")
        self.address = address
        self.gen = gen
        self.uuids = uuids_for(gen)
        self.store = store
        self.on_decode = on_decode
        self.verbose = verbose

        self.client: Optional[BleakClient] = None
        self.write_lock = asyncio.Lock()       # serialize all writes (OS BLE stacks need this)
        self.cmd_seq = 0xA0                     # live commands (high range)
        self.sync_seq = 5                       # batch ACKs (continue from INIT 0..4)
        self._asm = {role: FrameReassembler(gen) for role in ("cmd_from", "events", "data")}
        self.battery_pct: Optional[float] = None
        self.charging: Optional[bool] = None
        self.hello: Optional[HelloInfo] = None
        self.sync_complete = False
        self._sync_records = 0
        self._sync_events = 0
        self._tasks: list[asyncio.Task] = []
        self._live_enabled = False

    # ── logging ──
    def _log(self, *a):
        if self.verbose:
            print(*a)

    def _emit(self, direction: str, char_role: str, raw: bytes, frame: Optional[Frame]):
        if self.store:
            self.store.capture(direction, char_role, raw)
            if frame:
                self.store.record_frame(direction, char_role, frame)

    # ── connection ──
    async def scan(self, timeout: float = 10.0) -> Optional[str]:
        """
        Find a WHOOP. On macOS a *service-filtered* scan is required — CoreBluetooth
        hides the service UUID (and often the name) in passive scans. We match on
        the advertised WHOOP service UUID OR a 'whoop' name.
        """
        self._log(f"Scanning {timeout:.0f}s for a WHOOP…")
        services = [uuids_for(g)["service"] for g in (Gen.HARVARD, Gen.PUFFIN)]
        prefixes = ("61080001", "fd4b0001")
        try:
            found = await BleakScanner.discover(timeout=timeout, service_uuids=services, return_adv=True)
            items = list(found.values())
        except TypeError:   # older bleak without these kwargs
            devs = await BleakScanner.discover(timeout=timeout)
            items = [(d, None) for d in devs]
        for dev, adv in items:
            name = (dev.name or (adv.local_name if adv else None) or "")
            svcs = [s.lower() for s in (adv.service_uuids if adv else [])]
            if "whoop" in name.lower() or any(s.startswith(p) for s in svcs for p in prefixes):
                self._log(f"  found {name or '(no name)'} @ {dev.address}")
                return dev.address
        return None

    async def connect(self) -> bool:
        if not self.address:
            self.address = await self.scan()
            if not self.address:
                self._log("No WHOOP found (put it in pair mode: off-wrist + double-tap).")
                return False
        self._log(f"Connecting to {self.address}…")
        self.client = BleakClient(self.address)
        await self.client.connect()
        if not self.client.is_connected:
            self._log("Connection failed.")
            return False
        self._log("Connected.")
        # bond (Android/Linux/Windows). macOS has no pair() API — newer fw works anyway.
        try:
            ok = await self.client.pair()
            self._log(f"Pairing: {'ok' if ok else 'not supported here'}")
        except Exception as e:
            self._log(f"Pairing unavailable ({e.__class__.__name__}) — continuing.")
        # subscribe to all notify characteristics
        await self.client.start_notify(self.uuids["cmd_from"], self._mk_handler("cmd_from"))
        await self.client.start_notify(self.uuids["events"],   self._mk_handler("events"))
        await self.client.start_notify(self.uuids["data"],     self._mk_handler("data"))
        try:
            await self.client.start_notify(self.uuids["memfault"], lambda h, d: None)
        except Exception:
            pass
        self._log("Subscribed to notify characteristics.")
        return True

    async def _write(self, raw: bytes, response: bool = True):
        # WRITE-WITH-RESPONSE by default. On macOS/CoreBluetooth this is what makes
        # the OS pair: if the strap gates the command characteristic behind
        # encryption, a with-response write returns an auth error and CoreBluetooth
        # pops the system pairing dialog (write-WITHOUT-response can never trigger it
        # — it gets no ack and no error). It also means our commands are actually
        # delivered + acknowledged, instead of silently dropped.
        async with self.write_lock:
            await self.client.write_gatt_char(self.uuids["cmd_to"], raw, response=response)
        self._emit("tx", "cmd_to", raw, parse_frame(raw, self.gen))

    async def send(self, opcode: int, payload: bytes = b"\x00") -> None:
        """Send a live command on the high (0xA0+) sequence counter."""
        frame = build_command(self.cmd_seq, opcode, payload, self.gen)
        self.cmd_seq = (self.cmd_seq + 1) & 0xFF
        if self.cmd_seq < 0xA0:           # keep it out of the sync range
            self.cmd_seq = 0xA0
        await self._write(frame)

    async def send_init(self):
        """Fire the 5-packet INIT one-at-a-time (do NOT pipeline)."""
        self._log("Sending 5-packet INIT…")
        for pkt in INIT_PACKETS:
            await self._write(pkt)
            await asyncio.sleep(0.12)

    # ── notify handling ──
    def _mk_handler(self, role: str):
        def handler(_char, data: bytearray):
            raw = bytes(data)
            for frame in self._asm[role].feed(raw):
                self._emit("rx", role, frame.raw, frame)
                self._on_frame(role, frame)
        return handler

    def _on_frame(self, role: str, frame: Frame):
        # Drop spurious / corrupt frames (e.g. junk emitted while resyncing the
        # stream after subscribing mid-record). Real frames always pass CRC.
        if not (frame.crc8_ok and frame.crc32_ok):
            return
        pt = frame.packet_type
        # sync markers first — they drive the ACK state machine
        if pt == PacketType.METADATA:
            self._handle_sync_marker(frame)
            return
        if pt in (PacketType.HISTORICAL_DATA,):
            self._sync_records += 1
        decoded = decode_frame(frame)
        if decoded.get("kind") == "event":
            self._sync_events += 1
            self._absorb_state(decoded)
        elif decoded.get("kind") == "cmd_response":
            self._absorb_state(decoded)
        if self.store:
            self.store.record_decoded(decoded)
        if self.on_decode:
            self.on_decode(decoded)
        if self.verbose and decoded.get("kind") in (
                "event", "cmd_response", "R10", "R21", "R24_recovery", "realtime_hr"):
            self._log(f"  « {decoded.get('kind')}: "
                      + json.dumps({k: v for k, v in decoded.items()
                                    if k not in ('crc_ok', 'packet_type', 'seq')})[:160])

    def _absorb_state(self, decoded: dict):
        if "battery_pct" in decoded:
            self.battery_pct = decoded["battery_pct"]
        if "charging" in decoded:
            self.charging = decoded["charging"]
        if "hello" in decoded:
            self.hello = HelloInfo(**decoded["hello"])

    def _handle_sync_marker(self, frame: Frame):
        m = parse_metadata(frame.inner)
        if not m:
            return
        if m.sub == SyncMeta.HISTORY_START:
            return  # informational
        if m.sub == SyncMeta.HISTORY_END and m.token:
            self._log(f"[SYNC] HistoryEnd batch={m.batch_id} "
                      f"records={self._sync_records} events={self._sync_events} → ACK")
            if self.store:
                self.store.record_batch(m.token, m.batch_id, self._sync_records, self._sync_events, False)
            ack = build_batch_ack(self.sync_seq, m.token, self.gen)
            self._log(f"[SYNC]   token={m.token.hex()} seq={self.sync_seq} "
                      f"ACK frame={ack.hex()}")
            self.sync_seq = (self.sync_seq + 1) & 0xFF
            asyncio.create_task(self._write(ack))      # ACK and KEEP listening
        elif m.sub == SyncMeta.HISTORY_COMPLETE:
            self._log(f"[SYNC] HistoryComplete — drained {self._sync_records} records, "
                      f"{self._sync_events} events. Done.")
            self.sync_complete = True
            if self.store:
                self.store.record_batch(None, None, self._sync_records, self._sync_events, True)

    # ── high-level flows ──
    async def get_battery(self):
        await self.send(Cmd.GET_BATTERY_LEVEL, b"")

    async def get_hello(self):
        await self.send(Cmd.GET_HELLO_HARVARD if self.gen == Gen.HARVARD else Cmd.GET_HELLO,
                        b"\x00" if self.gen == Gen.HARVARD else bytes([REVISION_1]))

    async def get_clock(self):
        await self.send(Cmd.GET_CLOCK, b"\x00")

    async def get_data_range(self):
        await self.send(Cmd.GET_DATA_RANGE, b"\x00")

    async def buzz(self, pattern: int = HAPTIC_SHORT_PULSE):
        await self.send(Cmd.RUN_HAPTICS_PATTERN, bytes([pattern, 0, 0, 0, 0]))

    async def enable_live_streams(self, hr=True, imu=True, optical=True,
                                  force_optical=False, persistent=False):
        """
        Turn on live sensor streams. THE LED BEHAVIOR IS THE IMPORTANT PART:

          • optical=True       sends ENABLE_OPTICAL_DATA (0x6B) only — opens the
                               PPG data path but lets the strap's WRIST-GATING
                               decide when the green LEDs actually run. The sensor
                               glows only while you're wearing it. (recommended)
          • force_optical=True adds TOGGLE_OPTICAL_MODE (0x6C) — forces the green
                               LEDs ON for the whole session, overriding wrist
                               detection. Guarantees HR even at rest, but the
                               sensor glows the entire time. Cleared on disconnect.
          • persistent=True    adds TOGGLE_PERSISTENT_R21 (0x9A) — *** DANGER ***:
                               the forced-on state survives reboots AND disconnects.
                               THIS is what makes the LED glow forever. Almost never
                               what you want; off by default now.
        """
        self._live_enabled = True
        if hr:
            await self.send(Cmd.TOGGLE_REALTIME_HR, b"\x01");      await asyncio.sleep(0.1)
        if imu:
            await self.send(Cmd.SEND_R10_R11_REALTIME, b"\x01");   await asyncio.sleep(0.1)
            await self.send(Cmd.TOGGLE_IMU_MODE, b"\x01");         await asyncio.sleep(0.1)
        if optical:
            await self.send(Cmd.ENABLE_OPTICAL_DATA, bytes([REVISION_1, 0x01])); await asyncio.sleep(0.1)
            if force_optical:
                await self.send(Cmd.TOGGLE_OPTICAL_MODE, bytes([REVISION_1, 0x01])); await asyncio.sleep(0.1)
            if persistent:   # DANGER — survives reboot; only if you really mean it
                await self.send(Cmd.TOGGLE_PERSISTENT_R21, bytes([REVISION_1, 0x01])); await asyncio.sleep(0.1)
        self._log(f"Live streams enabled (optical: "
                  f"{'FORCED-ON' if force_optical else 'wrist-gated'}"
                  f"{' + PERSISTENT' if persistent else ''}).")

    async def disable_live_streams(self):
        """
        Turn EVERYTHING off and CLEAR the persistent/forced-optical flags so the
        green LED returns to wrist-gated. Safe to call anytime (idempotent).
        """
        for op, pl in [
            (Cmd.TOGGLE_PERSISTENT_R21, bytes([REVISION_1, 0x00])),  # clear the dangerous flag
            (Cmd.TOGGLE_PERSISTENT_R20, bytes([REVISION_1, 0x00])),
            (Cmd.TOGGLE_OPTICAL_MODE,   bytes([REVISION_1, 0x00])),
            (Cmd.ENABLE_OPTICAL_DATA,   bytes([REVISION_1, 0x00])),
            (Cmd.SEND_R10_R11_REALTIME, b"\x00"),
            (Cmd.TOGGLE_IMU_MODE,       b"\x00"),
            (Cmd.TOGGLE_REALTIME_HR,    b"\x00"),
            (Cmd.STOP_RAW_DATA,         b"\x00"),
            (Cmd.EXIT_HIGH_FREQ_SYNC,   b"\x00"),
        ]:
            try:
                await self.send(op, pl); await asyncio.sleep(0.06)
            except Exception:
                pass
        self._live_enabled = False
        self._log("Live streams disabled — optical/persistent flags cleared.")

    async def calm(self, buzz: bool = True):
        """Recover a strap with a stuck/forever-on LED: clear all flags, then buzz."""
        await self.disable_live_streams()
        if buzz:
            try:
                await self.buzz()
            except Exception:
                pass

    async def reboot(self):
        """
        Hard-reset the strap (REBOOT_STRAP 0x1D). This is the documented fix for a
        stuck/forever-glowing optical LED: clearing the persistent flag (0x9A) only
        takes effect on a fresh boot, so the live optical engine keeps the LEDs lit
        until the strap restarts. We clear the flag first so it boots clean, then
        reboot. BLE drops; the strap re-advertises in ~10s.
        """
        await self.send(Cmd.TOGGLE_PERSISTENT_R21, bytes([REVISION_1, 0x00])); await asyncio.sleep(0.2)
        await self.send(Cmd.TOGGLE_OPTICAL_MODE,   bytes([REVISION_1, 0x00])); await asyncio.sleep(0.2)
        await self.send(Cmd.REBOOT_STRAP, b"\x00")
        self._log("REBOOT_STRAP sent — strap drops BLE and restarts (~10s), LED clears.")

    async def bond(self) -> bool:
        """
        Attempt a TRUE BLE bond (what makes the strap stop blinking + buzz the
        pairing ack). Works on Linux(BlueZ)/Windows/Android-style backends.
        On macOS this raises NotImplementedError — CoreBluetooth has no userspace
        pairing API; it only bonds implicitly when an encrypted characteristic is
        accessed, and WHOOP exposes none. Returns True on a confirmed bond.
        """
        try:
            ok = bool(await self.client.pair())
            self._log(f"bond: {'SUCCESS — watch for BLE_BONDED + haptic' if ok else 'returned False'}")
            return ok
        except Exception as e:
            self._log(f"bond NOT available on this OS: {type(e).__name__}: {e}")
            return False

    async def _heartbeat_loop(self, period=10.0):
        while True:
            await asyncio.sleep(period)
            try:
                await self.send(Cmd.LINK_VALID, b"\x00")
            except Exception:
                return

    async def _battery_loop(self, period=20.0):
        while True:
            await asyncio.sleep(period)
            try:
                await self.get_battery()
            except Exception:
                return

    def start_background_loops(self):
        self._tasks.append(asyncio.create_task(self._heartbeat_loop()))
        self._tasks.append(asyncio.create_task(self._battery_loop()))

    async def run_sync(self, timeout: float = 120.0) -> bool:
        """Full connect → INIT → drain history (ACK each batch) → return on complete/idle."""
        if not self.client or not self.client.is_connected:
            if not await self.connect():
                return False
        self.start_background_loops()
        await self.send_init()
        start = time.time()
        last = self._sync_records
        idle_since = time.time()
        while not self.sync_complete and (time.time() - start) < timeout:
            await asyncio.sleep(1.0)
            if self._sync_records != last:
                last = self._sync_records
                idle_since = time.time()
            elif time.time() - idle_since > 8.0:
                # stream went quiet without a HistoryComplete — nudge it.
                self._log("[SYNC] idle — sending ABORT_HISTORICAL to settle.")
                try:
                    await self.send(Cmd.ABORT_HISTORICAL_TRANSMITS, b"\x00")
                except Exception:
                    pass
                break
        return self.sync_complete

    async def disconnect(self):
        for t in self._tasks:
            t.cancel()
        if self._live_enabled:
            try:
                await self.disable_live_streams()
            except Exception:
                pass
        if self.client and self.client.is_connected:
            try:
                await self.client.disconnect()
            except Exception:
                pass
        self._log("Disconnected.")


# ════════════════════════════════════════════════════════════════════════════
# SECTION 10 — CHARGER / BATTERY PACK SUBSYSTEM
# ════════════════════════════════════════════════════════════════════════════
# The slide-on charging puck ("battery pack") is its own MCU. Over BLE the strap
# reports it via events (7/8 charging, 21/22 pack connected/removed, 109 serial)
# and commands (0x97 GET_BATTERY_PACK_INFO, 0x62 extended battery, 0x1A poll).
# Over USB the puck enumerates as a serial port; a dead pack is revived by the
# magic "Reboot" string (reverse-repos/Whoop4.0BatteryReset).

def reset_charger_via_serial(port: str, baud: int = 9600, magic: str = "Reboot",
                             wait: float = 1.0) -> str:
    """
    Revive a stuck/dead WHOOP 4.0 charging puck (battery pack).

    Connect the puck via USB; it shows up as a serial port (Windows: 'COMx' under
    Ports; Linux: '/dev/ttyUSB0' or '/dev/ttyACM0'; macOS: '/dev/tty.usbserial-*').
    We open it at 9600-8N1 and write the ASCII string "Reboot", which the puck's
    diagnostic bootloader interprets as a reset — the LED comes back and it
    charges again. Mechanism reproduced exactly from the C# COMSpike tool.

    Requires `pyserial` (`pip install pyserial`). Returns any bytes the puck echoed.
    """
    try:
        import serial  # type: ignore
    except Exception as e:  # pragma: no cover
        raise RuntimeError("pyserial required — `pip install pyserial`") from e
    sp = serial.Serial(port, baudrate=baud, bytesize=serial.EIGHTBITS,
                       parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                       timeout=wait)
    try:
        sp.write(magic.encode("ascii"))
        time.sleep(wait)
        echo = sp.read(sp.in_waiting or 64)
        return echo.decode("ascii", "replace")
    finally:
        sp.close()


# ════════════════════════════════════════════════════════════════════════════
# SECTION 11 — SELF-TEST  (no hardware: proves the protocol layer is correct)
# ════════════════════════════════════════════════════════════════════════════

def selftest() -> bool:
    ok = True
    def check(name, cond):
        nonlocal ok
        print(f"  [{'OK ' if cond else 'FAIL'}] {name}")
        ok = ok and cond

    # CRC8 over the size field of an 8-byte frame must be 0xA8; of a 16-byte ACK 0x57.
    check("crc8([08,00]) == 0xA8", crc8(b"\x08\x00") == 0xA8)
    check("crc8([10,00]) == 0x57", crc8(b"\x10\x00") == 0x57)

    # Regenerating the INIT packets from build_command must match the HCI-snoop bytes.
    regen = [
        build_command(0, Cmd.GET_HELLO_HARVARD, b"\x00"),
        build_command(1, Cmd.GET_ADVERTISING_NAME_HARVARD, b"\x00"),
        build_command(2, Cmd.GET_DATA_RANGE, b"\x00"),
        build_command(3, Cmd.GET_ALARM_TIME, b"\x01"),
        build_command(4, Cmd.SEND_HISTORICAL_DATA, b"\x00"),
    ]
    for i, (a, b) in enumerate(zip(regen, INIT_PACKETS)):
        check(f"INIT[{i}] regenerated == snoop ({b.hex()})", a == b)

    # Frame round-trip.
    f = parse_frame(build_command(0xA0, Cmd.LINK_VALID, b"\x00"))
    check("link_valid round-trips", f is not None and f.crc8_ok and f.crc32_ok
          and f.packet_type == 0x23 and f.opcode == 0x01)

    # Batch ACK must reproduce the observed 'aa1000 57 23 .. 1701 ..' prefix.
    ack = build_batch_ack(5, bytes.fromhex("1122334455667788"))
    check("batch ACK prefix aa10005723", ack[:5].hex() == "aa1000" + "5723"[:4]
          and ack[:4].hex() == "aa100057" and ack[4] == 0x23 and ack[6] == 0x17)
    check("batch ACK length == 20", len(ack) == 20)

    # Reassembler splits two concatenated frames + null padding.
    two = build_command(0xA0, Cmd.LINK_VALID) + b"\x00\x00" + build_command(0xA1, Cmd.GET_BATTERY_LEVEL, b"")
    frames = FrameReassembler().feed(two)
    check("reassembler splits 2 frames", len(frames) == 2)

    # Event decode.
    ev_inner = bytes([0x30, 0x01]) + struct.pack("<H", Event.WRIST_ON) + struct.pack("<I", 1700000000) + b"\x00\x00\x00\x00"
    ev = parse_event(ev_inner)
    check("event WRIST_ON decodes", ev is not None and ev.name == "WRIST_ON")

    # Analytics sanity.
    check("spo2_from_ratio clamps", spo2_from_ratio(0.5) == 97.5 and spo2_from_ratio(2.0) == 85.0)
    check("rmssd basic", rmssd([800, 820, 810, 830]) is not None)

    print(f"\nSelf-test: {'ALL PASS ✓' if ok else 'FAILURES ✗'}")
    return ok


# ════════════════════════════════════════════════════════════════════════════
# SECTION 12 — CLI
# ════════════════════════════════════════════════════════════════════════════

def _print_decoded(d: dict):
    kind = d.get("kind", "?")
    if kind == "event":
        extra = {k: v for k, v in d.items() if k in ("battery_pct_tentative", "charging",
                 "on_wrist", "pack_connected", "skin_temp_c", "puck_serial")}
        print(f"[EVENT] {d.get('event')}  {extra or ''}")
    elif kind == "cmd_response":
        print(f"[RESP ] {d.get('opcode')}  "
              + json.dumps({k: v for k, v in d.items() if k not in
                            ('kind', 'opcode', 'packet_type', 'packet_name', 'seq', 'crc_ok')})[:200])
    elif kind in ("R10", "R21", "R24_recovery", "R25_rr", "R2", "realtime_hr", "comprehensive_0x5c", "r17_rr"):
        print(f"[DATA ] {kind}: "
              + json.dumps({k: v for k, v in d.items() if k not in ('kind', 'packet_type', 'packet_name', 'seq', 'crc_ok')})[:200])
    elif kind == "metadata":
        print(f"[META ] {d.get('sub')} token={d.get('token')}")


async def _cli_live(args, do_sync: bool, do_stream: bool):
    store = None if args.no_store else WhoopStore(
        db_path=args.db, capture_path=args.capture)
    client = WhoopClient(address=args.address, store=store,
                         on_decode=_print_decoded if args.verbose else None,
                         verbose=args.verbose)
    try:
        if not await client.connect():
            return
        await client.get_hello()
        await asyncio.sleep(0.5)
        # Align the strap RTC to real time so records carry correct unix timestamps
        # (the band ships with an unset clock that drifts to bogus dates).
        await client.send(Cmd.SET_CLOCK, struct.pack("<II", int(time.time()), 0))
        await asyncio.sleep(0.3)
        if do_sync:
            await client.run_sync(timeout=args.timeout)
        if do_stream:
            await client.enable_live_streams(force_optical=getattr(args, "force_optical", False))
            print(f"Streaming for {args.duration}s (Ctrl-C to stop)…")
            await asyncio.sleep(args.duration)
        elif not do_sync:
            client.start_background_loops()
            print(f"Monitoring events for {args.duration}s (Ctrl-C to stop)…")
            await asyncio.sleep(args.duration)
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    finally:
        await client.disconnect()
        if store:
            store.close()


async def _cli_oneshot(args, build_fn):
    client = WhoopClient(address=args.address, verbose=args.verbose)
    try:
        if not await client.connect():
            return
        await build_fn(client)
        await asyncio.sleep(args.duration)
    finally:
        await client.disconnect()


def main(argv=None):
    p = argparse.ArgumentParser(
        description="Complete WHOOP 4.0 BLE client + protocol toolkit.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--address", "-a", help="BLE MAC/UUID (otherwise scan)")
    p.add_argument("--db", default="whoop.db", help="SQLite path (default whoop.db)")
    p.add_argument("--capture", default="whoop_capture.jsonl", help="raw capture JSONL path")
    p.add_argument("--no-store", action="store_true", help="don't persist anything")
    p.add_argument("--duration", type=float, default=60.0, help="seconds to run (default 60)")
    p.add_argument("--timeout", type=float, default=120.0, help="sync timeout (default 120)")
    p.add_argument("--quiet", dest="verbose", action="store_false", help="less output")
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("selftest", help="run protocol self-tests (no hardware)")
    sub.add_parser("scan", help="scan for nearby WHOOP straps")
    sub.add_parser("info", help="connect + print HELLO identity, then exit")
    sub.add_parser("monitor", help="connect + stream system events (battery/wrist/charge/…)")
    sub.add_parser("sync", help="drain historical flash with correct batch ACKs")
    lv = sub.add_parser("live", help="enable HR + IMU + optical live streams (wrist-gated)")
    lv.add_argument("--force-optical", action="store_true",
                    help="force green LEDs ON the whole session (overrides wrist-gating)")
    sub.add_parser("off", help="clear stuck/forced optical flags (fix a forever-on LED) + buzz")
    sub.add_parser("reboot", help="hard-reset the strap — fixes a stuck/glowing LED a plain 'off' can't")
    sub.add_parser("pair", help="attempt a TRUE BLE bond — Linux/Android only (macOS can't)")
    sub.add_parser("battery", help="poll battery once")
    sub.add_parser("clock", help="read the strap's RTC (GET_CLOCK) + compare to live record ts vs system time")
    scp = sub.add_parser("set-clock", help="SET_CLOCK the strap to real time, then verify GET_CLOCK + fresh record ts")
    scp.add_argument("--epoch", type=int, default=None, help="unix epoch to set (default: system now)")
    sub.add_parser("labrador", help="enable the Labrador filtered pipeline (R17) to probe for RR-intervals/HRV, then clean up")
    rn = sub.add_parser("rename", help="set the strap advertising name (SET_ADVERTISING_NAME_HARVARD)")
    rn.add_argument("name", help="new strap name (≤20 ASCII chars)")
    hp = sub.add_parser("haptic", help="buzz the strap")
    hp.add_argument("--pattern", type=int, default=HAPTIC_SHORT_PULSE)
    al = sub.add_parser("alarm", help="set a smart alarm at an epoch time")
    al.add_argument("epoch", type=int, help="unix epoch seconds to fire")

    dec = sub.add_parser("decode", help="decode a single hex frame (no hardware)")
    dec.add_argument("hex", help="hex string of one frame")
    rp = sub.add_parser("replay", help="re-decode a capture file (no hardware)")
    rp.add_argument("path", help="JSONL capture or hex-per-line dump")
    rp.add_argument("--gen", type=int, default=4, choices=[3, 4, 5])

    rc = sub.add_parser("reset-charger", help="revive a dead battery puck over serial")
    rc.add_argument("port", help="serial port (COMx / /dev/ttyUSB0 / /dev/tty.usbserial-*)")

    cat = sub.add_parser("catalog", help="print the command / event / record catalogs")

    args = p.parse_args(argv)
    cmd = args.cmd

    if cmd == "selftest":
        sys.exit(0 if selftest() else 1)

    if cmd == "decode":
        fr = parse_frame(bytes.fromhex(args.hex.replace(" ", "")))
        if not fr:
            print("Not a valid frame."); return
        print(json.dumps(decode_frame(fr), indent=2))
        return

    if cmd == "replay":
        n = {"count": 0}
        def cb(d):
            n["count"] += 1
            _print_decoded(d)
        results = replay_capture(args.path, gen=Gen(args.gen), on_decode=cb)
        kinds: dict[str, int] = {}
        for d in results:
            kinds[d.get("kind", "?")] = kinds.get(d.get("kind", "?"), 0) + 1
        print(f"\nDecoded {len(results)} frames: {kinds}")
        return

    if cmd == "reset-charger":
        print(f"Writing 'Reboot' to {args.port} @ 9600-8N1…")
        echo = reset_charger_via_serial(args.port)
        print(f"Puck replied: {echo!r}\nDone — check the puck LED.")
        return

    if cmd == "catalog":
        print("── COMMANDS ──")
        for c in Cmd:
            flag = " ⚠" if c in DANGEROUS_CMDS else ("  (2-byte payload)" if c in TWO_BYTE_TOGGLES else "")
            print(f"  0x{int(c):02X}  {c.name}{flag}")
        print("\n── EVENTS ──")
        for e in Event:
            print(f"  {int(e):3d}  {e.name}")
        print("\n── RECORD TYPES ──")
        for r in Record:
            print(f"  {int(r):3d}  {r.name}")
        return

    # ── live (hardware) subcommands ──
    if not _HAVE_BLEAK:
        print("This subcommand needs bleak: pip install bleak")
        return

    if cmd == "scan":
        async def _scan():
            c = WhoopClient(verbose=True)
            await c.scan()
        asyncio.run(_scan()); return
    if cmd == "info":
        asyncio.run(_cli_oneshot(args, lambda c: c.get_hello())); return
    if cmd == "battery":
        asyncio.run(_cli_oneshot(args, lambda c: c.get_battery())); return
    if cmd == "clock":
        async def _clock(a):
            import time as _t
            cap: dict = {}
            ts_seen: list = []
            def on_dec(d):
                if d.get("kind") == "cmd_response" and "epoch" in d:
                    cap["band"] = d["epoch"]
                # collect any record timestamps we see (realtime/historical)
                for k in ("ts_epoch", "ts"):
                    if isinstance(d.get(k), int) and d[k] > 0:
                        ts_seen.append((d.get("kind"), d[k]))
                _print_decoded(d)
            c = WhoopClient(address=a.address, verbose=a.verbose, on_decode=on_dec)
            if not await c.connect():
                return
            sys_now = int(_t.time())
            print(f"\n[clock] SYSTEM unix now = {sys_now}  "
                  f"({_t.strftime('%Y-%m-%d %H:%M:%S', _t.gmtime(sys_now))} UTC)")
            await c.get_clock()
            await asyncio.sleep(1.5)
            await c.get_data_range()
            await asyncio.sleep(1.0)
            # grab a few live records so we can compare their ts to the band RTC
            print("[clock] enabling HR + R10 live for 6s to sample record timestamps…")
            await c.send(Cmd.TOGGLE_REALTIME_HR, b"\x01")
            await c.send(Cmd.SEND_R10_R11_REALTIME, b"\x01")
            await asyncio.sleep(6.0)
            await c.send(Cmd.TOGGLE_REALTIME_HR, b"\x00")
            await c.send(Cmd.SEND_R10_R11_REALTIME, b"\x00")
            print("\n────────── CLOCK SUMMARY ──────────")
            print(f"[clock] system now      : {sys_now}")
            if "band" in cap:
                b = cap["band"]
                plausible = 1_000_000_000 < b < 4_000_000_000
                print(f"[clock] band GET_CLOCK   : {b}   (delta {b - sys_now:+d}s)")
                print(f"[clock] band as UTC      : "
                      + (_t.strftime('%Y-%m-%d %H:%M:%S', _t.gmtime(b)) if plausible
                         else f"NOT a unix time → RTC unset / relative ({b/86400:.1f} days)"))
            else:
                print("[clock] band GET_CLOCK   : (no response)")
            if ts_seen:
                print("[clock] sample record ts :")
                for kind, t in ts_seen[:8]:
                    pl = 1_000_000_000 < t < 4_000_000_000
                    print(f"          {kind:14s} ts={t}  "
                          + ("unix→ " + _t.strftime('%Y-%m-%d %H:%M:%S', _t.gmtime(t)) if pl
                             else f"NOT unix ({t/86400:.1f} days)"))
            else:
                print("[clock] sample record ts : (none captured — band may be off-wrist/quiet)")
            print("───────────────────────────────────")
            await c.disconnect()
        asyncio.run(_clock(args)); return
    if cmd == "rename":
        async def _rn(a):
            c = WhoopClient(address=a.address, verbose=a.verbose)
            if not await c.connect():
                return
            await c.send(Cmd.GET_ADVERTISING_NAME_HARVARD, b"\x00")
            await asyncio.sleep(1.0)
            print(f"[rename] setting name → {a.name!r}")
            await c.send(Cmd.SET_ADVERTISING_NAME_HARVARD,
                         bytes([0x01, len(a.name.encode('ascii', 'ignore')[:20])])
                         + a.name.encode('ascii', 'ignore')[:20] + b"\x00\x00\x00\x00")
            await asyncio.sleep(1.0)
            await c.send(Cmd.GET_ADVERTISING_NAME_HARVARD, b"\x00")  # read back
            await asyncio.sleep(1.5)
            await c.disconnect()
        asyncio.run(_rn(args)); return
    if cmd == "labrador":
        async def _lab(a):
            import time as _t
            tally = {"r17": 0, "rr": []}
            def on_dec(d):
                if d.get("kind") == "r17_rr":
                    tally["r17"] += 1
                    tally["rr"].extend(d.get("rr_raw", []))
                _print_decoded(d)
            c = WhoopClient(address=a.address, verbose=a.verbose, on_decode=on_dec)
            if not await c.connect():
                return
            await c.send(Cmd.SET_CLOCK, struct.pack("<II", int(_t.time()), 0))
            await asyncio.sleep(0.3)
            # Enable optical (PPG must run) + the Labrador filtered pipeline that
            # produces R17 (0x2B rec_type 17) — the RR/HRV carrier.
            print("[labrador] enabling optical + Labrador data-generation + filtered…")
            await c.send(Cmd.TOGGLE_REALTIME_HR, b"\x01")
            await c.send(Cmd.SEND_R10_R11_REALTIME, b"\x01")
            await c.send(Cmd.ENABLE_OPTICAL_DATA, bytes([REVISION_1, 0x01]))
            await asyncio.sleep(0.2)
            await c.send(Cmd.TOGGLE_OPTICAL_MODE, bytes([REVISION_1, 0x01]))
            await asyncio.sleep(0.2)
            # TOGGLE_LABRADOR_DATA_GENERATION=124(0x7C), RAW_SAVE=125(0x7D), FILTERED=139(0x8B).
            # Enable generation + raw-save + filtered (R17 may be SAVED to flash and
            # delivered via historical sync rather than streamed live).
            for opc in (0x7C, 0x7D, 0x8B):
                await c.send(opc, b"\x01")
                await asyncio.sleep(0.15)
                await c.send(opc, bytes([REVISION_1, 0x01]))
                await asyncio.sleep(0.15)
            secs = int(a.duration) if a.duration else 40
            print(f"[labrador] generating {secs}s (wear the band, stay still)…")
            await asyncio.sleep(secs)
            # Now drain history — flash-saved Labrador/R17 records come through here.
            print("[labrador] draining history to pull any flash-saved R17…")
            try:
                await c.run_sync(timeout=60)
            except Exception as e:
                print(f"[labrador] sync error: {e}")
            # Clean up: turn Labrador + optical OFF so the LED doesn't stay on.
            print("[labrador] disabling Labrador + optical…")
            for opc in (0x8B, 0x7D, 0x7C):
                await c.send(opc, b"\x00")
                await c.send(opc, bytes([REVISION_1, 0x00]))
                await asyncio.sleep(0.1)
            await c.calm()
            await asyncio.sleep(1.0)
            print("\n────────── LABRADOR RESULT ──────────")
            print(f"[labrador] R17 packets seen : {tally['r17']}")
            if tally["rr"]:
                rr = tally["rr"][:20]
                print(f"[labrador] RR samples (raw)  : {rr}")
                print(f"[labrador] RR range          : {min(tally['rr'])}–{max(tally['rr'])}")
                print("  → if ~600–1200 it's MILLISECONDS (HR 50–100); if ~700–1400 & HR-implausible, it's 1/1024s ticks (×1000/1024).")
            else:
                print("[labrador] no R17/RR seen — Labrador toggles may need a different payload/sequence, or wrist contact.")
            print("──────────────────────────────────────")
            await c.disconnect()
        asyncio.run(_lab(args)); return
    if cmd == "set-clock":
        async def _setclock(a):
            import time as _t
            cap: dict = {}
            ts_seen: list = []
            def on_dec(d):
                if d.get("kind") == "cmd_response" and "epoch" in d:
                    cap.setdefault("clocks", []).append(d["epoch"])
                for k in ("ts_epoch", "ts"):
                    if isinstance(d.get(k), int) and d[k] > 0:
                        ts_seen.append((d.get("kind"), d[k]))
                _print_decoded(d)
            c = WhoopClient(address=a.address, verbose=a.verbose, on_decode=on_dec)
            if not await c.connect():
                return
            target = a.epoch if a.epoch else int(_t.time())
            print(f"\n[set-clock] BEFORE — reading current RTC…")
            await c.get_clock()
            await asyncio.sleep(1.5)
            print(f"[set-clock] SET_CLOCK → {target} "
                  f"({_t.strftime('%Y-%m-%d %H:%M:%S', _t.gmtime(target))} UTC)")
            await c.send(Cmd.SET_CLOCK, struct.pack("<II", target, 0))
            await asyncio.sleep(1.5)
            print("[set-clock] AFTER — reading RTC back…")
            await c.get_clock()
            await asyncio.sleep(1.5)
            print("[set-clock] sampling fresh live record ts for 6s…")
            await c.send(Cmd.TOGGLE_REALTIME_HR, b"\x01")
            await c.send(Cmd.SEND_R10_R11_REALTIME, b"\x01")
            await asyncio.sleep(6.0)
            await c.send(Cmd.TOGGLE_REALTIME_HR, b"\x00")
            await c.send(Cmd.SEND_R10_R11_REALTIME, b"\x00")
            print("\n────────── SET-CLOCK RESULT ──────────")
            clocks = cap.get("clocks", [])
            if len(clocks) >= 1:
                print(f"[set-clock] RTC before : {clocks[0]}")
            if len(clocks) >= 2:
                b = clocks[-1]
                pl = 1_000_000_000 < b < 4_000_000_000
                print(f"[set-clock] RTC after  : {b}  (target {target}, delta {b - target:+d}s) "
                      + ("✓ now real time" if pl and abs(b - target) < 120 else "✗ did not take"))
            fresh = [t for kind, t in ts_seen if kind in ("realtime_hr", "realtime", "data", "record")]
            for kind, t in ts_seen[-8:]:
                pl = 1_000_000_000 < t < 4_000_000_000
                print(f"[set-clock] rec {kind:12s} ts={t}  "
                      + ("UNIX ✓ " + _t.strftime('%H:%M:%S', _t.gmtime(t)) if pl
                         else f"still relative ({t/86400:.1f}d)"))
            print("──────────────────────────────────────")
            print("If RTC after ≈ target AND record ts are UNIX → SET_CLOCK is the fix.")
            print("If record ts stay relative → records use a boot counter; we align via offset.")
            await c.disconnect()
        asyncio.run(_setclock(args)); return
    if cmd == "haptic":
        asyncio.run(_cli_oneshot(args, lambda c: c.buzz(args.pattern))); return
    if cmd == "alarm":
        asyncio.run(_cli_oneshot(args, lambda c: c.send(Cmd.SET_ALARM_TIME,
                    b"\x01" + struct.pack("<I", args.epoch) + b"\x00\x00"))); return
    if cmd == "monitor":
        asyncio.run(_cli_live(args, do_sync=False, do_stream=False)); return
    if cmd == "sync":
        asyncio.run(_cli_live(args, do_sync=True, do_stream=False)); return
    if cmd == "live":
        asyncio.run(_cli_live(args, do_sync=False, do_stream=True)); return
    if cmd == "off":
        async def _off(a):
            c = WhoopClient(address=a.address, verbose=True)
            if await c.connect():
                await c.calm()
                await asyncio.sleep(1.5)
                await c.disconnect()
        asyncio.run(_off(args)); return
    if cmd == "reboot":
        async def _rb(a):
            c = WhoopClient(address=a.address, verbose=True)
            if await c.connect():
                await c.reboot()
                await asyncio.sleep(1.0)
                try:
                    await c.disconnect()
                except Exception:
                    pass
        asyncio.run(_rb(args)); return
    if cmd == "pair":
        async def _pair(a):
            import platform
            c = WhoopClient(address=a.address, verbose=True)
            if not await c.connect():
                return
            print("\nPut the strap in PAIRING MODE first (off-wrist, double-tap until it blinks),")
            print("then attempting a true BLE bond…")
            ok = await c.bond()
            await asyncio.sleep(3.0)   # let BLE_BONDED + the pairing haptic land
            if not ok and platform.system() == "Darwin":
                print("\n→ macOS cannot initiate BLE pairing (CoreBluetooth has no userspace API).")
                print("  Connect + stream still work UNBONDED. For a real bond (blue light off +")
                print("  pairing haptic), run this `pair` command from Linux or use Android.")
            await c.disconnect()
        asyncio.run(_pair(args)); return


if __name__ == "__main__":
    main()
