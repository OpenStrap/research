#!/usr/bin/env python3
"""Decode + analyze WHOOP events to crack the unknown IDs (68, 69, 102, 103).

Event frame: [0]=0x30 [1]=seq [2:4]=event_id u16 LE [4:8]=ts u32 LE
             [8:12]=subsec u32 LE [12:]=payload
"""
import json
import struct
from collections import defaultdict

# Authoritative names from the device enum. Unknowns left blank.
NAMES = {
    1: "ERROR", 3: "BATTERY_LEVEL", 7: "CHARGING_ON", 8: "CHARGING_OFF",
    9: "WRIST_ON", 10: "WRIST_OFF", 11: "BLE_CONNECTION_UP", 12: "BLE_CONNECTION_DOWN",
    14: "DOUBLE_TAP", 18: "PAIRING_MODE", 21: "BATTERY_PACK_CONNECTED",
    22: "BATTERY_PACK_REMOVED", 23: "BLE_BONDED", 29: "STRAP_CONDITION_REPORT",
    32: "CAPTOUCH_AUTOTHRESHOLD", 33: "BLE_REALTIME_HR_ON", 36: "AFE_RESET",
    44: "BLE_SYSTEM_ON", 45: "BLE_SYSTEM_INITIALIZED", 60: "HAPTICS_FIRED",
    63: "EXTENDED_BATTERY_INFO", 100: "HAPTICS_TERMINATED",
    102: "PPG_SEARCH_ON?", 103: "PPG_SEARCH_OFF?",  # empirical, not in enum
}


def decode(hexstr):
    b = bytes.fromhex(hexstr)
    return {
        "id": struct.unpack_from("<H", b, 2)[0],
        "ts": struct.unpack_from("<I", b, 4)[0],
        "subsec": struct.unpack_from("<I", b, 8)[0] if len(b) >= 12 else None,
        "payload": b[12:],
    }


def main():
    evs = [dict(decode(e["hex"]), event_id=e["event_id"]) for e in json.load(open("/tmp/events.json"))]
    evs.sort(key=lambda e: e["ts"])

    # 1) Payload signatures per event id
    print("=" * 70)
    print("PAYLOAD SIGNATURES (per event id)")
    print("=" * 70)
    by_id = defaultdict(list)
    for e in evs:
        by_id[e["id"]].append(e["payload"])
    for eid in sorted(by_id):
        pls = by_id[eid]
        # per-byte: constant or varying
        maxlen = max(len(p) for p in pls)
        cols = []
        for i in range(maxlen):
            vals = {p[i] for p in pls if len(p) > i}
            cols.append(f"{next(iter(vals)):02x}" if len(vals) == 1 else f"[{'/'.join(f'{v:02x}' for v in sorted(vals))}]")
        print(f"  {eid:3d} {NAMES.get(eid,'?'):24s} n={len(pls):3d} payload= {' '.join(cols)}")

    # 2) Timeline around the unknowns — do they pair / bracket known events?
    print("\n" + "=" * 70)
    print("TIMELINE (unknowns 68/69/102/103 in context, first 60 events)")
    print("=" * 70)
    for e in evs[:60]:
        mark = "  <-- UNKNOWN" if e["id"] in (68, 69, 102, 103) else ""
        print(f"  ts={e['ts']:>11} {e['id']:3d} {NAMES.get(e['id'],'?'):22s} "
              f"pl={e['payload'].hex():10s}{mark}")

    # 3) Pairing analysis: gap between consecutive 68->69 and 102->103
    print("\n" + "=" * 70)
    print("ON/OFF PAIRING (does X turn on then off?)")
    print("=" * 70)
    for on, off in [(102, 103), (68, 69), (7, 8), (9, 10), (11, 12)]:
        seq = [e for e in evs if e["id"] in (on, off)]
        durs = []
        last_on = None
        for e in seq:
            if e["id"] == on:
                last_on = e["ts"]
            elif e["id"] == off and last_on is not None:
                durs.append(e["ts"] - last_on)
                last_on = None
        if durs:
            print(f"  {on}->{off}: {len(durs)} pairs, durations(s)={durs[:10]} "
                  f"avg={sum(durs)//len(durs)}s")
        else:
            print(f"  {on}->{off}: no clean pairs")


if __name__ == "__main__":
    main()
