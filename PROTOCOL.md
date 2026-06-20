# WHOOP 4.0 BLE — protocol reference

The byte-level reference for talking to a WHOOP 4.0 band ("Gen 4"), and a map of how
`research_playground.py` implements it. Confidence is marked throughout: **verified**
= confirmed on real hardware; **empirical** = inferred by correlation, plausible but
not certain; **unknown** = not determined.

> Not affiliated with or endorsed by WHOOP. For interoperability with a device you own.

The band is a **sensor pipe** — it does no analytics on-device. It records raw +
lightly-processed sensor records to flash and streams/relays them to a phone. The
scores (recovery/strain/sleep) are computed off-device in the cloud and are **not**
available here.

---

## 1. BLE transport & framing

### 1.1 Connecting
- **Scan must be service-filtered** by the service UUID below — a passive scan hides
  the UUID and name, so the band shows as anonymous.
- BLE is **one-central**: any other app holding the band (e.g. the official one) must
  be disconnected first or the band won't advertise to you.
- macOS can connect + stream **unbonded**; a true bond (pairing haptic, solid LED)
  needs a platform that can initiate pairing (Linux/BlueZ, or Android `createBond`).

### 1.2 GATT service (Gen 4)
- Service:            `61080001-8d6d-82b8-614a-1c8cb0f8dcc6`
- cmd → strap (write):    `61080002-…`
- cmd ← strap (notify):   `61080003-…`
- events ← strap (notify):`61080004-…`
- data ← strap (notify):  `61080005-…`
- memfault (notify):      `61080007-…`

(Other device families use different services; this reference is Gen 4 / `6108…` only.)

### 1.3 Frame envelope
```
[0xAA SOF][u16 LE size][CRC8(size bytes, poly 0x07)][inner, padded to /4][u32 LE CRC32 over padded inner]
size = len(inner_padded) + 4   # includes the trailing CRC32
```
- CRC-8 covers the two size bytes; CRC-32 (zlib) covers the padded inner.
- **The reassembler must be length-based, not "reset on 0xAA"** — sensor payloads
  contain `0xAA` bytes and BLE chunk splits land on them. Append chunks, extract by the
  declared length at true boundaries, resync to the next SOF only if the head isn't a
  valid frame, and drop CRC-failed frames. (`research_playground.py:FrameReassembler`.)

### 1.4 Inner packet
```
inner = [packet_type @0][seq @1][opcode | event_id | record_type @2][body…]
```

### 1.5 Packet types (`inner[0]`)
| Byte | Name | Notes |
|---|---|---|
| `0x23` | COMMAND | phone → strap |
| `0x24` | COMMAND_RESPONSE | strap → phone |
| `0x28` | REALTIME_DATA | small live records |
| `0x2B` | REALTIME_RAW_DATA | live R10 (HR + IMU) |
| `0x2F` | HISTORICAL_DATA | flash-drain records (type-24) |
| `0x30` | EVENT | discrete events |
| `0x31` | METADATA | sync markers (HistoryStart/End/Complete) |
| `0x32` | CONSOLE_LOGS | firmware logs |
| `0x33` / `0x34` | REALTIME / HISTORICAL IMU stream | |

All **verified**.

---

## 2. Command set (sent inside a `0x23` COMMAND; opcode at `inner[2]`)

Full list in `research_playground.py:Cmd`. Key ones (all verified unless noted):

| Opcode | Name | Use |
|---|---|---|
| `0x23` | GET_HELLO_HARVARD | identity / battery / wrist |
| `0x1A` | GET_BATTERY_LEVEL | authoritative battery: resp `[5:7]` u16 LE ÷10 = % |
| `0x16` | SEND_HISTORICAL_DATA | start the flash drain |
| `0x14` | ABORT_HISTORICAL_TRANSMITS | stop the drain cleanly |
| `0x17` | HISTORICAL_DATA_RESULT | the batch ACK (see §4) |
| `0x21` | SET_READ_POINTER | u32 offset — rewind/seek the read cursor |
| `0x22` | GET_DATA_RANGE | backlog window |
| `0x03` | TOGGLE_REALTIME_HR | live HR |
| `0x3F` | SEND_R10_R11_REALTIME | live R10/R11 (HR + IMU) |
| `0x6A` | TOGGLE_IMU_MODE | live IMU |
| `0x6B` | ENABLE_OPTICAL_DATA | wrist-gated optical (the HR source) |
| `0x6C` | TOGGLE_OPTICAL_MODE | forced optical |
| `0x0A` | SET_CLOCK | set RTC: `[u32 epoch LE][u32 0]` |
| `0x1A`/`0x62`/`0x97` | battery / extended battery / pack info | charge state |
| `0x19` | FORCE_TRIM | **flash erase — destructive; never sent** |
| `0x1D` | REBOOT_STRAP | hard reset (manual recovery only) |
| `0x9A` | TOGGLE_PERSISTENT_R21 | **footgun**: forces optical across reboots → stuck LED |

**Stuck-LED footgun:** `0x9A` forces the optical engine on persistently; clearing the
flag isn't enough (the running engine stays lit until a fresh boot). Recovery: turn
streams off, then `REBOOT_STRAP`. The client defaults to wrist-gated optical (`0x6B`
only) and never sends `0x9A` casually.

---

## 3. Session init

A short opening sequence (regenerable byte-for-byte by `build_command`):
```
GET_HELLO_HARVARD → GET_ADVERTISING_NAME → GET_DATA_RANGE → GET_ALARM_TIME → SEND_HISTORICAL_DATA
```
The final `SEND_HISTORICAL_DATA` triggers the drain: the band floods `0x2F` historical
records punctuated by `0x31` metadata markers, then waits for ACKs.

---

## 4. Historical sync (the careful path) — **verified**

### 4.1 Metadata markers (`0x31`, sub-type at `inner[2]`)
- `1 = HISTORY_START` → informational; ignore (a burst opened).
- `2 = HISTORY_END` → **ACK it and keep listening** (burst closed; advance cursor).
- `3 = HISTORY_COMPLETE` → backlog drained; **stop** (do not ACK).

### 4.2 The ACK
The END marker carries an 8-byte continuation **token**:
- `inner[9:13]` = a record-id / batch counter (not echoed)
- `inner[13:17]` + `inner[17:21]` → **token = `inner[13:21]`**

The ACK sent back (`research_playground.py:build_batch_ack`):
```
inner = [0x23 COMMAND][seq][0x17 HISTORICAL_DATA_RESULT][0x01 SUCCESS] + token(8B)   # 12 bytes
```
`0x01` = SUCCESS (0 = FAILURE, 2 = PENDING, 3 = UNSUPPORTED).

### 4.3 Behaviour (observed live)
- ACK advances the read cursor (token's first word += 5 per batch); the band replies
  with a `0x24 HISTORICAL_DATA_RESULT` ACK-of-ACK.
- The cursor is **persistent across connections**; not ACKing leaves it where it was.
- Draining is **non-destructive** (we never erase) and the cursor can be repositioned
  with `SET_READ_POINTER`. Stop conditions: HISTORY_COMPLETE, an idle timeout (~8 s),
  or catching up to the live edge.

---

## 5. The 1 Hz record (type-24) — 96 bytes — the main payload

Drained from flash via §4; one per second of wear. The **header + HR are verified**;
the sensor block is **raw/RELATIVE** (the band relays it uncalibrated and the cloud
derives %SpO₂/°C). Offsets below were re-validated by **per-byte variance analysis**
across **811 real records** (550 golden capture + 261 R2 spanning 113 h) and
cross-checked against two independent decoders (contributor/wearable; reference implementation
V24). A field is listed only if it actually VARIES like a sensor.

| Bytes | Field | Confidence |
|---|---|---|
| `[0]` | packet type `0x2F` | verified |
| `[1]` | record type = 24 | verified |
| `[2]` | sub-field (const `0x05`) | verified |
| `[3:7]` | u32 record counter (+1/record) | verified |
| `[7:11]` | **u32 UNIX timestamp (s)** | verified |
| `[11:13]` | u16 sub-seconds | verified |
| `[17]` | **heart rate (bpm)**; 0 = no reading | verified |
| `[18]` + `[19:19+2n]` | rr_count (u8) + R-R intervals (i16 LE, ms) | verified (HRV source) |
| `[29]` | u16 raw green PPG ADC | validated (varies) |
| `[31]` | u16 raw red/IR PPG ADC | validated (varies) |
| `[36:48]` | tri-axial accel (g), float32 ×3 | verified |
| `[51]` | u8 skin-contact **quality** (0–198) — NOT a wear flag | validated (varies) |
| `[52:64]` | f32 ×3 — **byte-identical mirror of `[36:48]`** (not an extra sensor) | validated (=accel) |
| `[64]` | u16 raw **red** ADC — RELATIVE | validated (varies) |
| `[66]` | u16 raw **IR** ADC — RELATIVE (pairs with red → SpO₂ ratio) | validated (varies) |
| `[68]` | u16 raw **skin-temp** ADC — RELATIVE (°C in cloud) | validated (varies) |
| `[70]` | u16 raw **ambient-light** ADC — RELATIVE | validated (varies) |
| `[72]`/`[74]` | u16 LED-drive current (config; near-constant) | config, not biometric |
| `[76]` | ~~resp_rate_raw~~ — **bit-constant 3073** across all 811 records | **rejected** (fixed trailer) |
| `[78]` | ~~signal_quality~~ — **bit-constant 3074** | **rejected** (fixed trailer) |
| `[88]` | u8 resting/baseline HR (held value, varies slowly, ≠ live HR) | empirical |

> ⚠️ **Corrected 2026-06-20.** Earlier revisions read SpO₂ as u8 `[72]` and temp as
> `[70]/4 °C`; both were **misidentified** — `[72]` is LED-drive config and `[70]` is the
> ambient-light ADC. The real raw ADCs are red `[64]` / IR `[66]` / temp `[68]` / ambient
> `[70]`, all u16. Respiration is **not** in the record (the `resp_rate_raw[76]` other
> projects list is bit-constant here); WHOOP derives it in-cloud from PPG, which the
> backend mirrors in `resp.ts`. The full payload is still preserved verbatim in
> `parse_r24().raw_tail` so records can be re-decoded as the map improves.

> Decoder: `research_playground.py:parse_r24`. Historical flash typically holds only
> type-24; raw R10/R21 records are **live-stream only**.

---

## 6. Live streaming (foreground)

- Enable with `SEND_R10_R11_REALTIME (0x3F)` + `ENABLE_OPTICAL_DATA (0x6B)` (+ IMU `0x6A`).
- **Live R10 (HR + IMU)** arrives as `0x2B` REALTIME_RAW_DATA, record type `0x0A` (10),
  ~1.9 KB across several BLE notifications. A compact `0x28` record also carries HR.
  Both decode paths agreed within ±1 bpm on a worn band (verified 94–98 bpm).
- Live HR is optical/PPG-derived — no optical, no HR.
- Parsers: `research_playground.py:parse_r10 / parse_r21`.

**Optical PPG (R21):** a 6-channel optical record (~1244 B) seen in the live stream;
green channels carry the pulse waveform. This is the only honest source of
respiratory rate (from the amplitude/baseline modulation) — and it is **live-stream
only**, so it isn't present in normal overnight flash.

---

## 7. Events (`0x30`)

Layout: `[0]=0x30 [1]=seq [2:4]=event_id u16 [4:8]=ts_sec [8:12]=subsec [12:]=payload`.
Full enum in `research_playground.py:Event`. Key ones (verified):

| id | Event | Meaning |
|---|---|---|
| 3 | BATTERY_LEVEL | fires on charge-state change |
| 7 / 8 | CHARGING_ON / OFF | |
| **9 / 10** | **WRIST_ON / WRIST_OFF** | authoritative wrist status |
| 13 | RTC_LOST | battery fully died (clock reset) |
| 14 | DOUBLE_TAP | physical gesture |
| 21 / 22 | BATTERY_PACK_CONNECTED / REMOVED | charging puck |
| 23 | BLE_BONDED | true pair success |
| 28 | FLASH_INIT_COMPLETE | boot |
| 96 | HIGH_FREQ_SYNC_PROMPT | flash filling → strap wants a fast sync |

Events `61/62/67/69` appear during boot on newer firmware; meaning unknown.

---

## 8. State signals

| Signal | Poll | Event |
|---|---|---|
| Battery % | `GET_BATTERY_LEVEL` resp `[5:7]` u16 ÷10 | BATTERY_LEVEL (3) |
| On-wrist | HELLO body wrist byte; type-24 HR `[17]` = 0 when off-wrist | **WRIST_ON (9) / OFF (10)** |
| Charging | `GET_EXTENDED_BATTERY_INFO`: charge-current/voltage fields | CHARGING_ON/OFF (7/8), pack 21/22 |

The standard `0x180F` battery service reads a buggy constant 100% — use
`GET_BATTERY_LEVEL` instead.

---

## 9. HELLO (identity)

`GET_HELLO_HARVARD` response body offsets **drift across firmware**, so
`research_playground.py:parse_hello` parses by **content**: battery = first u16 in
1–100; serial = first short alphanumeric run; commit = long hex run; charging/wrist =
known body bytes (best-effort; events 9/10 are authoritative for wrist).

---

## 10. Cloud-only (not available from the band)
Recovery score, strain score, sleep stages, HRV (beat-to-beat RR — needs continuous
raw PPG, not present in type-24 flash), and the meaning of the opaque type-24 bytes.
OpenStrap computes its own equivalents from the substrate above, using published
algorithms, clearly labelled for confidence.

---

## 11. How `research_playground.py` is organized
- **UUIDs / generations** — `Gen`, `UUID_FAMILIES`, `uuids_for()`.
- **Enums** — `PacketType`, `Cmd`, `Event`, `Record`, `SyncMeta`, `Wrist`/`BodyLimb`.
- **Framing** — `crc8/crc32`, `build_frame`, `parse_frame`, `FrameReassembler`
  (length-based), `build_command`, `build_batch_ack`.
- **Command builders** — `cmd_*` one-liners for every opcode.
- **Decoders** — `parse_r24` (1 Hz record), `parse_r10`/`parse_r21` (live), `parse_r2`,
  `parse_realtime_hr`, `parse_event`, `parse_hello`, `parse_metadata`.
- **Client + CLI** — async client that scans, connects, runs the init, drains history
  with the §4 ACK loop, optionally live-streams, and prints decoded records.
  `python3 research_playground.py selftest` exercises framing/ACK/decoders offline.
