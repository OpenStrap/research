# WHOOP 5.0 BLE — protocol reference

The byte-level reference for talking to a WHOOP 5.0 band ("Gen 5"). Same spirit as
`PROTOCOL.md`, different device family: confidence is marked throughout, **verified**
= confirmed on real hardware, **empirical** = inferred by correlation, plausible but
not certain, **unknown** = not determined.

> Not affiliated with or endorsed by WHOOP. For interoperability with a device you own.

Validated against one WHOOP 5.0 on firmware `50.40.1.0`, hardware family `13`, optical
revision `82`. Everything below is from that device unless marked otherwise — a single
strap is a real limitation, and cross-device confirmation would be welcome.

Like Gen 4, the band is a **sensor pipe**. It records raw and lightly-processed sensor
records to flash and streams them to a phone. Sleep staging, recovery and strain are
**not computed on the band** and never come down the wire.

Unlike Gen 4, a fair amount of signal processing *does* happen on-device, and some of it
is exposed: the Gen 5 record carries **beat-to-beat R-R intervals in milliseconds**,
which `PROTOCOL.md` lists as genuinely unknown for Gen 4. See §7.

---

## 0. Gen 5 is a different family — do not point a Gen 4 decoder at it

Everything below differs from Gen 4. If you reuse a Gen 4 decoder you will get CRC
failures, not subtly wrong data — which is the good outcome.

| | Gen 4 | Gen 5 |
|---|---|---|
| service UUID | `61080001-8d6d-82b8-614a-1c8cb0f8dcc6` | `fd4b0001-cce1-4033-93ce-002d5875f58a` |
| header checksum | CRC-8 (poly `0x07`) over the size bytes | **CRC-16/Modbus** over frame bytes 0..5 |
| envelope | `[0xAA][u16 size]` | `[0xAA][rev][u16 size][src][dst][crc16]` |
| command packet type | `0x23` | `35` (`0x23` — same) |
| main 1 Hz record | type-24, 96 bytes | type 47 revision 18, 99-byte body |

Dispatch on the **service UUID**, not on the frame — that is the only field that
separates the families before you have parsed anything.

---

## 1. BLE transport & framing

### 1.1 Connecting — **verified**
- Scan filtered by the service UUID below.
- BLE is one-central, as on Gen 4: disconnect the official app first.
- Request **MTU 247**; that is what the official client asks for and gets.
- There are **two delays that matter** in the official bring-up: ~600 ms after service
  discovery *before* registering notifications, and ~500 ms *after* registering them
  before the first command. Skipping them produces intermittent early-command failures.

### 1.2 GATT service (Gen 5) — **verified**

Suffix for all Gen 5 UUIDs: `cce1-4033-93ce-002d5875f58a`

| UUID | direction | role | notify |
|---|---|---|---|
| `fd4b0001-…` | — | service | — |
| `fd4b0002-…` | host → band | commands | — |
| `fd4b0003-…` | band → host | command responses | required |
| `fd4b0004-…` | band → host | events | required |
| `fd4b0005-…` | band → host | realtime / history / data | required |
| `fd4b0007-…` | band → host | crash/diagnostic data | optional |

### 1.3 Frame envelope — **verified**

All multi-byte integers little-endian.

| offset | size | value |
|---:|---:|---|
| 0 | 1 | `0xAA` start of frame |
| 1 | 1 | envelope revision; `0x01` outgoing |
| 2 | 2 | declared inner length, **including** the trailing CRC32 |
| 4 | 1 | source role; host writes `0` |
| 5 | 1 | destination role; host writes `1` |
| 6 | 2 | CRC-16/Modbus over offsets 0..5 |
| 8 | var | inner packet padded to a multiple of 4, then CRC-32 (zlib) |

Total frame length is `8 + declared`.

**The same reassembly trap as Gen 4 applies, and for the same reason.** Notification
chunks are a byte stream: one chunk is not one frame and one frame may span several.
Retain incomplete data *per characteristic*, scan for `0xAA`, read the declared length,
and only then emit a candidate. Anything that resyncs on `0xAA` will shred sensor
payloads.

Worked example — `GET_HELLO(145)`, sequence 1, body `01`:

```
aa0108000001e671 23019101 363e5c8d
| outer header | inner  | CRC32  |
```

### 1.4 Inner packet — **verified**

```
command   = [35][sequence][opcode][body…]
response  = [36][sequence][echoed opcode][originating sequence][result][body…]
result:  0 FAILURE  1 SUCCESS  2 PENDING  3 UNSUPPORTED
```

Write **with response**. Install the notification handler **before** the first write —
a fast response can otherwise arrive before anything is listening.

**Accept a response only when the originating sequence *and* the echoed opcode both
match.** A sequence match with the wrong opcode is not a success. Sequences are a byte,
wrap modulo 256, and **zero is a valid sequence**.

The generic await is **5,000 ms**, applied once, with **no automatic resend**. Do not
wrap it in a blind retry loop — several opcodes mutate persistent state, and a duplicate
write after a slow-but-successful response is a real hazard.

---

## 2. Command set — **verified** unless noted

Opcode at `inner[2]`. The ones you actually need:

| opcode | name | notes |
|---:|---|---|
| 145 | `GET_HELLO` | identity; gates readiness |
| 10 / 11 | `SET_CLOCK` / `GET_CLOCK` | |
| 141 | `GET_ADVERTISING_NAME` | |
| 151 | `GET_BATTERY_PACK_INFO` | charging only |
| 34 | `GET_DATA_RANGE` | history bounds |
| 22 | `SEND_HISTORICAL_DATA` | start a drain |
| 23 | `HISTORICAL_DATA_RESULT` | the ACK — see §8 |
| 20 | `ABORT_HISTORICAL_TRANSMITS` | |
| 3 | `TOGGLE_REALTIME_HR` | live type-40 |
| 106 | `TOGGLE_IMU_MODE` | live R21 |
| 107 | `ENABLE_OPTICAL_DATA` | historical R20 recording |
| 108 | `TOGGLE_OPTICAL_MODE` | live R20 |
| 117 / 118 | `START_FF_KEY_EXCHANGE` / `SEND_NEXT_FF` | feature flags, §9 |
| 120 / 128 | `SET_FF_VALUE` / `GET_FF_VALUE` | **persistent writes** |
| 115 / 116 | device-config enumerate | |
| 119 / 121 | device-config set / get | **persistent writes** |

### Footguns — keep these behind a guard

| opcode | why |
|---:|---|
| 29 | `REBOOT_STRAP` (empty body) |
| 33 | `SET_READ_POINTER` — rewrites the history cursor |
| 45 | `ENTER_BLE_DFU` |
| 119 / 120 | persistent config/flag writes, EEPROM-class |

Normal history catch-up **never needs opcode 33**. The drain in §8 is non-destructive
and re-runnable, exactly like Gen 4.

---

## 3. Session init — **verified**

```
scan (service-filtered) → connect → discover
→ wait ~600 ms → register notifications on 0003/0004/0005 → wait ~500 ms
→ GET_HELLO(145)
→ clock check
→ GET_ADVERTISING_NAME(141)
→ READY
```

There is **no protocol-version negotiation** on Gen 5. `GET_MAX_PROTOCOL_VERSION(2)`
exists in the vocabulary but the official bring-up does not call it.

`GET_HELLO` failures are counted; five consecutive failures trigger a bond removal in
the official client.

### Hello body — the fields worth reading

| offset | size | meaning | observed |
|---:|---:|---|---|
| 79 | 4 | hardware family | `13` |
| 87 | 4 | optical revision / family discriminator | `82` |

The family discriminator is what actually pins the device to WHOOP 5.0; the valid
interval is `48 <= value < 86`. The service UUID alone is not the final discriminator.

---

## 4. Packet types & revisions — **verified**

Type at `inner[0]`.

| type | meaning |
|---:|---|
| 40 | realtime heart rate |
| 43 | realtime sensor body (revision at `inner[1]`) |
| 47 | historical sensor body (revision at `inner[1]`) |
| 48 | events |
| 49 | history metadata markers |
| 51 / 52 | dedicated IMU stream wrapper |

For types 43/47 the body starts at **inner offset 13**.

| rev | full frame | content |
|---:|---:|---|
| 10 | 1,932 | HR + fixed accel/gyro arrays (returned `UNSUPPORTED` on a tested band) |
| 16 | 1,584 | partially documented |
| 17 | 240 | filtered reading |
| **18** | **124** | **processed sensor body, 99-byte body — the main payload** |
| 20 | 2,140 | optical / PPG, five channel blocks |
| 21 | 1,244 | IMU |
| 22 | 188 | research telemetry, six tagged variants (§9) |
| 26 | 88 | Pulse Information Packet |

---

## 5. The 1 Hz record — type 47 revision 18 — **verified layout**

99-byte body. This is the Gen 5 equivalent of Gen 4's type-24 and it is considerably
richer.

| body | size | field |
|---:|---:|---|
| 0 | 1 | zero |
| 1 | 1 | heart rate (rounded signal-processing estimate; same value as type-40 HR) |
| 2 | 1 | **emitted R-R interval count, 0..4** — see §7 |
| 3 | 8 | **four R-R intervals in ms, `u16` LE**; unused slots zero |
| 11 | 8 | two signal-processing flag words (A at 11, B at 15) |
| 19 | 1 | signal-processing status |
| 20 | 4 | max adjacent acceleration-magnitude delta, `float32` g |
| 24 | 12 | mean acceleration X/Y/Z, three `float32` g |
| 36 | 2 | cumulative step count, `u16` LE |
| 38 | 2 | low byte = U6.2 cadence interval code; high byte zero |
| 40 | 2 | zero |
| 42 | 1 | step-activity class: `0` unknown, `1` walk, `2` run, `0xff` invalid |
| 43 | 5 | zero |
| 48 | 2 | cell temperature, signed, 0.1 °C/LSB |
| 50 | 2 | ambient temperature, signed, 0.1 °C/LSB |
| 52 | 2 | skin temperature (AS6221), signed, 0.01 °C/LSB; `-5000` = unavailable |
| 54 | 6 | three packed per-channel AGC/state `u16` words |
| 60 | 1 | packed state byte — **sleep state in bits 4..5** |
| 61 | 1 | SpO₂ estimate/status byte |
| 62 | 21 | zero |
| 83 | 1 | garment/body-tag location id (`1` = on-wrist) |
| 84 | 1 | movement-fundamental bin, `0.732421875` cycles/min per LSB — **not** heart rate |
| 85 | 2 | selected-source PD-B/PD-A DC-window levels |
| 87 | 2 | selected-source PD-B/PD-A pSNR, signed `int8` dB; `-128` unavailable |
| 89 | 3 | zero |
| 92 | 4 | natural log of the HR-tracker variance, `float32` |
| 96 | 3 | zero |

### The two flag words — body 11 and body 15 — **verified**

Both are `u32`. Neither is 32 independent booleans, and both carry fields that *look*
like data and are not. **Retain each word whole** rather than flattening it.

**Flags word A (body 11).** Only the low half is flag-like — bits 0..8, 11..15 and 28
are individual signal-processing diagnostics, and bits 9..10 are unwritten. Bits 29..30
are the low two bits of one four-state internal tracker: a 2-bit value, not two flags.
Bits 16..27 plus bit 31 serialise a rotating engineering telemetry stream — bits 16..23
a mode-dependent byte, bits 24..27 a rotating nibble, bit 31 its block-start marker.
None of bits 16..27 has a fixed standalone meaning across records.

**Flags word B (body 15).**

| bits | meaning |
|---|---|
| `0x10` | processing source index 2 (CH3 infrared) is selected — the cleanest source oracle |
| `0x20` | transition/hold indication from the same three-state switch |
| `0x40` | set when source 2 is selected, but a global override can also set it; body 11 bit `0x40` mirrors it |
| `0x80` | a diagnostic-format / special-state marker — **see the warning below** |
| 8..15 | an HR-domain scalar, **not** flags — see §11 |
| 16..31 | a frequency-derived pair, **not** R-R data — see below |

> **Bit `0x80` is not a validity bit.** It is not HR-present, HR-valid, confidence,
> corroboration, quality or walk-detected. It has two producers and the record does not
> serialise which one fired, so the same wire bit means different things in different
> states. Heart rate is routinely present and in range while it is clear. **Gate nothing
> on it.** What actually drives it is **unknown** — neither documented producer condition
> reproduces cleanly against captured history.

> **Bits 16..31 are not R-R intervals.** They are a frequency-derived pair in cycles per
> minute. Real R-R intervals live at body 2 and body 3..10, in milliseconds — see §7.
> Body 17 is written in roughly 98% of records and spans `25..209`; body 18 spans
> `26..209` and is zero in most records, consistent with a second tracker slot that is
> frequently absent. Both are engineering diagnostics with no stable per-record name:
> retain them raw and decode neither as an interval, a rate, nor a bitfield.

### Sleep state — body 60 bits 4..5 — **verified**

```
0 WAKE   1 STILL   2 SLEEP   3 UP
```

Bits 2..3 carry a passive strap-fit classifier state. This is the band's own state
machine, not a sleep *stage* — stages are still cloud-side.

### Step cadence — body 38 — **verified**

Body 38 is **not** steps per minute. It is an unsigned U6.2 count of quarter-samples
between steps:

```
steps_per_minute = 240 * EDMP_ODR_Hz / raw
```

The tested firmware leaves the pedometer rate at its 50 Hz reset setting, giving
`12000 / raw`. Guard `raw == 0` rather than dividing by it, and store the raw byte —
do not bake the constant into a decoder meant to outlive one firmware.

---

## 6. Realtime heart rate — type 40 — **verified**

Exact 20-byte revision-2 inner layout:

| offset | size | field |
|---:|---:|---|
| 0 | 1 | packet type `40` |
| 1 | 1 | revision; observed `2` |
| 2 | 4 | whole seconds, `u32` |
| 6 | 2 | subseconds, 32768 units/s |
| 8 | 1 | heart rate |
| 9 | 1 | **emitted R-R interval count, 0..4** |
| 10 | 8 | **four R-R intervals in ms, `u16` LE**; unused slots zero |
| 18 | 1 | `isOffWrist = (value == 0)` |
| 19 | 1 | garment/body-location code |

Location codes: `0` UNKNOWN, `1` WRIST, `2` BICEP, `3` CALF, `4` SIDE_TORSO, `5` GLUTE,
`7` ANKLE, `128` NOT_CONCLUSIVE, `160` UNKNOWN_GARMENT. Treat an unrecognised value as
an error rather than coercing it.

**Off-body the band emits no type-40 frames at all**, even though enable and disable
both return success — the packet is wear-gated rather than reporting an off-body value
in byte 18.

---

## 7. R-R intervals — **verified**

`PROTOCOL.md` lists beat-to-beat timing as genuinely unknown for Gen 4. **Gen 5 exposes
it**, in two places carrying one contract: type-40 offsets 9..17, and R18 body 2 and
body 3..10. Decode it once and share the result.

### How it is produced

A peak detector runs over a rolling four-second, 400-sample optical pulse waveform at
100 Hz and forms ordered adjacent-peak intervals. Each is clamped to a lower bound of
1/3 second — 0.5 second in some source/mode states — and an upper bound of 2.4 seconds.
At most the first four are emitted, rounded to whole milliseconds, so **every emitted
value falls in `333..2400`**.

### The motion gate — the part that will confuse you

Emission is suppressed by movement. The band takes 100 Hz acceleration magnitude in g,
passes it through two IIR stages, takes the absolute value, applies a third IIR stage,
keeps each one-second block's maximum in a 30-entry history, and emits intervals only
while the maximum of that history is strictly below `0.03`. After warm-up that is the
rolling 30 seconds. If the gate fails, the count and all four slots are zero.

### Decoder rules

- The count is **not** a beat count for that second. It is zero in most records and
  falls further as heart rate rises.
- **A zero count is ambiguous** — it does not distinguish "no new valid interval" from
  "suppressed by motion". Never surface it as a physiological event.
- Intervals within one record are chronological and consecutive.
- **The series across records is sparse.** Never bridge a zero-count record, a timestamp
  gap, or non-adjacent records. Consecutive records do not imply consecutive beats.
- Do not synthesize intervals from heart rate. In aggregate they track `60000 / HR`
  closely (median ratio ≈ 0.999) but only a small fraction exactly restate that rounded
  period.

This is enough for HRV work that Gen 4 cannot support, with the caveat that the gate
makes coverage bursty and strongly biased toward stillness.

---

## 8. Historical sync — **verified**

Same shape as Gen 4 — a drain with an echo-back ACK — but a different correctness rule.

```
GET_DATA_RANGE(34)        → bounds
SEND_HISTORICAL_DATA(22)  → band streams records + type-49 markers
  … per burst: HISTORY_END marker → HISTORICAL_DATA_RESULT(23) echoing the markers …
HISTORY_COMPLETE          → done
```

Type-49 metadata discriminators: `1` HISTORY_START, `2` HISTORY_END, `3`
HISTORY_COMPLETE.

### The count gate — the Gen 5 equivalent of the Groundhog Day bug

Each `HISTORY_END` declares an expected count. **Count each complete frame once,
independent of revision or byte length**, and only for these types:

| type | counts |
|---:|---|
| 47, 48, 50, 53, 54, 55 | **yes**, once per complete frame |
| 49 | no |
| 51, 52 | no |

Get the membership wrong and the band re-offers the same burst forever. Retain both
markers alongside your database position as an audit trail.

Other behaviour worth knowing:

- Default burst size **50**; after **5 consecutive negative results** it drops to **10**.
- **15 failed validations** is the terminal stuck boundary — abort with opcode 20.
- The drain is **non-destructive**: it never erases flash, so you can re-drain freely.
- A repeated `(revision, sequence)` is **not** by itself proof of a duplicate.
- **Never interleave config writes with an active transfer.**

---

## 9. Device config & feature flags — **persistent, read this twice**

Two parallel key/value stores, both **band-enumerated** — do not hardcode the key list.

```
device config:  115 enumerate → 116 next → 119 SET / 121 GET
feature flags:  117 enumerate → 118 next → 120 SET / 128 GET
```

Value block, both directions:

```
01 + key[32 ASCII/NUL] + value[32 ASCII/NUL]
```

Enumeration: the count is a **single unsigned byte at response-body offset 1**, not a
`u16`. In a NEXT response read the key at **offset 3**, accepting it only when the byte
at **offset 2 equals `1`**.

### The boolean encoding will bite you — **verified**

```
ASCII "1" → enabled
ASCII "2" → disabled
ASCII "0" → raw zero / unset, NOT "off"
```

A band commonly sits at `"2"`, not `"0"`. **Raw zero is not restorable** — neither `"0"`
nor `"2"` is a proven restoration of an observed raw-zero state. If a key snapshots as
`0`, skip it entirely. A blanket "enable everything, then restore" sequence is *not*
reversible.

Rules that are worth the trouble: snapshot fresh every session, one key at a time, GET
after every SET, persist a rollback record before writing, and restore-and-verify at the
end. For values the firmware caches, a reconnect may not be enough — a reboot may be
required, and a stale cache looks identical to an inert flag.

### Revision 22 — research telemetry — **verified**

`enable_r22_packets` is the master **and** the historical publication gate.
`enable_r22_v2_packets` … `v6` select one writer, priority `v6 → v5 → v4 → v3 → v2 →
otherwise variant 1`. `enable_r22_v8_packets` is **not** in the selector and has no
reader — dormant, do not write it.

Body byte 0 tags one of six layouts: `1` CH1 optical window, `2` tag 1 + metrics,
`3` CH2-A/CH4-A windows, `4` tag 2 + per-channel config, `5` a completed PIP ring
record, `6` 25 × `i16` accel X/Y/Z.

Three things that make R22 confusing in practice:

- **It is historical, not realtime.** Enabling opcodes 106/108 does not mirror R22 into
  the live channel. With the flag set and the strap rebooted, two 60-second realtime
  captures produced 60 R20 + 61 R21 + **0 R22**.
- **Selected writer ≠ emitted tag.** Variant 3 falls back to tag 2 when its source
  windows are zero; variant 5 falls back to tag 4 without a completed PIP record.
- **Tag 3 needs sleep.** Its channels populate only during the SpO₂ episodes that occur
  inside sleep state 2. A 1,500-second awake run on variant 3 produced 1,119 packets,
  all tag 2.

**Is it worth enabling?** Mostly no. Tag 5 duplicates R26. Tags 1–4 are a lossy subset
of what R20 already carries across five channel blocks. Only tag 6 — historical raw
acceleration — is unavailable elsewhere. A passive drain with no flag writes at all
returned 2,447 R18 + 2,447 R20 + 80 R26 + 0 R22, which is most of what an app needs.

---

## 10. Other bodies, briefly

- **Revision 20 — optical/PPG.** 2,115-byte body, five blocks (CH1, CH2, CH4, CH5-with-
  CH3-fallback, CH6), each a count + 20-byte descriptor + two `50 × i32` A/B streams.
  Raw 20-bit optical ADC codes off a MAX86176. CH2/CH4 blocks are dormant while awake.
  Historical via opcode 107, realtime via 108.
- **Revision 21 — IMU.** 1,232-byte inner: up to 100 `i16` accel X/Y/Z then 100 gyro
  X/Y/Z. Acceleration `raw / 4096` g (±8 g), gyro `raw / 16.4` °/s (±2000 °/s). Inner
  byte 625 is a **status bitfield**, not the gyro range selector. Realtime only.
- **Revision 26 — Pulse Information Packet.** 63-byte body, episodic detector output —
  observed episodes are exactly 40 records at 1 Hz, not one-per-second. The saturating
  `i16` delta encoding is **lossy**: a `-32768` clamp destroys its operand, so zero
  padding, a flat signal and the negative rail all reconstruct identically.

---

## 11. Where I'm confident, where I'm guessing

**Verified on real hardware:** the service and characteristics, framing and CRCs, the
command/response correlation rule, session bring-up, the R18 layout, type-40, the R-R
interval contract and its motion gate, the history drain and its count gate, the config
and feature-flag exchange including the `1`/`2`/`0` encoding, and the R22 selector
behaviour.

**Empirical:** the finer diagnostic fields in R18 — the AGC/state words, pSNR, DC-window
levels — move correctly against things that can be checked but have no independent
ground truth.

**Genuinely unknown:**
- exact LED package identity, channel-to-colour wiring, and peak wavelengths
- stable per-record meaning for flags-A bits 16..27, which carry a rotating engineering
  telemetry stream rather than fixed fields
- named body positions for garment/body-tag location ids `2..15`
- per-mode current draw — traffic ownership is known, current is not inferable from
  protocol behaviour
- WHOOP's own sleep-stage, recovery and strain scores, which are cloud-side on Gen 5
  exactly as on Gen 4

### One field specifically worth *not* guessing about

R18 body 16 (flags-B bits 8..15) looks like a second heart rate and resembles the
published HR closely enough that small samples suggest it is a duplicate. Over 1.29M
records it takes only `0` and values in `30..210` — `1..29` and `211..255` never occur —
so it is definitely an HR-domain scalar and definitely not a bitfield. But **how often
it equals the published HR ranges from 4% to 80% across ordinary captures of the same
band**, depending on an internal branch the packet does not serialise. Retain it raw;
do not name it, and do not use agreement with the HR byte as a validity signal.
