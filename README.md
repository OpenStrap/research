# OpenStrap — research

A protocol reference and reference client for talking to a **WHOOP 4.0 band**
directly over Bluetooth Low Energy, so a strap you already own keeps working after
you stop paying for the service.

> Not affiliated with, endorsed by, or connected to WHOOP. "WHOOP" is a trademark
> of its owner; it's used here only to say which hardware this talks to.

## Why this exists

The band is a sensor. The subscription is the service. When the subscription ends,
the hardware doesn't stop working — it just stops being *useful*, because the app
that reads it goes dark. A retired athlete, someone between subscriptions, anyone
who already bought the device — suddenly holding e-waste on a charger.

OpenStrap is the honest answer to that. It speaks the band's BLE protocol, pulls the
raw sensor records the device records, and derives what those records can *honestly*
support — using **published, peer-reviewed algorithms**, clearly labelled for
confidence.

**It is not a WHOOP replacement and never claims to be.** WHOOP's value is years of
proprietary research, a cloud, and an app. A single hobby project does not reproduce
that, and pretending otherwise would be dishonest. If those insights are worth it to
you, keep paying for them — this isn't for you. This is for the strap that would
otherwise be in a drawer.

## What's in this folder

| File | What it is |
|---|---|
| `research_playground.py` | Single-file reference client + CLI: scan, connect, drain history, live-stream HR/PPG/IMU, decode records, read battery/wrist/charge. Importable as a module. |
| `decode_events.py` | Small helper for decoding event frames. |
| `PROTOCOL.md` | The full technical reference: BLE framing, packet/record/command formats, the historical-sync handshake, the 1 Hz record layout, and how the client is organized. |

Run with no hardware to sanity-check the framing/ACK/decoders:
```bash
python3 research_playground.py selftest
```

## What we know — and how sure we are

Honesty about confidence is the whole point. Three tiers:

### ✅ Confident — verified on real hardware
- **BLE transport**: the proprietary GATT service + its five characteristics, and the
  framing envelope (SOF, length, CRC-8 over the length, CRC-32 over the padded inner).
- **Packet types** (`0x23` command, `0x24` response, `0x28`/`0x2B` live, `0x2F`
  historical, `0x30` events, `0x31` sync metadata).
- **Heart rate** — byte `[17]` of the 1 Hz record; cross-checked against the live
  stream within ±1 bpm on a worn band.
- **Historical sync** — the start/end/complete handshake and the exact 8-byte
  continuation token (`inner[13:21]`) the ACK must echo. Reproduced live; the read
  cursor advances on ACK and persists across connections; the drain is
  non-destructive (we never erase flash).
- **Events** — wrist on/off (9/10), charging (7/8), double-tap (14), bonded (23), and
  the rest of the enum.
- **Commands** — battery, data range, read pointer, live-stream toggles, alarm, haptics.
- **Live HR/PPG/IMU streaming** when optical is enabled (HR is optical-derived).

### 📊 Empirical — fingerprinted, not certain
Past the record header, most of the 1 Hz record is opaque (the band relays it raw to
the cloud, which decodes it). These were inferred by correlating bytes against known
state across hundreds of real records — plausible, **not** guaranteed:
- **Tri-axial accelerometer** (g) at `[36:48]` (|a|≈1 g at rest).
- **Skin / temperature channel** (`≈ [70]/4 °C`) — tracks warming on-wrist; whether
  it's a true thermal reading or an optical/gain register is **unverified**.
- **SpO₂** at `[72]` (stable 92–94 at rest).
- **Resting/baseline HR** at `[88]` (a held value, distinct from live HR).

### ❓ Unknown — we don't have these, and we say so
- The remaining opaque bytes of the 1 Hz record (raw-PPG-ish clusters, small flags).
- **HRV / beat-to-beat intervals** — not recoverable from what the band exposes here.
- WHOOP's **recovery, strain, and sleep scores** — those are computed in their cloud
  from relayed raw records. We can't read them, and we don't fake them. OpenStrap
  computes *its own* equivalents from the substrate we do have, with published methods.

## Safety

The band has destructive commands (flash erase, reboot, persistent-optical). The
client **never** sends the erase command, defaults optical to wrist-gated, and treats
reboot as a manual recovery action. See `PROTOCOL.md` for the footguns and how to
recover (e.g. a stuck optical LED).

## License & use

For personal, non-commercial use with hardware you own. Not a medical device — it
makes no diagnostic claims and carries no warranty. See the repo license.
