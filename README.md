# OpenStrap research

This is where the protocol gets written down and where you can talk to a WHOOP 4.0 band
yourself, over Bluetooth, from a terminal. Scan for it, connect, drain its history,
stream live heart rate and motion, decode every record, or replay a capture you saved
earlier with no band plugged in at all. One Python file does all of it, plus a document
that maps out the bytes.

If the rest of OpenStrap is the product, this is the lab notebook. The decoders that run
in production got figured out in here first.

> Not affiliated with, endorsed by, or connected to WHOOP. "WHOOP" is their trademark and
> I'm only using it to tell you which device this talks to.

## The honest pitch

A WHOOP band is a genuinely good piece of hardware. When the subscription stops, the band
doesn't break, it just goes dark, because the only app that could read it stops talking to
it. So you've got a perfectly good sensor turning into a paperweight in a drawer. This
exists so that band can keep doing something.

Is it a replacement for WHOOP? Honestly, no. Probably never will be. They've got years of
research and a whole company behind those scores; I've got a reverse-engineered protocol
and textbook equations. What this gives you is a second life for hardware that would
otherwise be e-waste, and the raw data off your own band to do whatever you want with. If
you're paying for WHOOP and happy with it, keep paying, this isn't trying to win you over.
This is for the strap that's already done with its subscription.

## A real warning before you start

**Once you start using this on your band, stop opening the official WHOOP app with it.**
If the band reconnects to WHOOP it may pull a firmware update, and there's a real chance
the events and records this relies on shift or stop working the way they do now. I've
tested all of this on WHOOP 4.0 firmware as it exists today. I can't promise it survives
an update I haven't seen. Pick a lane.

Also: **everything here was tested on a WHOOP 4.0 and nothing else.** If you've got a 3.0
or a 5.0, some of this might work and some definitely won't, and I'd love to hear which.

## The events are a group project

Here's the thing about the event codes and the empirical fields. A lot of them are
educated guesses. I watched the band do something, watched a byte change, and wrote down
what I think it means. Wrist on, wrist off, charging, double-tap, those I'm confident
about because I could trigger them on demand and watch them fire. But plenty of the rest
came out of pattern-matching and hope.

No single person can confirm this stuff to 100%. It takes a bunch of people, with
different bands and different habits, noticing the same thing independently before a guess
becomes a fact. So if you run the tools and something doesn't line up, or you crack an
event I've got wrong, say so. With enough people poking at it over enough time, I genuinely
think we get to the point where every event is nailed down with real confidence. We're not
there yet. We get there together or not at all.

## The command line

```bash
python3 research_playground.py <command> [--address ... --duration ... --quiet]
```

Nothing installed, no band needed:

| Command | What it does |
|---------|--------------|
| `selftest` | Checks the framing layer against itself: CRCs, frame round-trips, the ACK bytes, the reassembler. Run this first. |
| `decode HEX` | Decode a single frame you paste in, prints JSON. |
| `replay PATH` | Re-decode a saved capture or hex dump, no hardware. |
| `catalog` | Dump every command, event, and record type it knows about, with the dangerous ones flagged. |

With `bleak` installed and a band nearby:

`scan` to find it, `info` to read its identity and battery, `monitor` to watch events roll
in, `sync` to drain the historical flash properly, `live` to stream HR and motion (add
`--force-optical` only if you mean it). Plus `battery`, `clock` / `set-clock`, `rename`,
`haptic`, `alarm`, and `off` / `reboot` if the optical LED gets stuck on.

## How the band actually talks

The full map is in `PROTOCOL.md`. The short version:

Every message is wrapped in a frame: a `0xAA` start byte, a two-byte length, a CRC-8 over
just those length bytes, the actual payload padded to a multiple of four, and a CRC-32
over that padded payload. One trap worth knowing, the reassembler keys off the declared
length, not the `0xAA` byte. Sensor payloads are full of `0xAA` bytes and Bluetooth splits
land right on them, so anything that "resyncs on `0xAA`" will shred your records. Length
based or bust.

Inside the frame it's `[packet type][sequence][opcode or event id][body]`. The packet
types you'll see: `0x23` for commands you send, `0x24` for replies, `0x28` and `0x2B` for
live data, `0x2F` for historical records during a sync, `0x30` for events, `0x31` for the
sync markers.

The sync handshake is the fiddly part. You send a five-packet intro ending in "send me
your history," and the band floods you with records and markers. Each time it hits a
`HistoryEnd` marker it includes an 8-byte token at `inner[13:21]`, and you have to echo
that exact token back. Get it even slightly wrong and the band cheerfully re-sends the
same batch forever, the Groundhog Day bug. Echo it right and the cursor advances. It stops
when it sends `HistoryComplete`, and it never erases its own flash doing this, so you can
drain the same band as many times as you like.

The code in `research_playground.py` is laid out in the same order as the doc: framing
first (`build_frame`, `parse_frame`, `FrameReassembler`), then commands
(`build_command`, `build_batch_ack`), then the decoders (`parse_r24` and friends), then
`WhoopClient`, the async state machine that ties it together.

## Where I'm confident, where I'm guessing

**Confident, checked on a real band:** the Bluetooth service and its characteristics, the
framing, the packet types, heart rate at byte `[17]` (matches the live stream within a
beat or two), the whole sync handshake and that 8-byte token, the obvious events (wrist,
charging, double-tap, bonded), the basic commands, and live HR/PPG/IMU streaming.

**Empirical, fingerprinted but unconfirmed:** most of the 1 Hz record past the header. The
accelerometer at `[36:48]` (sits near 1g at rest), the temperature channel around `[70]`
(climbs when worn), SpO₂ at `[72]` (low 90s at rest), the held resting HR at `[88]`. These
move the right way against things I can verify, which is encouraging, but the band relays
them to the cloud raw so I've got no ground truth to check against.

**Genuinely unknown:** the leftover opaque bytes, anything resembling HRV or beat-to-beat
timing (I haven't found it in what the band exposes), and WHOOP's own recovery, strain,
and sleep scores, which are computed in their cloud and never come down the wire.

## Don't brick your band

The band has commands that can erase its flash, reboot it, or jam the optical LEDs on. The
client never sends the erase command, keeps optical wrist-gated by default, treats reboot
as a deliberate manual recovery step, and keeps the dangerous opcodes behind a guard so
you can't fire one by accident. If the LED does get stuck on, `off` then `reboot` usually
sorts it. `PROTOCOL.md` lists the footguns.

## Using it

For yourself, your own band, non-commercial. This is not a medical device, it diagnoses
nothing, and it comes with no warranty. MIT licensed.
