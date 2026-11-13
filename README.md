# TB VS Bass 100 (Mountain Lion VST)

A model of the **H/H VS Bass 100** bass amplifier as a VST 2.4 plugin, 64-bit Intel, for
**Mac OS X 10.8.5** (Pro Tools 11 through Blue Cat PatchWork). Non-commercial, personal use.

Built from the H/H drawing *V-S Bassamp Circuit Diagram* C/AC00015, issue 5 (in `Docs/`), which
covers the AS00016 Ch 1 board, the AS000/B tone correction module's wiring and the AS00014/2
output driver board. The front panel photo is `Docs/front_panel.jpg`.

## The panel

Laid out like the VS Bass 100's silver front panel, each group in its own outlined box:

| Control band | **Gain** (0–10), **Bass Boost** (Off/On rotary), **Bass, Middle, Treble, Presence** (−5 · 0 · +5), **Valve Sound** (Off/On rotary, tinted box), **Master Volume** (0–10) |
|---|---|
| Name strip | TB electronic logo, model name, red **Mains** rocker |
| Plugin strip | **Cab** (1x15 / 4x10), **Speaker** (miked cab or speaker terminals), **Input** ±12 dB, **Output** ±24 dB, 8 named presets, readout |

The input jack, effects input and Valve Sound footswitch socket are left out. **Mains** off
passes the dry signal (time-aligned). Every switch lights up on its engaged side; the rotary
selectors light their engaged legend.

| # | Name | Control | Range |
|---|------|---------|-------|
| 1 | Gain | Gain | 0 – 10 |
| 2 | BassBst | Bass Boost (S3) | Off / On |
| 3 | Bass | Bass | −5 · 0 · +5 |
| 4 | Middle | Middle | −5 · 0 · +5 |
| 5 | Treble | Treble | −5 · 0 · +5 |
| 6 | Presence | Presence | −5 · 0 · +5 |
| 7 | ValveSnd | Valve Sound (S2) | Off / On |
| 8 | M Volume | Master Volume | 0 – 10 |
| 9 | Mains | Mains switch | Off / On |
| 10 | Cab | *plugin only*: speaker cab | 1x15 / 4x10 |
| 11 | Speaker | *plugin only*: miked cab, or the speaker-terminal signal for your own IR | On / Off |
| 12 | Input | *plugin only*: input trim | ±12 dB |
| 13 | Output | *plugin only*: output trim | ±24 dB |

**Presets:** click **SAVE**, then a slot **1–8**, type a name and press Return. Click a slot to
recall it; click the readout while a preset is in use to rename it. Presets are kept in
`~/Library/Application Support/TB/TB VS Bass 100/Presets.txt`, shared by every copy of the plugin.

## What is modelled

**From the component values on the drawing** (nodal circuit solver, 4x oversampled, 741s with
1 MHz gain-bandwidth, 0.5 V/µs slew, ±14 V swing on the ±16 V zener rails):

- **IC1 gain stage** (AS00016): R1 10K, C1 0.1µ, R3 100K into the Q1/Q2 2N5089 pair and 741;
  feedback R9 22K ∥ C4 68p; R8 2K7 with R22 1K linked across it (issue 5, pins 26–27), C3 6µ8 and
  the dual-action 25K reverse-log **Gain** pot (one leg in the feedback, the other shunting the
  output).
- **Bass Boost** (S3): puts R21 1K5 across R11 10K at the top of the tone stack.
- **Tone stack**: Bass VR5 100K lin with C6/C7 0.047µ and R12 10K; Treble R14 10K, VR6 100K lin,
  R15 10K, C8 4700p; R13 100K; C9 0.1µ into **IC2** (inverting, R16 1M5), whose clipping is the
  V-S sustain.
- **Power amp** (AS00014/2, issue 3 values R3 390R, C2 6µ8): closed-loop gain ×38.7 with its input
  and feedback-leg low cut, slew, clipping on ±48 V rails that sag through the 180 VA transformer
  and 4700 µF reservoirs, the Q4/Q5 V-I limiter, and the output network (L1, R29 15R, Zobel
  R30 15R + C14 0.047µ) into the cab's electrical load.

**From behaviour, because they are not circuits on the drawing:**

- The **AS000/B V-S tone correction module** is drawn only as a box with pins, so its **Middle**
  (25K lin) and **Presence** (25K lin) controls and the **Valve Sound** switch are modelled from
  what they do: a mid peak/dip, a presence shelf, and asymmetric soft saturation (the same Valve
  Sound section as TB VS Musician).
- **The cabs**, both 8 ohms: the owner's **H/H 15" (1977) in a 1x15** and **Trace Elliot 4x10
  (1984)**. Each is an electrical driver model (voice-coil resistance and inductance, the
  resonance in the box) that the power amp drives, plus a fixed acoustic voice. With Speaker on,
  the output is the cone's acceleration through that voice.

### Estimates (values the drawing doesn't fix)

| Constant | Value | Why |
|---|---|---|
| `kMidFreq`, `kMidRange` | 500 Hz, ±12 dB | Middle, inside the AS000/B module |
| `kPresFreq`, `kPresRange` | 2.5 kHz shelf, ±10 dB | Presence, inside the module |
| `kModuleOut` | 0.27 | module output level into the Master Volume |
| `kRailRs`, `kRailDrop`, `kIMax`, `kPaSlew` | 1.5 ohm, 2.5 V, 7.5 A, 2 V/µs | supply source resistance (180 VA), output stage losses, V-I limit, slew |
| L1 | 3 µH | not marked |
| `kPickupR` | 8K | bass pickup |
| Cab models (`kCabs`) | 1x15: resonance 45 Hz, rolls off from 3 kHz; 4x10: resonance 68 Hz, more upper mids, rolls off from 5.2 kHz | to be tuned against the real cabs |
| Cab low-end weight | 1x15: +6 dB shelf at 100 Hz; 4x10: +9 dB shelf at 110 Hz | voiced after the first test: weight from about 40 to 100 Hz without modern sub bass (below ~35 Hz the cabs still roll off). Miked path only; Speaker off stays as the circuit gives it, for your own IRs |
| Effects input | left open | R2 10K goes to the effects input, which the plugin doesn't have |

Notes from the drawing:
- **Bass Boost is gentle** as drawn (about +1 to +3 dB, most in the low mids with Bass turned
  up), because the 100K Bass pot dominates R11/R21.
- The power amp's low cut (R3 390R + C2 6µ8, the issue 3 change) and the tone stack's values
  give the amp a lean low end at the speaker terminals. With Speaker on, the cab voicing adds
  weight from about 40 to 100 Hz (see the table above).

## Build and test

GitHub Actions builds and verifies the bundle on an Intel macOS runner (minimum OS 10.8.5,
only 10.8-era system libraries), loads it in a mini host, drives the panel offscreen and
publishes screenshots to the `gui-preview` branch. Locally on Linux:

    make -C Tests run vst-run      # DSP tests, preset store, plugin in a mini VST host
    make -C Tests asan             # the same under AddressSanitizer + UBSan

## Install

1. Actions → latest run → Artifacts → **TB-HH-Bass-mountain-lion-vst** → download.
2. Unzip (twice: the artifact zip holds `TB-HH-Bass-mountain-lion-vst.zip`).
3. Copy **TB VS Bass 100.vst** to `/Library/Audio/Plug-Ins/VST/`.
4. Rescan in Blue Cat PatchWork.
