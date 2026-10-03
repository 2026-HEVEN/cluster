# Lap controller temperature telemetry

## Scope

- Instantaneous controller temperature remains controller-origin FB2 data:
  Left `0x1802D0EF`, Right `0x1802D0F0`, byte 0 unsigned, multiplier 1,
  offset -40, Celsius. Cluster does not rebroadcast these instantaneous values.
- New Cluster statistics cover laps 1-51, independently for left and right.
- Existing lap-time/battery IDs and GPS start/crossing/pause/resume logic retain
  their meaning; those older telemetry features still support 99 laps internally.
- VCU LV voltage is received under the new mirrored `lv_monitor_protocol.h`
  contract. No estimated LV voltage or LV SOC is introduced.

## VCU LV supply voltage (dev integration)

Cluster dev `b8b201c` and VCU dev `36f03a0` share an identical LV contract.
VCU measures GPIO33 via a 3.3k/1k divider from `+12V_FUSED` and broadcasts
Extended `0x1C09C0D0`, DLC 8, every 100 ms (10 Hz). Cluster receives it directly,
without retransmission, and displays it as VCU LV VOLTAGE in Car Check power
details when valid and received within 500 ms. The local home-screen adapter
also shows LV directly below HV PACK voltage, right-aligned, with two decimal
places; unseen, invalid or older-than-500-ms samples show `-- V` instead.
HV voltage remains sourced from the BMS, independently of LV freshness.

| Bytes | Signal | Type | Multiplier | Offset |
| --- | --- | --- | --- | --- |
| 0-1 | LV supply voltage | uint16 LE | 0.01 V | 0 |
| 2-3 | ADC pin voltage | uint16 LE | 0.001 V | 0 |
| 4 bit 0 | Sample valid | bool | 1 | 0 |
| 5 | Sequence | uint8 | 1 | 0 |
| 6-7 | Reserved | - | - | - |

JSON adds VCU LV Supply Voltage (byte 0-1, unsigned LE, multiplier 0.01,
offset 0, unit Volt) and VCU LV Sample Valid (bit 32, unit State).
Voltage filter/mask: `0000000001000000` / `0000000001000000`, requiring
byte 4 bit 0. Example `B0 04 E7 0A 01 2A 00 00` means 12.00 V, ADC 2.791 V,
valid, sequence 42. A filtered invalid sample can leave Monolith's previously
held voltage visible until its UI timeout; inspect Sample Valid as well.

Important limits: ADC >=3100 mV is declared invalid, corresponding to input
>=13.33 V with the nominal divider. This can exclude normal battery charging
voltages and needs VCU/hardware-team review if the LV rail exceeds this level.
A zero ADC sample is considered usable by this firmware and does NOT prove
the measurement wiring is connected. These are diagnostic measurements, not
a battery SOC estimate or a safety gate. Real-voltage calibration remains a
vehicle/bench task.

## Calculation

Mean temperature is `sum(temperature_C * valid_running_ms) / valid_running_ms`.
Latest FB2 temperature is held until the next FB2 sample, for at most 300 ms.
The integral is updated on FB2 reception and GPS task polling. No frame means
no measurement: left and right validity are independent.

A mean is valid only after some covered time exists and at least 90% of the
observed timer-running interval has fresh temperature coverage. Missing/stale
intervals are not filled with zero and do not silently produce a full-lap mean.
The mean is computed from the covered part of an accepted lap, not a claim of
100% coverage. Temperature source resolution remains 1 C even though averages
and differences are encoded at 0.1 C.

`rise(lap N) = mean(lap N) - mean(lap N-1)` for the same controller.
Negative values mean cooling. Lap 1 has no valid rise. A missing/invalid previous
lap is not skipped: the next lap's rise also remains invalid for that side.
During a lap the running mean/rise updates; completed values are frozen.
These are differences between lap means, NOT within-lap start/end rises.

Start/Reset clears all statistics. Waiting for departure collects nothing.
Stop freezes them. Resume excludes paused time but continues the same lap.
Statistics begin when departure is confirmed and roll over when the GPS parser
detects a crossing. Unlike lap-time interpolation, temperature integration does
not retrospectively split samples at the interpolated crossing timestamp.
Boundary uncertainty includes the departure confirmation interval and the
GNSS reporting/parser latency; this is an analysis feature, not protection logic.

## CAN contract

Both frames: 29-bit Extended Classic CAN, DLC 8, little endian.
IDs are distinct from all IDs defined in this Cluster repository, including
the reset report `0x1CFDFFC0`; vehicle-wide allocation still needs coordination.

| ID | Name | Byte 2-3 | Byte 4-5 |
| --- | --- | --- | --- |
| `0x18FDFFC0` | Lap Controller Mean Temperature | Left mean x10 | Right mean x10 |
| `0x18FEFFC0` | Lap Controller Temperature Rise | Left rise x10 | Right rise x10 |

| Byte | Meaning |
| --- | --- |
| 0 | Lap number 1-51 |
| 1 bit 0 | Left mean valid |
| 1 bit 1 | Right mean valid |
| 1 bit 2 | Left rise valid |
| 1 bit 3 | Right rise valid |
| 1 bit 4 | Completed |
| 1 bit 5 | Active lap |
| 1 bit 6 | Timer running |
| 1 bit 7 | Timer paused |
| 2-3 | Left int16, 0.1 C/bit, offset 0 |
| 4-5 | Right int16, 0.1 C/bit, offset 0 |
| 6 | Same session byte as lap-time/battery telemetry |
| 7 | Same life counter as the corresponding lap-time/battery announcement |

Invalid raw is `INT16_MIN` (-32768, bytes `00 80`). Do not interpret it as a
temperature of -3276.8 C. Flags are authoritative.

Current lap: target 5 Hz. The latest nine completed laps: target 1 Hz each,
distributed over the second. The current lap plus those completed laps form
the recent ten-slot window. Older completed laps: target 0.1 Hz each, spread
over ten seconds. On lap 11, lap 1 slows down; on lap 12, lap 2 also slows down.
Final announcements follow the existing repeated-final path. Start/Reset still
clears the previous session. CAN congestion can delay these target rates.
Frames for mean and rise are transmitted with the corresponding lap history
announcement. A bounded 16-packet queue retries TWAI enqueue failures without
blocking, and drains at most four packets per 5 ms service. Prolonged CAN
congestion/offline operation can reduce rates; this is not guaranteed delivery.
Old pending packets are discarded on a session change.

## Monolith decoder settings

| Name (N=1..51) | ID | Unit | Multiplier | Offset | Bytes | Signed | Endian |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Lap N Time | `18FBFFC0` | Second (s) | 0.001 | 0 | 2-5 | No | Little |
| Lap N Left Controller Mean Temp | `18FDFFC0` | Lap Controller Mean Temperature (C) | 0.1 | 0 | 2-3 | Yes | Little |
| Lap N Right Controller Mean Temp | `18FDFFC0` | Lap Controller Mean Temperature (C) | 0.1 | 0 | 4-5 | Yes | Little |
| Lap N Left Controller Temp Rise | `18FEFFC0` | Lap Controller Temperature Rise (C) | 0.1 | 0 | 2-3 | Yes | Little |
| Lap N Right Controller Temp Rise | `18FEFFC0` | Lap Controller Temperature Rise (C) | 0.1 | 0 | 4-5 | Yes | Little |

Filters are payload-byte hex strings, using the same ordering as the supplied
Monolith JSON. `NN` means the lap number encoded as two hex digits (lap 51=33).

| Signal | Data Filter | Data Mask |
| --- | --- | --- |
| Time | `NN00000000000000` | `ff00000000000000` |
| Left mean | `NN01000000000000` | `ff01000000000000` |
| Right mean | `NN02000000000000` | `ff02000000000000` |
| Left rise | `NN04000000000000` | `ff04000000000000` |
| Right rise | `NN08000000000000` | `ff08000000000000` |

Example completed lap 2, left mean 53.3 C, right mean unavailable,
left rise -3.3 C, session 7, life 12:

```text
18FDFFC0: 02 15 15 02 00 80 07 0C
18FEFFC0: 02 15 DF FF 00 80 07 0C
```

Invalid temperatures and first-lap rises do not match their decoder filters.
Monolith may retain its last decoded value after a reset until its own display
timeout; an invalid/reset packet cannot force a conditional decoder to show
blank. Session separation must also be respected when interpreting saved logs.
The files preserve the supplied time/battery reset filtering behavior and
unrelated channels, except the obsolete EM LV decoder is removed to avoid
confusing it with the new VCU LV source. Some broad profiles retain other
legacy EM channels from the originals; these may be hidden/removed in Control
Hub if the energy meter is absent. Existing GNSS or
other decoder correctness was not re-audited as part of this temperature change.

## Generated configurations

Run `tools/update_monolith_lap_temperature.py` with an input directory containing
the three supplied files and an output directory. Originals are never modified.
Outputs in `docs/monolith/lap51/`:

- `316signals_RaceMonitor_Lap51_ControllerTemperature.json`
- `348signals_Field_Lap51_ControllerTemperature.json`
- `434signals_Full_Lap51_ControllerTemperature.json`

Each contains 51 lap-time signals, 51 existing battery-use signals, 102 lap
controller-mean signals and 102 rise signals, two VCU LV signals, plus its
original non-lap channels.
All lap-numbered signals above 51 are removed. JSON syntax, unit references and
temperature validity-filter matching are checked by the generator. Control Hub
import, live radio throughput and plotting hundreds of channels require a real
device check; firmware compilation is not proof of those UI behaviors.
