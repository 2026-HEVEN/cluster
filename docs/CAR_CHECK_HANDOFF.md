# Car Check redesign and team handoff

Updated: 2026-09-09. PM feedback is implemented locally after user approval. VCU Car Check v1 is integrated against the pinned commit below. Hardware validation is pending. No message has been sent to another team. Current Korean report and handoff: `docs/ui_report/REPORT.md`, `docs/ui_report/CAN_DATA_REQUIRED.md`.

## Accepted UI decisions

- Assume the touchscreen works normally.
- Main page -> Warning reasons (only when warnings exist) -> Car Check menu.
- Car Check has four touch targets: top-left Motor, top-right Power, bottom-left VCU/Sensors, bottom-right GPS/RTK.
- Each target opens decoded measurements with units and diagnostic states, not hexadecimal CAN payload dumps.
- Detail back navigation returns to Car Check.
- GPIO13 momentary button is now HOME: pressing it from any page returns to the main page. Each non-home page also has a rectangular BACK touch target.
- Touching home WSS or throttle opens a continuously updated graph. Detail rows marked `*` also open graphs. Latest decision replaces the older 10-minute WSS buffer with 12 numeric channels at 2Hz for 60 seconds, fixed RAM, plus bounded RTK transitions and 32 boot-session events. There is no older-than-60-second scrollback in this version. Live display scheduling targets 20Hz.
- Display live zero as zero, missing values as --, and expired values as stale. Distinguish Cluster requests from VCU-confirmed applied states.
- Unspecified signals must not be invented or shown as measured zero. Integrate them after the user supplies the VCU team's confirmed CAN specification.

## VCU / torque-vectoring team handoff

The earlier new-sender backlog is superseded by the pinned VCU review below. Current Cluster receives all four Car Check frames. Confirm vehicle deployment and interpretation; request a new sender only for missing measurements such as calibrated brake pressure.

| Item | Request / confirmation needed |
| --- | --- |
| Four individual wheel speeds | 0x1806C0D0 is integrated, FL/FR/RL/RR, unsigned LE x0.1, 0xFFFF invalid. Confirm wheel calibration and physical sensor validity criteria. |
| Steering | 0x1804C0D0 is integrated, normalized x0.001, not degrees. Confirm deployed validity metadata and steering direction. |
| IMU | 0x1805C0D0 is integrated, yaw deg/s and accel X/Y g, x0.01. Confirm sensor mounting/vehicle-axis sign. |
| TV applied state | GPIO25 request remains 0x1801D0C0 Byte1 bit0. 0x1807C0D0 v1 echoes requests and reports active/block status. Verify deployed VCU version. |
| Regen applied state | Rotary 0=OFF, 1/2/3=the same ON; 0x1801D0C0 Byte1 bit1 is boolean enable. 0x1807C0D0 v1 reports permission/active/block status, not measured charging power. |
| Signal contract | For each signal supply ID, Extended/Standard, DLC, byte/bit positions, signedness, endian, scale/offset, unit, period, validity/sentinel, counter and timeout guidance. Intended consumers include both Cluster and TMA-1. |

Already received by Cluster: gear, brake, HV active, throttle with valid flag, Paddock applied feedback via 0x1801C0D0, and representative WSS speed via 0x1803C0D0. Do not request these as entirely new functions.

Controller bus/phase current, RPM, temperatures and faults originate in direct controller feedback. Add missing Cluster parsing/display instead of asking VCU to retransmit them. BMS and EM data also have independent sources.

Implemented: controller phase current, EM HV current/CPU temperature, four wheels, steering, IMU and TV/regen diagnostics. Brake pressure remains NOT PROVIDED. Missing/stale/invalid/legacy metadata are distinguished. HV SOC remains BMS-sourced; LV has voltage only. Current magnitude uses abs(EM V*I); charge/discharge direction remains unconfirmed.

## Wiring / PCB team handoff

| Function | GPIO / connection | Note |
| --- | --- | --- |
| TV ON/OFF switch | GPIO25 to GND | Internal pull-up; change PCB TC label to TV. |
| Regen rotary bit0 | GPIO27 | Internal pull-up, active-low. |
| Regen rotary bit1 | GPIO34 | External 10k pull-up to 3.3V, active-low. Proposal to move to GPIO13 was withdrawn. |
| Rotary common | GND | 0=OFF; 1/2/3 identical ON, not three strengths. |
| HOME momentary button | GPIO13 to GND | Internal pull-up. Label HOME for latest UI decision; old PCB label WARNING_DETAIL. |
| VESS PWM output | GPIO26 | Former DEBUG input; remain an output to VESS. |
| Paddock | GPIO36 | Keep its separate external pull-up. |

The UI redesign itself needs no extra analog wiring. EM voltage data arrives over the existing 250kbps CAN bus.

## Follow-up evidence: 2026-09-09

Read-only GitHub review of VCU dev commit `8498e2f20cfa79f8c71416a25a2676677b59a270`
(2026-09-08, combined test-week changes). No firmware or decoder JSON was changed in this review.

- Brake pressure is still absent: `src/core/board_pins.h` reserves GPIO26 as
  `BRAKE_PRESSURE_ADC_RESERVED`, while `brake_update()` reads GPIO33 digitally only
  if `BRAKE_SENSOR_INSTALLED` is true. That flag is currently false. `brake_compute()`
  maps its input 0/4095 to 0/100 percent; this is not a calibrated pressure measurement.
  Ask VCU for installed sensor identification, ADC acquisition, calibrated bar/MPa,
  invalid/fault handling and a published CAN contract, not merely a pressure decoder.
- IMPORTANT update to the earlier backlog: VCU now sends steering 0x1804C0D0,
  IMU 0x1805C0D0, four wheels 0x1806C0D0 and control diagnostics 0x1807C0D0.
  `send_sensor_telemetry()` is called by the 50ms scheduler task. See
  https://github.com/2026-HEVEN/vcu/blob/8498e2f20cfa79f8c71416a25a2676677b59a270/docs/CAR_CHECK_CAN.md
  These signals no longer require a request for a brand-new VCU sender. Local UI
  integration / existing remote Cluster changes must be reconciled separately.
- Steering numeric scaling remains signed int16 /1000 normalized (not degrees).
  IMU remains signed int16 /100, yaw deg/s and acceleration g in sensor axes.
  Byte6 validity bits and byte7 life are now part of both frames. Wheels use
  uint16 LE x0.1 km/h, FL/FR/RL/RR, with 0xFFFF invalid. Control byte0 is version 1;
  byte1 is echoed requests, byte2 observed states, bytes3/4 block reasons, byte7 life.
  Regen active is a command/pipeline observation, not measured recovered power.
- Supplied `107signals (1).json` has exactly 107 CAN decoders. Missing: EM RECORD
  0x1CF5FFC1 and SYNC 0x1CF6FFC1, four wheels/control diagnostics, steering/IMU validity
  and life, throttle/valid and applied Paddock in 0x1801C0D0, BMS valid/BLE/life and
  BMS detail 0x18F4FFC0. It contains Brake Active, not brake pressure. Existing
  TV/regen/Paddock entries on 0x1801D0C0 describe requests, not applied feedback.
  The Debug entry is legacy/reserved, not VESS PWM. Decoder existence is not proof
  of actual reception. Invalid wheel 0xFFFF must not display as 6553.5 km/h.
- HV SOC comes from BMS BLE (`bms_ble.cpp::apply_summary`), not EM. EM has HV V/I,
  LV supply V, CPU temperature, and separate sync/status; neither HV nor LV SOC is
  supplied by the cited EM contract. A voltage bar must not be labelled SOC.

### Include in the next report: interpretation and vehicle-test checklist

- PM PDF `BMS·motor-controller current based 10 kW limit` (2026-09-08), pp1-2/6:
  intended EM installation is the common HV path before the L/R controller split.
  Treat actual installation as a vehicle check, not proven by the document.
- Determine EM current polarity with known discharge/charge operation. Keep signed
  V*I for CHARGE/POWER display; the PDF's abs(V*I) limiter sketch loses direction.
- Confirm LV chemistry, voltage range and actual EM supply measurement point.
  LV SOC still requires an appropriate independent SOC source/estimator.
- Confirm steering left/right sign and IMU mounting axes; contract units are now
  known but vehicle-axis calibration is not. Verify wheel scaling and sensor validity.
- Determine power-bar full-scale separately for discharge/charge. PM's 10kW
  control target is not automatically the charge full-scale or proof of compliance.
- Measure actual EM CAN cadence/reception. Producer parser tests and successful
  firmware builds do not prove installed vehicle operation at 100Hz.
- PDF reports approximately 139ms median BMS CAN arrival on the analyzed run and
  slow value changes. Separate that observation from nominal 100ms Cluster scheduling.
