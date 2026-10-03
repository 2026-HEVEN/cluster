# BMS SOC decoder configurations

These copies preserve the supplied RaceMonitor and Full configurations and add
or normalize these channels:

| Name | Extended CAN ID | Range | Type | Multiplier | Unit |
| --- | --- | --- | --- | --- | --- |
| BMS SOC | 0x18F3FFC0 | Byte 1 | unsigned byte | 1 | % |
| BMS Data Valid | 0x18F3FFC0 | Bit 0 | unsigned bit | 1 | State |
| BMS BLE Connected | 0x18F3FFC0 | Bit 1 | unsigned bit | 1 | State |

Offsets are zero. SOC uses little endian and no payload filter, so every BMS
status frame updates its display. Treat SOC as usable only while Data Valid is
1; the decoder does not itself hide stale or disconnected values. BLE Connected
alone is not a freshness guarantee.

Cluster already schedules `bms_can_tx_update` every 100 ms (10 Hz), with both
status and detail sent by `send_bms_status`. This change does not modify that
schedule or the BLE polling rate. Repeated telemetry is not a new BMS measurement.
Successful delivery still depends on CAN health, BMS freshness, and Monolith's
own receive/render behavior. JSON import and on-device rendering need bench
verification; this configuration cannot guarantee an absence of missing values.

The RaceMonitor copy has 319 channels; Full has 436. Full already contained
BMS SOC, so it is not duplicated. Other decoders, units, and settings are retained.

Generation and validation: `tools/add_monolith_soc.py`. Validation covers
unrelated-setting preservation, SOC values 0/1/78/100, validity flag combinations,
idempotence, and a JSON round trip.
