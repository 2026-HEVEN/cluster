# Configurations without standalone validity decoders

- RaceMonitor: 317 channels, SOC included.
- Full: 430 channels, SOC included.

Standalone channel names containing the word `Valid` are removed. Existing
value decoders, including their validity filters, remain unchanged to avoid
displaying invalid sentinel temperatures or LV voltages as physical values.
BLE connection status and other non-validity diagnostics remain available.
Removing a decoder does not remove any CAN frames or reduce bus traffic.

Lap history now keeps the active lap at 5 Hz, the nine latest completed laps
at 1 Hz each, and older completed laps at 0.1 Hz each. These are target rates,
not guaranteed delivery during bus faults or saturation. Old lap values may
show missing in Control Hub if its display timeout is shorter than ten seconds.
The CAN IDs and byte layout are unchanged; the JSON needs no rate setting.

Generation: `tools/remove_monolith_valid.py`. Originals are preserved and all
settings except the explicitly removed channels are checked for equality.
