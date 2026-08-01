# HEVEN Cluster Rev.B Routing R3

GitHub Actions R2 DRC report (2026-08-01) fixes:

- Moved H6 mounting hole from (235,85) to (238,85) to clear J_TMA_GPS.
- Removed the redundant 5V via overlapping ESP32 pad 19 and terminated the B.Cu 5V route at the PTH pad.
- Changed GND zones to remove isolated copper islands during refill.
- Marked ESP32 pad 14 as intentionally unused; ESP32 GND pads 20 and 26 remain connected.
- Added the local `HEVEN_Cluster_RevB.pretty` footprint library and `fp-lib-table`.
- Moved the right-panel silkscreen label away from the reserved-area rectangle.

This remains unapproved for fabrication until the cloud KiCad 10 DRC reports zero errors.
