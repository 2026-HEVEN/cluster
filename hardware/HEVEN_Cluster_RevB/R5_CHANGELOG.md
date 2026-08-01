# Rev.B Routing R5 changes

- Replaced the two locally rotated `C_0805` instances (`C_PWR_HF`, `D_TVS5`) with `C_0805_R180` geometry stored in the project footprint library.
- Preserved the physical pad positions and net locations while removing the top-level 180-degree local override.
- Added explicit B.Cu GND tracks and F.Cu/B.Cu stitching vias for the LV/power and CAN ground regions.
- Added a direct B.Cu GND route from SN65HVD230 pad 2 to the CAN ground stitching via.
- No signal-net routing, GPIO mapping, board outline, LCD placement, or switch placement was changed.

Run KiCad zone refill and DRC again. Gerber remains blocked until DRC reports 0 errors and 0 unconnected items.
