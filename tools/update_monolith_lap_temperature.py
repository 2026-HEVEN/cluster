"""Create non-destructive Monolith configuration copies with 51 lap statistics."""
import argparse
import json
import re
from pathlib import Path

MEAN_ID = 0x18FDFFC0
RISE_ID = 0x18FEFFC0


def decoder(can_id, lap, side, valid_bit, unit):
    return {
        "id": can_id, "multiplier": 0.1, "offset": 0, "unit": unit,
        "mode": "byte", "sign": True, "start": 2 + side * 2,
        "end": 3 + side * 2, "endian": "little",
        "filter": f"{lap:02x}{valid_bit:02x}000000000000",
        "mask": f"ff{valid_bit:02x}000000000000",
    }


def update(config):
    channels = config["views"]["can"]["view"]
    for name in list(channels):
        match = re.match(r"Lap (\d+)\b", name)
        if match and int(match.group(1)) > 51:
            del channels[name]
        elif (channels[name].get("id") == 0x1CF5FFC1 and
              channels[name].get("mode") == "byte" and
              channels[name].get("start") == 4 and channels[name].get("end") == 5):
            # Retired EM LV input is not the deferred VCU LV measurement.
            del channels[name]
    # Ensure original temperature channels exist without retransmitting FB2.
    for side, can_id in (("Left", 0x1802D0EF), ("Right", 0x1802D0F0)):
        channels[f"{side} Controller Temp"] = {
            "id": can_id, "multiplier": 1, "offset": -40,
            "unit": "Temperature", "mode": "byte", "sign": False,
            "start": 0, "end": 0, "endian": "little",
        }
    config["units"]["Lap Controller Mean Temperature"] = {
        "unit": "\u00b0C", "display": "Lap Controller Mean Temperature (\u00b0C)"}
    config["units"]["Lap Controller Temperature Rise"] = {
        "unit": "\u00b0C", "display": "Lap Controller Temperature Rise (\u00b0C)"}
    channels["VCU LV Supply Voltage"] = {
        "id": 0x1C09C0D0, "multiplier": 0.01, "offset": 0,
        "unit": "Volt", "mode": "byte", "sign": False,
        "start": 0, "end": 1, "endian": "little",
        "filter": "0000000001000000", "mask": "0000000001000000"}
    channels["VCU LV Sample Valid"] = {
        "id": 0x1C09C0D0, "multiplier": 1, "offset": 0,
        "unit": "State", "mode": "bit", "sign": False,
        "start": 32, "end": 32}
    for lap in range(1, 52):
        # Retain the original reset behavior for time/battery decoders: invalid
        # clear frames carry zero so a new session replaces their held values.
        channels[f"Lap {lap} Time"] = {
            "id": 0x18FBFFC0, "multiplier": 0.001, "offset": 0,
            "unit": "Second", "mode": "byte", "sign": False,
            "start": 2, "end": 5, "endian": "little",
            "filter": f"{lap:02x}00000000000000", "mask": "ff00000000000000"}
        for side, label in enumerate(("Left", "Right")):
            channels[f"Lap {lap} {label} Controller Mean Temp"] = decoder(
                MEAN_ID, lap, side, 1 << side, "Lap Controller Mean Temperature")
            channels[f"Lap {lap} {label} Controller Temp Rise"] = decoder(
                RISE_ID, lap, side, 4 << side, "Lap Controller Temperature Rise")
    return config


def validate(config):
    channels = config["views"]["can"]["view"]
    voltage = channels["VCU LV Supply Voltage"]
    assert voltage["id"] == 0x1C09C0D0 and voltage["multiplier"] == 0.01
    assert voltage["start"] == 0 and voltage["end"] == 1 and not voltage["sign"]
    wanted, mask = bytes.fromhex(voltage["filter"]), bytes.fromhex(voltage["mask"])
    for payload, valid in ((bytes.fromhex("b004e70a012a0000"), True),
                           (bytes.fromhex("00000000002a0000"), False)):
        assert all((a & b) == (c & b) for a, b, c in zip(payload, mask, wanted)) == valid
    assert int.from_bytes(bytes.fromhex("b004"), "little") * voltage["multiplier"] == 12.0
    for name, channel in channels.items():
        match = re.match(r"Lap (\d+)\b", name)
        assert not match or 1 <= int(match.group(1)) <= 51
        assert channel["unit"] in config["units"], name
        if channel["id"] in (MEAN_ID, RISE_ID):
            assert channel["sign"] and channel["multiplier"] == 0.1
            wanted = bytes.fromhex(channel["filter"])
            mask = bytes.fromhex(channel["mask"])
            assert len(wanted) == len(mask) == 8
            valid_bit = wanted[1]
            payload = bytes([wanted[0], valid_bit, 0, 0, 0, 0, 0, 0])
            assert all((a & b) == (c & b) for a, b, c in zip(payload, mask, wanted))
            invalid = bytes([wanted[0], 0, 0, 128, 0, 128, 0, 0])
            assert any((a & b) != (c & b) for a, b, c in zip(invalid, mask, wanted))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    files = {
        "110signals_RaceMonitor_Lap1-51_20261003.json": "RaceMonitor",
        "238signals_field_GNSS_Lap99_BatteryUsed_Temperature_20261003.json": "Field",
        "325signals_full_GNSS_Lap99_BatteryUsed_20261002.json": "Full",
    }
    for name, profile in files.items():
        original = json.loads((args.input_dir / name).read_text(encoding="utf-8-sig"))
        config = update(original)
        validate(config)
        count = len(config["views"]["can"]["view"])
        output = args.output_dir / f"{count}signals_{profile}_Lap51_ControllerTemperature.json"
        output.write_text(json.dumps(config, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")
        print(f"{output}: {count} CAN channels")


if __name__ == "__main__":
    main()
