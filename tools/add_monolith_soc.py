"""Add continuously decoded BMS SOC and its health signals to supplied configs."""
import argparse
import copy
import json
from pathlib import Path


def update(config):
    result = copy.deepcopy(config)
    channels = result["views"]["can"]["view"]
    result["units"].setdefault("Percent", {"unit": "%", "display": "Percent (%)"})
    result["units"].setdefault("State", {"unit": "State", "display": "State (State)"})
    channels["BMS SOC"] = {
        "id": 0x18F3FFC0, "multiplier": 1, "offset": 0,
        "unit": "Percent", "mode": "byte", "sign": False,
        "start": 1, "end": 1, "endian": "little",
    }
    for name, bit in (("BMS Data Valid", 0), ("BMS BLE Connected", 1)):
        channels[name] = {
            "id": 0x18F3FFC0, "multiplier": 1, "offset": 0,
            "unit": "State", "mode": "bit", "sign": False,
            "start": bit, "end": bit,
        }
    return result


def validate(original, result):
    channels = result["views"]["can"]["view"]
    changed = {"BMS SOC", "BMS Data Valid", "BMS BLE Connected"}
    for name, decoder in original["views"]["can"]["view"].items():
        if name not in changed:
            assert channels[name] == decoder, name
    for key, value in original.items():
        if key not in ("views", "units"):
            assert result[key] == value
    for key, value in original["views"].items():
        if key != "can":
            assert result["views"][key] == value
    for key, value in original["views"]["can"].items():
        if key != "view":
            assert result["views"]["can"][key] == value
    soc = channels["BMS SOC"]
    assert "filter" not in soc and "mask" not in soc
    for percent in (0, 1, 78, 100):
        payload = bytes([3, percent, 0, 0, 0, 0, 40, 255])
        assert payload[soc["start"]] * soc["multiplier"] + soc["offset"] == percent
    for flags in range(4):
        assert ((flags >> channels["BMS Data Valid"]["start"]) & 1) == (flags & 1)
        assert ((flags >> channels["BMS BLE Connected"]["start"]) & 1) == ((flags >> 1) & 1)
    assert update(result) == result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("inputs", type=Path, nargs="+")
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for source in args.inputs:
        original = json.loads(source.read_text(encoding="utf-8-sig"))
        result = update(original)
        validate(original, result)
        count = len(result["views"]["can"]["view"])
        suffix = source.name.split("signals_", 1)[1].removesuffix(".json")
        target = args.output_dir / f"{count}signals_{suffix}_SOC.json"
        target.write_text(json.dumps(result, ensure_ascii=True, indent=2) + "\n", encoding="utf-8")
        assert json.loads(target.read_text(encoding="utf-8")) == result
        print(f"{target}: {count} channels; preservation and decoder tests passed")


if __name__ == "__main__":
    main()
