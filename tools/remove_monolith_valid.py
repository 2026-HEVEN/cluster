"""Remove standalone validity channels without changing value filters."""
import argparse
import copy
import json
import re
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("inputs", type=Path, nargs="+")
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for source in args.inputs:
        original = json.loads(source.read_text(encoding="utf-8-sig"))
        result = copy.deepcopy(original)
        channels = result["views"]["can"]["view"]
        removed = [name for name in channels if re.search(r"\bvalid\b", name, re.I)]
        for name in removed:
            del channels[name]
        expected = copy.deepcopy(original)
        for name in removed:
            del expected["views"]["can"]["view"][name]
        assert result == expected
        assert "BMS SOC" in channels
        assert not any(re.search(r"\bvalid\b", name, re.I) for name in channels)
        count = len(channels)
        suffix = source.name.split("signals_", 1)[1].removesuffix(".json")
        target = args.output_dir / f"{count}signals_{suffix}_NoValid.json"
        target.write_text(json.dumps(result, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")
        assert json.loads(target.read_text(encoding="utf-8")) == result
        print(f"{target}: {count} channels; removed {', '.join(removed)}; other settings unchanged")


if __name__ == "__main__":
    main()
