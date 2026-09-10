"""Package native framebuffer previews with the Korean Markdown report.

Run after: pio test -e native -f test_diagnostics
Requires Pillow. No vehicle data or hardware photographs are generated.
"""
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs" / "ui_report"
NAMES = ["home", "menu", "motor", "power", "vcu", "gps", "sensors", "control",
         "reasons", "reasons_2", "links", "links_2", "events", "warning", "motor_wait", "home_wait",
         "graph_wss", "graph_bus", "graph_phase", "graph_motor_v", "graph_hv", "graph_lv",
         "graph_bms_v", "graph_em_a", "graph_throttle", "graph_rtk", "home_limits"]


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    font_path = Path("C:/Windows/Fonts/consola.ttf")
    font = ImageFont.truetype(str(font_path), 18) if font_path.exists() else ImageFont.load_default()
    for i, name in enumerate(NAMES):
        if i % 8 == 0:
            sheet = Image.new("RGB", (1000, 1592), "#eeeeee")
            draw = ImageDraw.Draw(sheet)
        source = ROOT / ".tmp" / f"ui_{name}.ppm"
        with Image.open(source) as original:
            assert original.size == (320, 240)
            colors = original.convert("RGB").getcolors(320 * 240)
            white = sum(count for count, color in colors if color == (255, 255, 255))
            assert 200 < white < 60000, f"Blank or saturated screen: {name}"
            original.resize((960, 720), Image.Resampling.NEAREST).save(OUT / f"ui_{name}.png")
            x, y = (i % 2) * 500 + 10, ((i % 8) // 2) * 398
            draw.text((x, y + 8), f"{i+1:02d}  {name.upper()}", font=font, fill="#111111")
            sheet.paste(original.resize((480, 360), Image.Resampling.NEAREST), (x, y + 34))
        if i % 8 == 7 or i == len(NAMES)-1:
            sheet.save(OUT / f"overview_{i//8+1}.png")
    with ZipFile(OUT / "HEVEN_UI_Report.zip", "w", ZIP_DEFLATED) as bundle:
        bundle.write(OUT / "REPORT.md", "REPORT.md")
        bundle.write(OUT / "CAN_DATA_REQUIRED.md", "CAN_DATA_REQUIRED.md")
        images = [OUT / f"ui_{name}.png" for name in NAMES]
        images += [OUT / f"overview_{i+1}.png" for i in range((len(NAMES)+7)//8)]
        for path in images:
            bundle.write(path, path.name)
    print(f"Verified {len(NAMES)} screen images; report bundle: {OUT / 'HEVEN_UI_Report.zip'}")


if __name__ == "__main__":
    main()
