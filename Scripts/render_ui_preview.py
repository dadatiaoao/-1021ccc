"""Make a contact sheet from the real dashboard renderer's host framebuffers."""
from pathlib import Path
from PIL import Image, ImageDraw

root = Path(__file__).resolve().parents[1]
names = ["home", "motor", "chassis", "arm", "limits", "gyro"]
image = Image.new("RGB", (1000, 906), "#282828")
draw = ImageDraw.Draw(image)
for i, name in enumerate(names):
    frame = Image.open(root / f"Output/host-tests/ui_{name}.pgm").convert("RGB")
    x, y = 10 + (i % 2) * 500, 30 + (i // 2) * 302
    image.paste(frame, (x, y))
    draw.text((x, y - 20), name, fill="white")
image.save(root / "Doc/UI_PREVIEW_2026-10-24.png")
Image.open(root / "Output/host-tests/ui_home.pgm").save(root / "Doc/UI_HOME_2026-10-24.png")
print("Saved Doc/UI_PREVIEW_2026-10-24.png (synthetic data, mounted viewing direction)")
