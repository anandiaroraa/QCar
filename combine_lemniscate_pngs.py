from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


SOURCE_DIR = Path("Tuning_lemniscate")
OUTPUT_PATH = SOURCE_DIR / "combined_lemniscate_1.8_2.0_2.2_exp1-3.png"

RADII = ["1.8", "2.0", "2.2"]
EXPERIMENTS = ["1", "2", "3"]


def load_font(size):
    for name in ("Arial.ttf", "Helvetica.ttc"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()


def main():
    files = [
        [SOURCE_DIR / f"lemniscate_{radius}_exp{exp}.png" for exp in EXPERIMENTS]
        for radius in RADII
    ]

    missing = [path for row in files for path in row if not path.exists()]
    if missing:
        raise FileNotFoundError("Missing files: " + ", ".join(str(path) for path in missing))

    images = [[Image.open(path).convert("RGB") for path in row] for row in files]
    tile_w, tile_h = images[0][0].size
    label_h = 54
    row_label_w = 92

    canvas = Image.new(
        "RGB",
        (row_label_w + tile_w * len(EXPERIMENTS), label_h + tile_h * len(RADII)),
        "white",
    )
    draw = ImageDraw.Draw(canvas)
    title_font = load_font(28)
    label_font = load_font(26)

    for col, exp in enumerate(EXPERIMENTS):
        x0 = row_label_w + col * tile_w
        draw.text(
            (x0 + tile_w / 2, label_h / 2),
            f"Exp {exp}",
            fill="black",
            font=title_font,
            anchor="mm",
        )

    for row, radius in enumerate(RADII):
        y0 = label_h + row * tile_h
        draw.text(
            (row_label_w / 2, y0 + tile_h / 2),
            f"r={radius}",
            fill="black",
            font=label_font,
            anchor="mm",
        )
        for col, image in enumerate(images[row]):
            x0 = row_label_w + col * tile_w
            canvas.paste(image, (x0, y0))

    canvas.save(OUTPUT_PATH)
    print(OUTPUT_PATH.resolve())


if __name__ == "__main__":
    main()
