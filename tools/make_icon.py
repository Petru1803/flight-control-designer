"""Generates data/app.ico: a HUD-style flight-path marker (green on black), multi-resolution."""
import os

from PIL import Image, ImageDraw

GREEN = (51, 255, 51, 255)
BG = (0, 0, 0, 255)


def draw(size):
    s = size
    img = Image.new("RGBA", (s, s), BG)
    d = ImageDraw.Draw(img)
    w = max(1, round(s / 16))
    m = s * 0.06
    d.rectangle([m, m, s - m, s - m], outline=(31, 111, 31, 255), width=max(1, w // 2))
    cx, cy, r = s / 2, s * 0.52, s * 0.14
    # flight-path marker: circle with wings and fin
    d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=GREEN, width=w)
    d.line([cx - r - s * 0.24, cy, cx - r, cy], fill=GREEN, width=w)
    d.line([cx + r, cy, cx + r + s * 0.24, cy], fill=GREEN, width=w)
    d.line([cx, cy - r, cx, cy - r - s * 0.16], fill=GREEN, width=w)
    # pitch ladder ticks
    for k, y in enumerate((s * 0.2, s * 0.84)):
        d.line([s * 0.3, y, s * 0.42, y], fill=GREEN, width=max(1, w // 2 + (k == 0)))
        d.line([s * 0.58, y, s * 0.7, y], fill=GREEN, width=max(1, w // 2 + (k == 0)))
    return img


def main():
    out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data", "app.ico")
    big = draw(256)
    big.save(out, sizes=[(16, 16), (24, 24), (32, 32), (48, 48), (64, 64), (128, 128), (256, 256)])
    draw(256).save(out.replace(".ico", ".png"))
    print(out)


if __name__ == "__main__":
    main()
