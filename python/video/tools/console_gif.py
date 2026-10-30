"""A looping GIF of a recorded console take (F6, Devpost): a crop of the console's map and Hypotheses columns, sped up,
with the speed printed on every frame.

  python python/video/tools/console_gif.py TAKE.webm OUT.gif [--start 400] [--speed 60] [--seconds 18] [--fps 8] \
      [--crop 10,40,1390,1000] [--width 640] [--label "Live run on maze_short_6"]

Runs on the host with PyAV and Pillow. The crop is in the take's own pixels (1920x1080 console, ?view=record layout).
"""
from __future__ import annotations

import argparse
from pathlib import Path

import av
from PIL import Image, ImageDraw, ImageFont

FONT = "/usr/share/fonts/truetype/noto/NotoSans-Bold.ttf"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("take", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--start", type=float, default=400.0, help="seconds into the take")
    ap.add_argument("--speed", type=float, default=60.0)
    ap.add_argument("--seconds", type=float, default=18.0, help="GIF length")
    ap.add_argument("--fps", type=int, default=8)
    ap.add_argument("--crop", default="10,40,1390,800", help="x0,y0,x1,y1 in the take's pixels")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--label", default="Live run on maze_short_6")
    ap.add_argument("--colors", type=int, default=128, help="palette size (fewer = smaller file)")
    a = ap.parse_args()

    box = tuple(int(v) for v in a.crop.split(","))
    w = a.width
    h = round(w * (box[3] - box[1]) / (box[2] - box[0]))
    ticks = [a.start + i * a.speed / a.fps for i in range(round(a.seconds * a.fps))]
    label = f"{a.label} · shown at {a.speed:g}× speed"
    f = ImageFont.truetype(FONT, 14)
    frames, k = [], 0
    with av.open(str(a.take)) as c:
        v = c.streams.video[0]
        v.thread_type = "AUTO"
        c.seek(int(max(0.0, a.start - 1.0) / v.time_base), stream=v)
        for fr in c.decode(v):
            if fr.time is None or fr.time < ticks[k]:
                continue
            im = fr.to_image().crop(box).resize((w, h), Image.LANCZOS)
            d = ImageDraw.Draw(im, "RGBA")
            tw = d.textlength(label, font=f)
            d.rounded_rectangle([6, 6, 18 + tw, 28], radius=5, fill=(0, 0, 0, 180))
            d.text((12, 8), label, font=f, fill=(230, 160, 40))
            frames.append(im.convert("P", palette=Image.ADAPTIVE, colors=a.colors))
            k += 1
            if k == len(ticks):
                break
    if k < len(ticks):
        raise SystemExit(f"{a.take}: ended after {k} of {len(ticks)} frames")
    a.out.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(a.out, save_all=True, append_images=frames[1:], duration=round(1000 / a.fps), loop=0, optimize=True)
    print(f"wrote {a.out}: {len(frames)} frames, {w}x{h}, {a.out.stat().st_size / 1e6:.1f} MB, "
          f"take {a.start:.0f}-{ticks[-1]:.0f} s at {a.speed:g}x")


if __name__ == "__main__":
    main()
