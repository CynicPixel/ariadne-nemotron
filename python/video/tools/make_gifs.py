"""Devpost GIFs: standalone algorithm animations cut from the GIF-mode renders (ARIADNE_GIF=1: no chips, titles,
panels or subtitles), tightly cropped onto the film's flat background. Each spec is one GIF: the scene, a time window,
a playback speed-up, and the crop regions (in 1280x720 source pixels), laid out as rows that are stacked; regions in
one row sit side by side. The GIFs carry no labels; captions go where the GIFs are used.

  ARIADNE_GIF=1 python/video/render.sh scenes/<scene>.py <Class> -qm --media_dir media_gif     # per scene, once
  docker run --rm --user $(id -u):$(id -g) -v $PWD:/repo -w /repo/python/video --entrypoint python \
      manimcommunity/manim:v0.22.0 tools/make_gifs.py [name ...]
Output: build/video/gifs/<name>.gif (~720 px wide, 15 fps, looping)
"""
from __future__ import annotations

import sys
from pathlib import Path

import av
from PIL import Image

HERE = Path(__file__).resolve().parents[1]
OUT = HERE.parents[1] / "build/video/gifs"
BG = (14, 17, 22)  # style.BG
FPS, WIDTH, GAP = 15, 720, 24
T = 1280 / 480  # the crops were read off 480-px-wide contact sheets


def box(x0, y0, x1, y1):
    return tuple(round(v * T) for v in (x0, y0, x1, y1))


# name: (scene dir, class, t0, t1, speed, [[crop, ...], ...])
SPECS = {
    "feature_tracking_lk": ("s03_seeing", "Seeing", 10.3, 20.0, 1.0, [[box(95, 35, 385, 200)]]),
    "frontend_lk_fb_ransac_stereo": ("s04a_ba", "BundleAdjust", 8.5, 33.0, 2.5, [[box(40, 60, 440, 175)]]),
    "window_ba_schur": ("s04a_ba", "BundleAdjust", 35.0, 52.0, 1.8, [[box(35, 65, 400, 200)]]),
    "two_stage_place_recognition": ("s05_sharing", "Sharing", 14.0, 35.0, 2.2, [[box(45, 60, 420, 185)]]),
    "exchange_funnel_recorded": ("s05_sharing", "Sharing", 37.5, 48.5, 1.1, [[box(45, 65, 425, 195)]]),
    "spectral_lambda2_selection": ("s05_sharing", "Sharing", 49.0, 68.5, 2.0, [[box(50, 65, 420, 172)]]),
    "aliasing_known_entry_gate": ("s06_trust", "Trust", 1.0, 21.0, 2.0,
                                  [[box(70, 80, 153, 182), box(262, 84, 450, 174)]]),
    "chi2_pcm_gnc": ("s06_trust", "Trust", 22.0, 36.0, 1.5,
                     [[box(38, 105, 162, 188), box(178, 105, 302, 188)], [box(318, 105, 442, 188)]]),
    "cbba_task_allocation": ("s07_cbba", "SharingWork", 6.5, 35.5, 3.0,
                             [[box(45, 28, 435, 62)], [box(50, 62, 225, 215), box(270, 118, 410, 182)]]),
    "agentic_decision_chain": ("s08_agentic", "HumanInLoop", 0.6, 16.5, 1.6, [[box(25, 22, 455, 212)]]),
    "responder_width_dstar_lite": ("s09_thread", "Thread", 0.5, 11.0, 1.0, [[box(40, 38, 245, 213)]]),
    "splat_submap_assembly": ("s10_splat", "SharedModel", 5.5, 22.5, 1.8, [[box(120, 60, 350, 208)]]),
    "splat_orbit_worldgen": ("s10_splat", "SharedModel", 25.6, 32.6, 1.0, [[box(120, 55, 350, 208)]]),
}


def compose(frame: Image.Image, rows) -> Image.Image:
    rows_im = []
    for row in rows:
        parts = [frame.crop(c) for c in row]
        w = sum(p.width for p in parts) + GAP * (len(parts) - 1)
        h = max(p.height for p in parts)
        im = Image.new("RGB", (w, h), BG)
        x = 0
        for p in parts:
            im.paste(p, (x, (h - p.height) // 2))
            x += p.width + GAP
        rows_im.append(im)
    w = max(r.width for r in rows_im)
    h = sum(r.height for r in rows_im) + GAP * (len(rows_im) - 1)
    out = Image.new("RGB", (w, h), BG)
    y = 0
    for r in rows_im:
        out.paste(r, ((w - r.width) // 2, y))
        y += r.height + GAP
    pad = Image.new("RGB", (w + 2 * GAP, h + 2 * GAP), BG)  # a margin, so the content does not touch the edge
    pad.paste(out, (GAP, GAP))
    return pad


LIGHTER = {"splat_orbit_worldgen": (12, 600)}  # (fps, width) where the default would pass ~5 MB


def make(name: str) -> None:
    d, cls, t0, t1, speed, rows = SPECS[name]
    fps, width = LIGHTER.get(name, (FPS, WIDTH))
    src = av.open(str(HERE / "media_gif/videos" / d / "720p30" / f"{cls}.mp4"))
    vs = src.streams.video[0]
    fps_in = float(vs.average_rate)
    want = [t0 + i * speed / fps for i in range(int((t1 - t0) / speed * fps))]
    frames, k = [], 0
    for i, fr in enumerate(src.decode(vs)):
        while k < len(want) and int(want[k] * fps_in) == i:
            frames.append(compose(fr.to_image(), rows))
            k += 1
        if k >= len(want):
            break
    src.close()
    scale = width / frames[0].width
    size = (width, round(frames[0].height * scale)) if frames[0].width > width else frames[0].size
    frames = [f.resize(size, Image.LANCZOS) for f in frames]
    pal = frames[len(frames) // 2].quantize(colors=160, method=Image.Quantize.MEDIANCUT)
    q = [f.quantize(palette=pal, dither=Image.Dither.NONE) for f in frames]
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / f"{name}.gif"
    q[0].save(path, save_all=True, append_images=q[1:], duration=round(1000 / fps), loop=0, optimize=True)
    print(f"{name:32s} {size[0]}x{size[1]} {len(q) / fps:4.1f} s {path.stat().st_size / 1e6:4.1f} MB")


if __name__ == "__main__":
    for n in sys.argv[1:] or SPECS:
        make(n)
