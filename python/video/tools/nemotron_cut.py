"""The hackathon cut (F6.1; script docs/submission/VIDEO_SCRIPT.md, notes docs/process/VIDEO_NOTES.md): reused beats of the
Ariadne film with its burned-in subtitles masked, new cards, live console footage, and the new narration.

  python python/video/tools/nemotron_cut.py --film build/video_src/submission/video/ariadne_final_1080p60.mp4 \
      --narration <tts dir> [<tts dir> ...] [--live REC.mp4 --live-label "LIVE · local run · <date UTC>" --live-in S \
      --live-speed X] --out build/video/cut.mp4 [--preview] [--stills DIR] [--final]

Runs on the host with PyAV, Pillow and NumPy (no Docker). Every narration line is placed from its TTS manifest, and the
subtitle is that manifest's `spoken` text (so it always matches the audio), with spelled-out names put back for reading.
Later --narration dirs override earlier ones line by line (a re-run batch replaces its lines).

Honesty rules enforced here:
- Missing footage renders as a loud PLACEHOLDER card, and --final refuses to render while any placeholder or missing
  narration line remains, the cut is 3:00 or longer, or there is less than 60 s of live footage.
- Live footage always carries its label; a speed-up other than 1x is printed in it.
- Reused film beats carry a corner label saying they come from the pre-Nemotron film.
The old subtitle band (0.80 H to the bottom; measured on every reused cut, VIDEO_NOTES.md) is painted with the film's
background before the new subtitle is drawn, so no old caption survives.
"""
from __future__ import annotations

import argparse
import json
import sys
import textwrap
import wave
from fractions import Fraction
from pathlib import Path

import av
import numpy as np
from PIL import Image, ImageDraw, ImageFont

BG, TEXT, DIM, WARN, OK, BAD = (14, 17, 22), (229, 233, 240), (92, 99, 112), (230, 160, 40), (80, 190, 110), (225, 80, 70)
FONT = "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf"
BOLD = "/usr/share/fonts/truetype/noto/NotoSans-Bold.ttf"
MASK_TOP = 0.80          # old captions span 0.831-0.968 H on every reused cut; content ends by 0.796 H, except beat B1's
                         # small explanatory line (0.805-0.821 H), which the band removes whole rather than halving
MASK_RGB = (12, 16, 20)  # the film's background as decoded (median of the frame margins), so the band leaves no seam
SUB_Y = 0.90             # centre of the new subtitle
DISPLAY = {"G P S": "GPS", "C plus plus": "C++", "M I T": "MIT"}   # spoken spellings -> written form
SR = 44100
LEAD_S, GAP_S = 0.5, 0.4  # narration starts this far into its beat; pause between lines

# The cut, in order (VIDEO_SCRIPT.md). film: (in_s, out_s) on the old film; card: a still drawn below; live: n seconds of
# the recording. Beat A-B reuse the film; C-D are live; E-G are cards.
EDL = [
    ("A", "film", (3.0, 10.0)),     # the maze is generated
    ("A", "film", (18.5, 25.5)),    # the "Ariadne" title over the dimmed maze
    ("B", "film", (35.0, 44.0)),    # FAST/ORB tracks on recorded frames
    ("B", "film", (90.0, 98.0)),    # sharing over one radio budget
    ("B", "film", (157.0, 166.0)),  # look-alike corridors; the known-entry gate refuses the loop
    ("C", "live", 12.0),
    ("D", "live", 60.0),
    ("E", "card", 20.0),
    ("F", "card", 20.0),
    ("G", "card", 13.0),
]
FILM_LABEL = "from the Ariadne film (pre-Nemotron): recorded runs and illustrations"


def font(px: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(BOLD if bold else FONT, px)


def load_narration(dirs: list[Path]) -> dict[str, dict]:
    lines = {}
    for d in dirs:
        m = json.loads((d / "manifest.json").read_text())
        for ln in m["lines"]:
            w = d / "wav" / f"{ln['id']}.wav"
            with wave.open(str(w)) as f:
                assert (f.getnchannels(), f.getsampwidth(), f.getframerate()) == (1, 2, SR), f"{w}: not 16-bit mono {SR}"
                pcm = np.frombuffer(f.readframes(f.getnframes()), np.int16).astype(np.float32) / 32768
            lines[ln["id"]] = {"id": ln["id"], "beat": ln["beat"], "spoken": ln["spoken"], "ok": ln.get("ok", True),
                               "pcm": pcm, "dur": len(pcm) / SR, "src": str(d)}
    return lines


def display(spoken: str) -> str:
    for k, v in DISPLAY.items():
        spoken = spoken.replace(k, v)
    return spoken


def schedule(cues: list[dict], lines: dict, beats: dict) -> tuple[list[dict], list[str]]:
    """Lines in cue order inside their beat, LEAD_S into it, GAP_S apart. Returns (placed, missing ids)."""
    placed, missing, cursor = [], [], {}
    for c in cues:
        ln = lines.get(c["id"])
        if ln is None:
            missing.append(c["id"])
            continue
        t0 = cursor.get(c["beat"], beats[c["beat"]][0] + LEAD_S)
        placed.append({**ln, "t0": t0, "t1": t0 + ln["dur"]})
        cursor[c["beat"]] = t0 + ln["dur"] + GAP_S
    for p in placed:
        b0, b1 = beats[p["beat"]]
        if p["t1"] > b1:
            sys.exit(f"narration {p['id']} ends at {p['t1']:.1f} s, after its beat {p['beat']} ({b1:.1f} s)")
    return placed, missing


def card(W: int, H: int, beat: str) -> Image.Image:
    im = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(im)
    s = H / 1080
    if beat == "E":
        d.text((W * .08, H * .16), "Three things never come from the model", font=font(int(54 * s), True), fill=TEXT)
        rows = [("Hypothesis status", "set by the slab verifier from the occupancy the robots observed"),
                ("Escape routes", "computed by the planner, checked at 0.40 m responder clearance"),
                ("Spending", "a hard cap in the ledger; on any model failure, the classical planner takes over")]
        for i, (k, v) in enumerate(rows):
            y = H * (.36 + .15 * i)
            d.text((W * .08, y), k, font=font(int(40 * s), True), fill=OK)
            d.text((W * .08, y + 52 * s), v, font=font(int(32 * s)), fill=TEXT)
    elif beat == "G":
        d.text((W / 2, H * .30), "Ariadne", font=font(int(110 * s), True), fill=TEXT, anchor="mm")
        d.text((W / 2, H * .42), "evidence-seeking robot teams for collapsed structures", font=font(int(36 * s)), fill=DIM, anchor="mm")
        d.text((W / 2, H * .54), "NVIDIA Nemotron on Nebius Token Factory chooses · observed geometry decides",
               font=font(int(34 * s)), fill=TEXT, anchor="mm")
        d.text((W / 2, H * .66), "github.com/CynicPixel/ariadne-nemotron · MIT", font=ImageFont.truetype(
            "/usr/share/fonts/truetype/noto/NotoSansMono-Regular.ttf", int(32 * s)), fill=TEXT, anchor="mm")
    else:
        return placeholder(W, H, f"beat {beat}: results card (F4 numbers pending)")
    return im


def placeholder(W: int, H: int, what: str) -> Image.Image:
    im = Image.new("RGB", (W, H), (60, 12, 12))
    d = ImageDraw.Draw(im)
    s = H / 1080
    d.text((W / 2, H * .42), "PLACEHOLDER: NOT FOR RELEASE", font=font(int(70 * s), True), fill=(255, 220, 220), anchor="mm")
    d.text((W / 2, H * .55), what, font=font(int(36 * s)), fill=(255, 220, 220), anchor="mm")
    return im


def film_frames(path: Path, t0: float, dur: float, fps: int, size: tuple[int, int]):
    """One frame per output tick: the first decoded frame at or after the tick."""
    with av.open(str(path)) as c:
        v = c.streams.video[0]
        v.thread_type = "AUTO"
        c.seek(int(max(0.0, t0 - 1.0) / v.time_base), stream=v)
        ticks = [t0 + i / fps for i in range(round(dur * fps))]
        k = 0
        for f in c.decode(v):
            while k < len(ticks) and f.time >= ticks[k] - 0.5 / 60:
                yield f.to_image().resize(size, Image.BILINEAR)
                k += 1
            if k == len(ticks):
                return
    sys.exit(f"{path}: ended before {t0 + dur:.1f} s")


def overlay(im: Image.Image, t: float, seg: dict, placed: list[dict]) -> Image.Image:
    W, H = im.size
    s = H / 1080
    d = ImageDraw.Draw(im, "RGBA")
    if seg["kind"] == "film":
        d.rectangle([0, int(MASK_TOP * H), W, H], fill=MASK_RGB)  # the old burned-in subtitle band
        d.text((16 * s, 12 * s), FILM_LABEL, font=font(int(20 * s)), fill=DIM)
    if seg.get("label"):
        d.rounded_rectangle([12 * s, 10 * s, 12 * s + d.textlength(seg["label"], font=font(int(24 * s), True)) + 24 * s,
                             48 * s], radius=8 * s, fill=(0, 0, 0, 170))
        d.text((24 * s, 14 * s), seg["label"], font=font(int(24 * s), True), fill=WARN)
    for p in placed:
        if p["t0"] <= t < p["t1"]:
            txt = page(display(p["spoken"]), (t - p["t0"]) / (p["t1"] - p["t0"]))
            f = font(int(34 * s))
            box = d.multiline_textbbox((W / 2, SUB_Y * H), txt, font=f, anchor="mm", align="center", spacing=6 * s)
            d.rounded_rectangle([box[0] - 14 * s, box[1] - 8 * s, box[2] + 14 * s, box[3] + 8 * s], radius=8 * s,
                                fill=BG + (200,))
            d.multiline_text((W / 2, SUB_Y * H), txt, font=f, fill=TEXT, anchor="mm", align="center", spacing=6 * s)
    return im


def page(text: str, frac: float, width: int = 64) -> str:
    """The two-line page of `text` showing at fraction `frac` of its line, pages timed in proportion to their length."""
    rows = textwrap.wrap(text, width)
    pages = ["\n".join(rows[i:i + 2]) for i in range(0, len(rows), 2)]
    total, acc = sum(len(x) for x in pages), 0
    for pg in pages:
        acc += len(pg)
        if frac < acc / total:
            return pg
    return pages[-1]


def fade(im: Image.Image, t: float, seg: dict, ramp: float = 0.25) -> Image.Image:
    a = min(1.0, (t - seg["t0"]) / ramp, (seg["t1"] - t) / ramp)
    return im if a >= 1 else Image.blend(Image.new("RGB", im.size, BG), im, max(0.0, a))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--film", type=Path, required=True)
    ap.add_argument("--narration", type=Path, nargs="+", required=True)
    ap.add_argument("--cues", type=Path, default=Path("docs/submission/narration_cues.jsonl"))
    ap.add_argument("--live", type=Path)
    ap.add_argument("--live-label", default="")
    ap.add_argument("--live-in", type=float, default=0.0, help="seconds into the recording where beat C starts")
    ap.add_argument("--live-speed", type=float, default=1.0)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--preview", action="store_true", help="640x360 at 15 fps, fast encode")
    ap.add_argument("--stills", type=Path, help="also write the middle frame of every segment here (mask spot-check)")
    ap.add_argument("--final", action="store_true", help="refuse placeholders, missing lines, < 60 s live, >= 3:00")
    a = ap.parse_args()

    W, H, fps = (640, 360, 15) if a.preview else (1920, 1080, 30)
    if a.live and not a.live_label:
        sys.exit("--live needs --live-label (where and when it was recorded)")
    live_label = a.live_label + (f" · {a.live_speed:g}× speed" if a.live_speed != 1 else "")

    segs, t, live_cursor = [], 0.0, a.live_in
    for beat, kind, arg in EDL:
        dur = arg[1] - arg[0] if kind == "film" else arg
        seg = {"beat": beat, "kind": kind, "t0": t, "t1": t + dur}
        if kind == "film":
            seg["src"] = arg
        elif kind == "live" and a.live:
            seg.update(src=(live_cursor, live_cursor + dur * a.live_speed), label=live_label)
            live_cursor += dur * a.live_speed
        elif kind == "live":
            seg["kind"] = "placeholder"
        elif kind == "card" and beat == "F":
            seg["kind"] = "placeholder"
        segs.append(seg)
        t += dur
    total = t
    beats = {}
    for s_ in segs:
        b = beats.setdefault(s_["beat"], [s_["t0"], s_["t1"]])
        b[1] = s_["t1"]

    cues = [json.loads(x) for x in a.cues.read_text().splitlines() if x.strip()]
    lines = load_narration(a.narration)
    placed, missing = schedule(cues, lines, beats)
    failed = [p["id"] for p in placed if not p["ok"]]
    live_s = sum(s_["t1"] - s_["t0"] for s_ in segs if s_["kind"] == "live")
    holes = [f"{s_['beat']} ({s_['kind']})" for s_ in segs if s_["kind"] == "placeholder"]
    print(f"cut {total:.1f} s · live {live_s:.0f} s · narration {len(placed)} lines placed, missing {missing or 'none'}, "
          f"failed checks {failed or 'none'} · placeholders {holes or 'none'}")
    if a.final:
        why = [m for m, bad in [(f"placeholders {holes}", holes), (f"missing narration {missing}", missing),
                                (f"failed TTS checks {failed}", failed), (f"live {live_s:.0f} s < 60 s", live_s < 60),
                                (f"length {total:.1f} s >= 180 s", total >= 180)] if bad]
        if why:
            sys.exit("--final refused: " + "; ".join(why))

    a.out.parent.mkdir(parents=True, exist_ok=True)
    out = av.open(str(a.out), "w")
    vs = out.add_stream("libx264", rate=fps)
    vs.width, vs.height, vs.pix_fmt = W, H, "yuv420p"
    vs.options = {"crf": "28", "preset": "ultrafast"} if a.preview else {"crf": "18", "preset": "medium"}
    as_ = out.add_stream("aac", rate=SR)
    as_.layout = "mono"

    stills = []
    for seg in segs:
        n = round((seg["t1"] - seg["t0"]) * fps)
        if seg["kind"] == "film":
            src = film_frames(a.film, seg["src"][0], seg["t1"] - seg["t0"], fps, (W, H))
        elif seg["kind"] == "live":
            sp = a.live_speed
            src = (im for k, im in enumerate(film_frames(a.live, seg["src"][0], (seg["t1"] - seg["t0"]) * sp, fps, (W, H)))
                   if k % max(1, round(sp)) == 0) if sp >= 1 else film_frames(a.live, seg["src"][0], seg["t1"] - seg["t0"], fps, (W, H))
        elif seg["kind"] == "card":
            still = card(W, H, seg["beat"])
            src = (still.copy() for _ in range(n))
        else:
            still = placeholder(W, H, f"beat {seg['beat']}: " + ("live console footage pending" if seg["beat"] in "CD"
                                                                  else "results card (F4 numbers pending)"))
            src = (still.copy() for _ in range(n))
        for i, im in zip(range(n), src):
            tt = seg["t0"] + i / fps
            im = fade(overlay(im, tt, seg, placed), tt, seg)
            if a.stills and i == n // 2:
                stills.append((f"{seg['beat']}_{seg['kind']}_{seg['t0']:05.1f}s.png", im.copy()))
            fr = av.VideoFrame.from_image(im)
            fr.pts = round(tt * fps)
            fr.time_base = Fraction(1, fps)
            for pk in vs.encode(fr):
                out.mux(pk)
    for pk in vs.encode():
        out.mux(pk)

    mix = np.zeros(int(total * SR) + SR, np.float32)
    for p in placed:
        i0 = int(p["t0"] * SR)
        mix[i0:i0 + len(p["pcm"])] += p["pcm"]
    mix = np.clip(mix[: int(total * SR)], -1, 1)
    for i in range(0, len(mix), 1024):
        fr = av.AudioFrame.from_ndarray(mix[None, i:i + 1024], format="flt", layout="mono")
        fr.sample_rate, fr.pts, fr.time_base = SR, i, Fraction(1, SR)
        for pk in as_.encode(fr):
            out.mux(pk)
    for pk in as_.encode():
        out.mux(pk)
    out.close()

    if a.stills:
        a.stills.mkdir(parents=True, exist_ok=True)
        for name, im in stills:
            im.save(a.stills / name)
    tl = {"out": str(a.out), "length_s": round(total, 2), "live_s": live_s, "live_label": live_label or None,
          "segments": [{k: v for k, v in s_.items()} for s_ in segs],
          "narration": [{k: p[k] for k in ("id", "beat", "t0", "t1", "spoken", "src")} for p in placed],
          "missing": missing, "placeholders": holes}
    a.out.with_suffix(".timeline.json").write_text(json.dumps(tl, indent=1))
    print(f"wrote {a.out} and {a.out.with_suffix('.timeline.json')}")


if __name__ == "__main__":
    main()
