#!/usr/bin/env python3
"""Screen-record the live operator console during a fork live run (F5 beat D take).

  python3 python/video/tools/record_console.py --out build/video/takes/<name> [--url http://127.0.0.1:8080/]
                                               [--max-min 30]

Headless Chromium (Playwright) at 1920x1080 records the page to OUT/console.webm until the run ends (OUT/stop exists,
or the console stops answering after it was up, or --max-min). OUT/events.jsonl logs wall time (UTC) when the page
loads and whenever the Hypotheses panel or the approvals box changes, so the edit can find the beats.
No clicks are scripted: an approval is given by a person in their own browser on the same console, and the recording
shows its effect. Nothing here reads keys or run secrets.
"""
from __future__ import annotations

import argparse
import json
import shutil
import time
import urllib.request
from pathlib import Path


def up(url: str) -> bool:
    try:
        with urllib.request.urlopen(url, timeout=2) as r:
            return r.status == 200
    except OSError:
        return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--url", default="http://127.0.0.1:8080/")
    ap.add_argument("--max-min", type=float, default=30.0)
    ap.add_argument("--wait-min", type=float, default=10.0, help="how long to wait for the console to come up")
    a = ap.parse_args()
    from playwright.sync_api import sync_playwright

    a.out.mkdir(parents=True, exist_ok=True)
    ev = (a.out / "events.jsonl").open("a")

    def log(kind: str, **kw) -> None:
        ev.write(json.dumps({"t_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "t": round(time.time(), 2),
                             "event": kind, **kw}) + "\n")
        ev.flush()

    t0 = time.time()
    while not up(a.url):
        if time.time() - t0 > a.wait_min * 60:
            log("console_never_up")
            return 1
        time.sleep(2)
    with sync_playwright() as p:
        b = p.chromium.launch()
        ctx = b.new_context(viewport={"width": 1920, "height": 1080}, record_video_dir=str(a.out / "raw"),
                            record_video_size={"width": 1920, "height": 1080})
        page = ctx.new_page()
        page.goto(a.url, wait_until="domcontentloaded")
        log("page_loaded", url=a.url)
        last = {}
        t_start, down_since = time.time(), None
        while time.time() - t_start < a.max_min * 60 and not (a.out / "stop").exists():
            page.wait_for_timeout(1000)
            for sel in ("#approvals", "#hyp-counts", "#hyp-model"):
                try:
                    txt = page.inner_text(sel, timeout=500)
                except Exception:
                    continue
                if txt != last.get(sel):
                    last[sel] = txt
                    log("changed", panel=sel, text=txt[:400])
            if up(a.url):
                down_since = None
            elif down_since is None:
                down_since = time.time()
            elif time.time() - down_since > 10:
                log("console_down")
                break
        log("stop")
        video = page.video
        ctx.close()
        b.close()
        if video:
            shutil.move(video.path(), a.out / "console.webm")
    ev.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
