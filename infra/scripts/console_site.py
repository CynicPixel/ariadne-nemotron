#!/usr/bin/env python3
"""Static build of the operator console for the hosted endpoint (CloudFront -> S3, no live worker).

Generates one scene through the real demo server, then mirrors the GET endpoints the frontend reads into
OUT/api/demo/; the frontend falls back to that snapshot when the live WebSocket is absent.

  (cd python/operator_console && npm ci && npm run build)
  docker run -d --rm --name console-site -u "$(id -u):$(id -g)" -v "$PWD":/work -w /work -e PYTHONPATH=/work/python \
    -e ARIADNE_DEMO=1 -e HOME=/tmp -p 127.0.0.1:18080:18080 --entrypoint python3 ariadne-console:1 \
    -m uvicorn operator_console.server:app --host 0.0.0.0 --port 18080 --workers 1
  python3 infra/scripts/console_site.py build/static_site "$PWD" maze_demo 6 && docker stop console-site
  # upload with explicit content types (status/entrances: application/json, dist: text/javascript), then invalidate
"""
import json, shutil, sys, time, urllib.request
from pathlib import Path

BASE = "http://127.0.0.1:18080/api/demo"
OUT = Path(sys.argv[1])
REPO = Path(sys.argv[2])
PRESET, SEED = sys.argv[3], int(sys.argv[4])


def req(path, method="GET", body=None, headers=None):
    r = urllib.request.Request(BASE + path, method=method, data=None if body is None else json.dumps(body).encode(),
                               headers={"Content-Type": "application/json", **(headers or {})})
    with urllib.request.urlopen(r, timeout=120) as f:
        return f.read()


tok = json.loads(req("/lease", "POST", {}))["token"]
req("/generate", "POST", {"preset": PRESET, "seed": SEED}, {"X-Controller-Token": tok, "Idempotency-Key": "export-gen"})
for _ in range(120):
    st = json.loads(req("/status"))
    if st["state"] in ("ready", "failed"):
        break
    time.sleep(1)
print("state:", st["state"], st.get("error"))
assert st["state"] == "ready"

api = OUT / "api/demo"
if OUT.exists():
    shutil.rmtree(OUT)
api.mkdir(parents=True)
# Status as a static site serves it: no worker, no controller, no session.
st.update(enabled=False, controller_remaining_s=0, commands=[], frame={}, session_id=None, state="offline", error=None)
st["links"] = {"github": "https://github.com/CynicPixel/Ariadne", "paper": None,
                "video": st["links"].get("video")}
(api / "status").write_text(json.dumps(st))
(api / "entrances").write_bytes(req("/entrances"))
(api / "world.obj").write_bytes(req("/world.obj"))
(api / "preview.png").write_bytes(req("/preview.png"))
web = REPO / "python/operator_console/static"
shutil.copy(web / "demo.html", OUT / "index.html")
shutil.copytree(web, OUT / "static", ignore=shutil.ignore_patterns("index.html", "splat.html", "demo.html"))
for p in sorted(OUT.rglob("*")):
    if p.is_file():
        print(f"{p.stat().st_size:>10}  {p.relative_to(OUT)}")
