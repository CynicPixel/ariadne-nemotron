# tools/scrub/rules.sed: removes the earlier competition's identity from the current tree (D300, D330).
#
# apply.sh runs this (GNU sed -E) over every tracked text file except evaluation/runs/ (evidence is byte-for-byte)
# and tools/scrub/, after every upstream merge. Each rule matches a whole phrase, so it cannot fire on unrelated
# text, and its output never matches its own pattern, so a second pass is a no-op. The denylist lives in check.sh;
# a rule here exists only for a line upstream also owns (a hand edit would be lost when a sync takes upstream's side).

# --- live-demo mode: the configs/mission/live_*demo*_v1.yaml headers and the ariadne_live.cpp option comment
s/The judges-live demo/The live demo/g
s/judges-live mode/live demo mode/g

# --- docs/architecture.md: the old challenge brief, link and award phrasing
s/which is the Agentic Vision "trace" deliverable\./which is the decision-trace deliverable./
s/^- Agentic Vision requires vision output/- The agentic layer requires vision output/
s/^- competition file \/ challenge information captured in `opencv-ai-competition-2026\(1\)\.md`$/- the earlier challenge brief (not part of this repository)/
/^- official challenge: https:\/\/opencv26\.devpost\.com\/$/d

# --- docs/design/live-runtime.md: the demo video has no default and the paper link is gone (demo.py)
s|; defaults to the published film \(https://youtu\.be/yrbcvQKnfkA\)\.|; unset by default, which hides the link.|
s|^GitHub points to the repository remote\. Paper uses `docs/report/main\.pdf` if present;$|GitHub points to the repository remote. There is no paper link;|
s|^that file currently belongs to uncommitted paper work\. Video is$|the video link is|

# --- ariadne CLI: the hosted frontend, and the COOL protocol whose scripts (infra/cool/) are deleted
s| \(its frontend is hosted at https://www\.ariadne\.cynicpixel\.xyz/\)$||
/^#   \.\/ariadne cloud cool +COOL protocol /d
/^ +cool\) +\$S\/spinup\.sh run --profile cool .*infra\/cool\/day1\.sh/d
# usage() printed a fixed line range; read up to the closing rule line instead, so deleted lines cannot shift it
s|^usage\(\) \{ sed -n '3,[0-9]+p' "\$0"|usage() { sed -n '3,/^# ---/{/^# ---/!p}' "$0"|

# --- the run index moves out of the deleted submission/ package into build/ (not committed)
s|rebuild submission/runs/runs\.csv|rebuild build/runs/runs.csv|
s|\[--out submission/runs/runs\.csv\]|[--out build/runs/runs.csv]|
s|default="submission/runs/runs\.csv"|default="build/runs/runs.csv"|
s|`submission/runs/runs\.csv` lists every AWS run|`build/runs/runs.csv` (written by `./ariadne runs`) lists every AWS run|

# --- infra/scripts/console_site.py (edited, never run: it deploys the upstream site): no paper to copy
/^shutil\.copy\(REPO \/ "submission\/report\/ariadne_technical_report\.pdf", api \/ "paper"\)$/d
s|^(st\["links"\] = \{"github": "https://github\.com/CynicPixel/Ariadne", )"paper": "/api/demo/paper",$|\1"paper": None,|

# --- python/operator_console/demo.py: no default film link, no paper (the /paper route is removed by hand), and the
#     console's GitHub link names this repository
s|^VIDEO_URL = "https://youtu\.be/yrbcvQKnfkA"$|VIDEO_URL = None  # set ARIADNE_DEMO_VIDEO_URL to show the film link|
s|^( +)"paper": "/api/demo/paper" if \(self\.root / "docs/report/main\.pdf"\)\.is_file\(\) else None,$|\1"paper": None,|
s|^( +"links": \{"github": )"https://github\.com/CynicPixel/Ariadne",$|\1"https://github.com/CynicPixel/ariadne-nemotron",|

# --- python/video/tools/make_gifs.py: GIFs are build outputs now (not committed)
s|^one row sit side by side\. Labels move to submission/media/README\.md\.$|one row sit side by side. The GIFs carry no labels; captions go where the GIFs are used.|
s|^Output: submission/media/gifs/<name>\.gif |Output: build/video/gifs/<name>.gif |
s|^OUT = HERE\.parents\[1\] / "submission/media/gifs"$|OUT = HERE.parents[1] / "build/video/gifs"|

# --- infra/scripts/spinup.sh: the COOL AMI profile stays; its day-1 script is gone
s|; needs the subscription \(infra/cool/day1\.sh\)\.$|; needs the Marketplace subscription.|

# --- docs/README.md, docs/build.md, evaluation/README.md: rows for deleted directories
/^\| \[`report\/`\]\(report\/\) \| LaTeX sources of the technical report/d
/^\| `\.\/ariadne cloud cool` \| /d
/^\| `submission\/` \| competition deliverables /d
s/^(\| `infra\/` \| Dockerfiles, run scripts, AWS launchers, Terraform), COOL protocol \|/\1 |/
s|\(stock OpenCV 5\.0\.0; COOL protocol: `infra/cool/`\)|(stock OpenCV 5.0.0)|
