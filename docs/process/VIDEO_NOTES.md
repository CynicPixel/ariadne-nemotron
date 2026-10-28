# Video notes (FV, F6)

Target: about 2:45 total, under 3:00, with at least 60 s of live application footage. Labels are honest: footage is local (or
Kaggle GPU), and any speed-up is shown on screen. No old-competition identity in any frame, caption or title card; the denylist is
`tools/scrub/check.sh`. Narration is a new script on the fork's TTS (`infra/scripts/kaggle.sh narration`; FM queues the job).

## Sources

The old film and its beat sources left the tree in F3.2 and are recoverable at `7496988` (D330). They are restored under `build/`
only, never into the tree:

```bash
mkdir -p build/video_src && git archive 7496988 python/video/scenes submission/video | tar -x -C build/video_src
```

## Beat audit: old-competition branding (from the scene sources at `7496988`, 29 Oct 02:40 IST)

The scene sources were searched for the denylist, sponsor and provider names, links and logos. The rendered film was then checked
from contact sheets: one frame every 5 s over 300 s (61 frames), made with PyAV in a scratch venv into `build/video_src/sheet/`,
on 29 Oct at 02:50 IST. The results are in the next section.

| beat | source | branding found | reuse verdict |
|---|---|---|---|
| 1 world generation + title | `s01_worldgen.py` | none: the title is "Ariadne" plus a descriptive subtitle | reusable |
| 3 seeing | `s03_seeing.py` | none | reusable |
| 4a bundle adjustment | `s04a_ba.py` | none | reusable |
| 5 sharing | `s05_sharing.py` | none | reusable |
| 6 trust (known-entry gate) | `s06_trust.py` | none | reusable |
| 7 CBBA allocation | `s07_cbba.py` | none | reusable |
| 8 agentic loop | `s08_agentic.py` | No denylist hit, but every label is the **pre-fork Bedrock chooser**: an "AI on Bedrock" stage, a "final live run · Amazon Bedrock" chip, and narration naming Amazon Bedrock. | **Replace** with Nemotron footage (hypotheses, model ID and latency). Reuse only with an on-screen "earlier Bedrock-era chooser" label. |
| 9 thread | `s09_thread.py` | none | reusable |
| 10 splat | `s10_splat.py` | AWS g4dn training run, which is honest compute, not branding | reusable |
| 11 architecture + closing card | `s11_architecture.py` | **The footer prints the old competition's name (denylist).** The AWS band lists the old hosted console's CDN and "Amazon Bedrock". The results line is Bedrock-era ("80 of 80 picks"). The author card is fine. | **Do not reuse as rendered.** Re-render from an edited copy under `build/` (new footer, a Nemotron/Token Factory band, results labelled by era) or replace. |
| old narration and subtitles | the old cue sheet, outside the fork | read for tone only; no old-competition lines are copied | new script |

## Rendered film: frame check [VERIFIED at 5 s sampling]

- **The old narration is burned into every frame as subtitles**, in the bottom band. A beat cut from the mp4 therefore shows the
  old sentences under the new voice, so it cannot be reused as is. There are two ways round this:
  - **(a) Mask** (recommended: no render job). Paint the subtitle band with the film's flat background (`style.BG` = rgb 14,17,22;
    the scenes keep their content above the band by design), then burn in the new cues.
  - **(b) Re-render** the scene from `build/video_src/python/video/scenes/` with `python/video/render.sh`. That is a Docker
    Manim job, so it needs FM's queue.
- **Beat 8 (3:40–3:55)** shows "measured · final live run · Amazon Bedrock" and an "AI on Bedrock" stage, as the source audit
  found. It is excluded.
- **The architecture beat (4:30–5:00)** shows the AWS band with "Amazon Bedrock" and "CloudFront console". Its chips read "every
  number from the report's evaluation suite", and the report is gone from the tree. The footer with the old competition's name
  (from the source) fades in after 4:55, between samples. The beat is excluded; G uses a new card.
- **No other frame** in the 61 samples shows denylisted text, a logo or a sponsor mark. Between samples, the source audit is the
  evidence.

Usable cuts from the old film, by time code:

| use | old film time | content | note |
|---|---|---|---|
| A | 0:00–0:25 | world generation, the "Ariadne" title | mask subtitles |
| B1 | 0:30–0:45 | stereo cameras, FAST/ORB tracks on recorded frames | mask |
| B2 | 1:30–1:40 | sharing over one radio budget | mask |
| B2 | 2:35–2:50 | look-alike corridors, the map tears, the loop is refused | mask; chip "measured · recorded run · gate off" is accurate |
| spare | 3:55–4:05 | D* Lite repair around rubble | mask |
| spare | 4:10–4:25 | the shared Gaussian model assembling | mask |

## Open

- Thumbnail: `docs/me/Ariadne_thumbnail.png` has no competition branding, but the logo carries a faint third-party stock-logo
  watermark. Use a clean thumbnail (a frame from the film) until the logo's licence is settled.
