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

The scene sources were searched for the denylist, sponsor and provider names, links and logos. The rendered frames are **not yet
checked** [UNVERIFIED]. A contact sheet of the rendered film is needed before any reuse.

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

## Open

- Thumbnail: `docs/me/Ariadne_thumbnail.png` has no competition branding, but the logo carries a faint third-party stock-logo
  watermark. Use a clean thumbnail (a frame from the film) until the logo's licence is settled.
