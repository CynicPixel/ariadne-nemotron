# ariadne-nemotron — phased plan (F0–F6)

Execution order for the fork. All times are IST. A phase opens on the previous gate, except
items tagged `∥`, which run in parallel sessions. An item is checked off only with its evidence
(REGISTER row + STATUS entry or run dir). Times re-baselined 28 Oct 09:00: the integration base
line (`integ/nemotron-base`, built by upstream M) is due at 12:00.

## F0 — Fork, scaffolding, go/no-go · FM · 28 Oct 04:30–10:00

- [x] **F0.1** Clone into `/data/code/ariadne-nemotron`; `upstream` = local Ariadne (push DISABLED); `origin` =
  `CynicPixel/ariadne-nemotron` (private).
- [ ] **F0.2** Scaffolding: `CLAUDE.md`, this plan, `REGISTER.md`, `STATUS.md`, D300 in `docs/decisions.md`, the
  design-doc skeleton `docs/design/nemotron-evidence-loop.md`.
- [ ] **F0.3** Integration base line `integ/nemotron-base` = main + perf tip (upstream M; ETA 12:00). It builds;
  `ctest -R 'LiveLink|Egress|HardPair'` is green.
- [ ] **F0.4** Bootstrap the clone on the base line: runtime build (its own `build/`), `models.sh`, console
  esbuild. Time it.
- [ ] **F0.5** Go/no-go: the user's Nebius key works from India. `GET /v1/models` IDs and the base host go in
  D301. One live `chat.completions` with an image and a tool call.

**Exit gate G0:** remotes verified; scaffolding committed; one logged live Nemotron response. Hard no-go
29 Oct 00:00 if Token Factory onboarding fails.

## F1 — Nemotron provider on Token Factory · FP · 28 Oct 09:00–14:00

- [ ] **F1.1** `python/orchestration/providers/`: the `Provider` interface; `bedrock.py` (moved out); `nebius.py`
  (OpenAI SDK).
- [ ] **F1.2** `configs/llm/nemotron.yaml` + `--llm-config`; the constants at `orchestrator.py:35-36` are removed;
  fallback on transport errors only.
- [ ] **F1.3** Adapter gotchas: reasoning vs `max_tokens`; `tool_choice` required → auto + nudge; images as a
  following user message; the `orchestrator` log keys (+`provider`).
- [ ] **F1.4** `orchestrator.Dockerfile` → image tag `ariadne-orchestrator:nemotron-1`; the `live.sh`
  `start_orchestrator` provider switch (`--env-file`).
- [ ] **F1.5** `python -m orchestration.bench`: decision replay over a recorded corpus (rebased paths,
  auto-verdict approval stub); picks the decision-tier model.
- [ ] **F1.6** Offline tests (recorded fixtures, fake-orchestrator cases).

**Exit gate G1 (14:00):** tests green; the bench table under `evaluation/runs/nemotron/bench_<ts>/`; a local live run
**on the base line** with `source: llm` from Nemotron; a 120 s run shows no OOM under the llm chooser.

## F2 — Evidence-seeking layer · FP · 28 Oct 14:00 – 29 Oct 04:00

- [ ] **F2.1** Slab export (`<did>_slab.png` + `slab{path,res_m,origin_xy_m,rows,cols}`), the only C++ edit;
  check that `ChooserLink` tolerates extra assignment fields.
- [ ] **F2.2** `hypotheses.py`: typed store + JSON schema; hypotheses are anchored to planner cluster/candidate IDs.
- [ ] **F2.3** `verifier.py`: deterministic, slab-only; responder-clearance crossing; unit tests on synthetic slabs.
- [ ] **F2.4** Async VL hypothesis worker; `list_hypotheses` MCP tool; `assign` takes `hypothesis_id` and `evidence_goal`.
- [ ] **F2.5** `hypotheses.jsonl` + `verifier.jsonl` traces aligned to `trace.jsonl`.
- [ ] **F2.6** Scene pre-screen over `maze_*` seeds (GT used for scene choice only; D31x).
- [ ] **F2.7** (optional) `source`/`ig_*` carried through `TaskOption`.

**Exit gate G2 (29 Oct 04:00):** one archived run shows hypothesis → decision → gate accept → motion → new
slab → status change (ideally a refutation) → a different next decision. If G2 slips past 07:00, seed
hypotheses deterministically.

## F3 — Clean tree, rebrand, console panel · FV ∥ · 28 Oct 09:00 – 29 Oct 12:00

- [ ] **F3.1** `tools/scrub/{rules.sed,apply.sh,check.sh}` (old-competition denylist + secret scan).
- [ ] **F3.2** Delete `submission/`, `docs/report/`, `infra/cool/`, the old film scenes; keep `python/video/{ariadne_video,tools}`.
- [ ] **F3.3** Fix the references: `console_site.py:52`, `runs_index.py:30`, `ariadne:24`, `demo.py` report link, the
  account ID in non-run scripts.
- [ ] **F3.4** Rewrite: README (Nemotron-on-Nebius architecture), architecture/decisions/algorithms/validation/results/
  agentic phrasing, code comments; `evaluation/runs/README.md` lineage note.
- [ ] **F3.5** Console Hypotheses panel (`GET /api/hypotheses`, status chips, provenance thumbnail, model+latency).

**Exit gate G3 (29 Oct 12:00):** `check.sh` clean; build + orchestration tests green; repo flipped public; pushed.

## F4 — Matched evaluation · FP + FM · 29 Oct 04:00–14:00

- [ ] **F4.1** B0 classical, B2 Nemotron (hypotheses off), B4 Nemotron + hypotheses; 2 scenes × 3 seeds × 90 s via
  slotq band 60+ (`6N-fm-<name>.job`); the key passed at run time only.
- [ ] **F4.2** Metrics: time to first verified resolution, egress-found time, path, fallback/invalid/out-of-set rates,
  gate rejections, latency p50/p95, tokens, cost. Per seed, descriptive.

**Exit gate G4 (14:00):** the table + plot in `evaluation/runs/nemotron/eval_<ts>/` with manifest and SHA256SUMS.

## F5 — Last sync + live GPU recording · FM · 29 Oct 12:00–22:00

- [ ] **F5.1** Sync S2 (last) at 12:00; re-scrub; targeted tests.
- [ ] **F5.2** AWS spot g4dn.xlarge (what launches; g5.2xlarge scores 1/10) via `live_aws.sh`: ROOT_GB=150,
  ship `static/dist`, `cpu_init` recon, quota release lag, cleared with M.
- [ ] **F5.3** At least 2 takes at RTF 1: the hypothesis panel, model ID/latency, a refuted hypothesis, a live operator
  directive or approval click.

**Exit gate G5 (22:00):** recording + trace archived; key rotated.

## F6 — Video, Devpost, submission · FV + FM · 29 Oct 22:00 – 30 Oct 18:00

- [ ] **F6.1** Video ~2:45 (≥ 60 s live), narration on the Nebius/Nemotron role; public on YouTube (user).
- [ ] **F6.2** `docs/submission/DEVPOST.md`, `FORM_ANSWERS.md` (measured data only; first commit 2026-09-06),
  judge bundle < 35 MB, tag `nebius-submission`.
- [ ] **F6.3** The user submits; verify in an incognito window.

**Exit gate G6:** submitted by 30 Oct 18:00. Buffer until 22:30.

## Standing rules

1. Upstream sync points: S0 (base line), S1 ~29 Oct 00:00 (optional), S2 29 Oct 12:00 (last). Afterwards,
   cherry-picks only, each with a stated need.
2. Cut order: Serverless Jobs eval → F2.7 → B1/B3/Ultra → console panel (reduced to a text overlay) → eval 1 scene →
   VL generation (replaced by deterministic seeding). Never cut: live Nemotron in the loop, ≥ 60 s live
   footage, public repo/MIT/README, the scrub gate, truthful form, submit by 18:00.
