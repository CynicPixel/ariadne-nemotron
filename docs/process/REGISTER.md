# Completion register

**Rules:**
- One row per hard gate, invariant and plan item.
- A row changes only together with its evidence: a D-ID, a STATUS entry or a run directory.

**States:** `done` · `partial` · `deferred → X` · `not started` · `blocked (by)` · `FAIL / NEGATIVE` · `proposed (branch)`.

**Owners:**
- FM: fork admin, syncs, compute queue, GPU recording, Devpost.
- FP: provider, bench, hypotheses, verifier, eval.
- FV: scrub, README, console, video, copy.
- U: the user.

## Track hard gates (Nebius × NVIDIA, Physical AI)

| # | item | state | evidence / missing | owner |
|---|---|---|---|---|
| H1 | Runtime call to Nebius Token Factory | partial | G0 live calls (D301); in-loop calls pending F1 | FM/FP |
| H2 | ≥ 1 NVIDIA open model that materially drives robot behaviour | not started | F1 + F2 traces | FP |
| H3 | Public repo, OSS license visible in About | partial | private repo created 28 Oct; MIT in tree; flip public at G3 | U/FV |
| H4 | README: setup, how Nemotron and Token Factory are used | partial | README rewritten on fv/rebrand; 5 `TODO(FM)` (video, bench pick, impl check, F4, run cmd); `check.sh --release` blocks them | FV |
| H5 | Public YouTube video < 3:00, ≥ 60 s live modules, narrated Nebius/NVIDIA role | not started | F6.1 | FV/U |
| H6 | Project text + track = Physical AI | not started | F6.2 | FV |
| H7 | Additional Info form answered truthfully from measured data | not started | F6.2 | FM/U |
| H8 | Testing access through 15 Dec (zero-credential quickstart + bench) | not started | F3.4, F1.5 | FV/FP |
| H9 | Submitted (not a draft) before 30 Oct 22:30 IST | not started | F6.3 | U |

## Invariants (CLAUDE.md §2)

| # | item | state | evidence / missing | owner |
|---|---|---|---|---|
| I1 | No secret in tracked files, snapshots, S3 or job files | partial | tree + full history (790 commits) clean, D330; snapshots/S3/job files not covered by `check.sh` | all |
| I2 | No sim GT in model inputs | not started | F2 design review | FP |
| I3 | Nemotron claims ← logged live calls; Bedrock-era runs labelled | partial | lineage note `evaluation/runs/README.md`, README labels; F4 manifests missing | FP/FV |
| I4 | Egress at ρ_responder | done | inherited (Ariadne D-entries) | — |
| I5 | Old-competition denylist clean (`check.sh`) | done (fv/rebrand) | `check.sh` clean; D330; re-run after each sync | FV |

## Phase items

| # | item | state | evidence / missing | owner |
|---|---|---|---|---|
| F0.1 | Fork + remotes | done | STATUS 2026-10-28 | FM |
| F0.2 | Scaffolding | done | 5a8de3b, 6f3c25a, dd10b3f | FM |
| F0.3 | `integ/nemotron-base` | done | 140957f (S0) + ce351fb (S0.1); STATUS 2026-10-28 | M |
| F0.4 | Bootstrap build/models/frontend | done | 7.5 min; LiveLink+Egress 17/17; STATUS 2026-10-28 09:20 | FM |
| F0.5 | Nebius go/no-go + model IDs | done | D301; `evaluation/runs/nemotron/g0_20261028T205925Z/` | U/FM |
| F1.1–F1.6 | Nemotron provider, bench, tests | partial | provider + ledger + live.sh merged 7be5b02; 22/22 in `ariadne-orchestrator:nemotron-1`; bench and G1 live run pending | FP |
| F2.1–F2.7 | Evidence-seeking layer | not started | — | FP |
| F3.1–F3.4 | Scrub, deletions, references, README + lineage | done (fv/rebrand, not merged) | D330; STATUS 2026-10-29 02:50 | FV |
| F3.5 | Console hypotheses panel | done (fv/rebrand); browser layout unverified | 2b01a4e; STATUS 2026-10-29 02:58; tests/console 19 passed | FV |
| F4.1–F4.2 | Matched evaluation | not started | — | FP/FM |
| F5.1–F5.3 | Last sync + GPU recording | not started | — | FM |
| F6.1–F6.3 | Video, Devpost, submit | partial | script, DEVPOST draft, cue batch 1 (17); beat D footage, F4 numbers, video link open | FV/FM/U |
