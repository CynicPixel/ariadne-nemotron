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
| H1 | Runtime call to Nebius Token Factory | not started | needs the key (F0.5) | FM/FP |
| H2 | ≥ 1 NVIDIA open model that materially drives robot behaviour | not started | F1 + F2 traces | FP |
| H3 | Public repo, OSS license visible in About | partial | private repo created 28 Oct; MIT in tree; flip public at G3 | U/FV |
| H4 | README: setup, how Nemotron and Token Factory are used | not started | F3.4 | FV |
| H5 | Public YouTube video < 3:00, ≥ 60 s live modules, narrated Nebius/NVIDIA role | not started | F6.1 | FV/U |
| H6 | Project text + track = Physical AI | not started | F6.2 | FV |
| H7 | Additional Info form answered truthfully from measured data | not started | F6.2 | FM/U |
| H8 | Testing access through 15 Dec (zero-credential quickstart + bench) | not started | F3.4, F1.5 | FV/FP |
| H9 | Submitted (not a draft) before 30 Oct 22:30 IST | not started | F6.3 | U |

## Invariants (CLAUDE.md §2)

| # | item | state | evidence / missing | owner |
|---|---|---|---|---|
| I1 | No secret in tracked files, snapshots, S3 or job files | partial | key kept outside the repo; secret scan in `check.sh` (F3.1) | all |
| I2 | No sim GT in model inputs | not started | F2 design review | FP |
| I3 | Nemotron claims ← logged live calls; Bedrock-era runs labelled | not started | F3.4 lineage note, F4 manifests | FP/FV |
| I4 | Egress at ρ_responder | done | inherited (Ariadne D-entries) | — |
| I5 | Old-competition denylist clean (`check.sh`) | not started | F3.1 | FV |

## Phase items

| # | item | state | evidence / missing | owner |
|---|---|---|---|---|
| F0.1 | Fork + remotes | done | STATUS 2026-10-28 | FM |
| F0.2 | Scaffolding | done | 5a8de3b, 6f3c25a, dd10b3f | FM |
| F0.3 | `integ/nemotron-base` | done | 140957f (S0) + ce351fb (S0.1); STATUS 2026-10-28 | M |
| F0.4 | Bootstrap build/models/frontend | done | 7.5 min; LiveLink+Egress 17/17; STATUS 2026-10-28 09:20 | FM |
| F0.5 | Nebius go/no-go + model IDs | blocked (user signup) | — | U/FM |
| F1.1–F1.6 | Nemotron provider, bench, tests | not started | — | FP |
| F2.1–F2.7 | Evidence-seeking layer | not started | — | FP |
| F3.1–F3.5 | Scrub, rebrand, console panel | not started | — | FV |
| F4.1–F4.2 | Matched evaluation | not started | — | FP/FM |
| F5.1–F5.3 | Last sync + GPU recording | not started | — | FM |
| F6.1–F6.3 | Video, Devpost, submit | not started | — | FV/FM/U |
