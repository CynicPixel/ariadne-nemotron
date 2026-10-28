# Video script (F6.1, draft)

Target **2:45** (hard limit under 3:00). **At least 60 s of live application footage**: beat D alone is 60 s; beats C and E add
console footage on top. Narration is about 350 words, on the fork's TTS (Ming US_James, `infra/scripts/kaggle.sh narration`; FM
queues it). Spoken text spells numbers out. Beat sources and their branding audit are in `docs/process/VIDEO_NOTES.md`.

**On-screen honesty rules**
- Every live clip carries a corner label: `LIVE · local run · <date UTC> · <speed>×`. Any speed-up is stated. A clip from a
  Kaggle GPU run says so.
- The model ID and latency chips are the console's own, read from the run trace. Nothing is overlaid by hand.
- Inherited Ariadne results are labelled "Ariadne (pre-Nemotron)". No Bedrock-era clip appears without an era label.
- Nothing from the old competition appears in any frame. The denylist is `tools/scrub/check.sh`. Beat 11's old footer and beat 8
  are excluded.

| beat | time | footage | narration status |
|---|---|---|---|
| A Open | 0:00–0:14 | old beat 1 (world generation), re-titled "Ariadne · evidence-seeking robots" | final |
| B Stack | 0:14–0:40 | old beats 3, 5, 6, cut short | final |
| C The question | 0:40–0:52 | live console, overview | final |
| D Live loop | 0:52–1:52 | **live console: hypotheses panel, model ID + latency, an approval click, a hypothesis settling** | lines D5–D6 depend on what the take shows |
| E What never comes from the model | 1:52–2:12 | live console + the gate/verifier diagram | final |
| F Results | 2:12–2:32 | the F4 table/plot | **pending F4** <!-- TODO(FM): F4 numbers --> |
| G Close | 2:32–2:45 | re-rendered architecture card (Token Factory band, era-labelled results), repo URL | final |

## Narration

**A Open** (old beat 1)
- A1: A building has collapsed. The lights are out, there is no G P S, and radios barely reach through the walls.
- A2: Before anyone goes in, a team of small robots does.

**B Stack** (old beats 3, 5, 6)
- B1: Each robot builds its own map from two cameras and a motion sensor.
- B2: They share only what is worth the bytes, and they refuse look-alike places that would tear the shared map apart.
- B3: The estimation, mapping and planning are our own code, and every number we show links to a recorded run.

**C The question** (live)
- C1: But a map is not a plan. Which opening should a robot check next, and why?
- C2: That is the job we gave to NVIDIA Nemotron, running on Nebius Token Factory.

**D Live loop** (live, 60 s; the narration covers about 45 s and the rest plays clean)
- D1: The planner proposes candidate goals. Its most promising frontiers become open hypotheses: does this gap lead somewhere a
  person could follow?
- D2: On every replan, Nemotron reads the candidates and the open hypotheses through tool calls, and assigns each robot a goal.
  It can name the hypothesis that goal should test.
- D3: On screen: the Nemotron model that answered, and how long it took.
- D4: That choice is only a proposal. A plain C plus plus gate checks it is one of the planner's own candidates, used exactly
  once, and anything risky waits for a person to approve it.
- D5 *(if the take shows a status change)*: The robot drives in, and new observations come back. The verifier, not the model,
  decides: this one is refuted. *(Use "supported" or "confirmed" if that is what the take shows.)*
- D6 *(if the next decision differs)*: And the next decision changes with it.

**E What never comes from the model** (live + diagram)
- E1: Three things never come from the model.
- E2: A hypothesis's status comes from what the robots actually observed. Escape routes come from the planner, checked at forty
  centimetres, the width of a responder. And spending has a hard cap.
- E3: If the model fails, the robots carry on with the classical planner.

**F Results** <!-- TODO(FM): two or three sentences from the F4 table only (B0 classical vs B2/B4 Nemotron; per seed,
descriptive). A negative or mixed result is said as it is. -->

**G Close**
- G1: Robots that look for evidence before people go in.
- G2: Nemotron on Nebius chooses; observed geometry decides.
- G3: The code is open, under the M I T licence.

Word count: about 300 without F. With F at about 40 words, about 340, or about 2:15 of speech in a 2:45 cut.

## Pacing: TTS batch 1 (Kaggle T4, 29 Oct; FM)

17 lines, 86.7 s in total. With a 0.4 s gap between lines, each beat's narration against its slot:

| beat | A | B | C | D (D1–D4) | E | G |
|---|---|---|---|---|---|---|
| narration | 9.8 s | 14.8 s | 9.3 s | 31.2 s | 16.3 s | 9.6 s |
| slot | 14 s | 26 s | 12 s | 60 s | 20 s | 13 s |

Every beat fits. Beat D keeps about 20 s of clean live footage after D5–D6.

**Batch 2 re-runs or adds these lines:**
- B3 and D2, reworded so they don't overclaim: we did not write gsplat, OpenCV or ROS, and `hypothesis_id` is optional in `assign`;
- D3, rephrased: the first wording ran at 278–293 wpm and failed the rate check;
- D5, D6 and F.

G2's brand names need a human listen, because Whisper heard "Nemetron / Nabius". If they sound wrong, spell them phonetically in
`spoken` only; the subtitle keeps the real names.
