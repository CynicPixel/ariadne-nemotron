"""Regenerates hypotheses_sample.jsonl (the record shape for the console's Hypotheses panel, design §3.1): the
hypothesis layer replayed over llm_live_run5's recorded requests and slabs (no model call), plus one `targeted` event.
Keeps every event of one confirmed, one refuted and one still-open hypothesis.

  PYTHONPATH=. python tests/orchestration/fixtures/make_hypotheses_sample.py
"""
import json
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from test_verifier_hypotheses import HCFG, corpus_requests  # noqa: E402

from orchestration.hypotheses import HypothesisLayer  # noqa: E402

with tempfile.TemporaryDirectory() as tmp:
    ddir = Path(tmp) / "run" / "decisions"
    ddir.mkdir(parents=True)
    hl = HypothesisLayer(ddir, HCFG)
    reqs = corpus_requests(Path(tmp))
    for i, r in enumerate(reqs):
        hl.before_turn(r)
        if i == 2:   # an orchestrator answer that names an open hypothesis (as assign records it)
            h = next(h for h in hl.h.values() if h["status"] in ("unresolved", "supported") and h["anchors"])
            k, c = next(iter(h["anchors"].items()))
            hl.after_turn(r, [{"agent": int(k), "candidate": c, "hypothesis_id": h["hypothesis_id"],
                               "hypothesis_anchored": True,
                               "evidence_goal": "drive to the standoff and look past the frontier for a 0.8 m gap"}])
    ev = [json.loads(x) for x in (Path(tmp) / "run" / "hypotheses.jsonl").read_text().splitlines()]
final = {}
for e in ev:
    final[e["hypothesis_id"]] = e["status"]
targeted = next(e["hypothesis_id"] for e in ev if e["event"] == "targeted")
pick = [targeted] + [next(h for h, s in final.items() if s == want and h != targeted) for want in ("confirmed", "refuted")]
out = Path(__file__).parent / "hypotheses_sample.jsonl"
out.write_text("".join(json.dumps(e) + "\n" for e in ev if e["hypothesis_id"] in pick))
print(f"{out}: {sum(e['hypothesis_id'] in pick for e in ev)} lines, hypotheses {pick} -> "
      f"{[final[h] for h in pick]}")
