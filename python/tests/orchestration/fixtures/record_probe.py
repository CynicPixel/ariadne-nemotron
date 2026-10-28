"""F1 fixture recording, part 1: the adapter flags FM left open at G0 (D301), one short paid call each, on the ledger.

  NEBIUS_API_KEY from the key file at run time only; ARIADNE_LLM_RECORD=<dir> saves each response body (no headers).
  python tests/orchestration/fixtures/record_probe.py OUT.json

Per case: does the model call the tool, how many reasoning tokens, is there <think> text in content.
"""
import json
import sys

from orchestration.providers import load_config
from orchestration.providers.nebius import NebiusProvider

TOOLS = [{"type": "function", "function": {
    "name": "assign", "description": "Assign one exploration candidate to agent 0.",
    "parameters": {"type": "object", "properties": {"candidate": {"type": "integer"}, "reason": {"type": "string"}},
                   "required": ["candidate", "reason"]}}}]
MSGS = [{"role": "system", "content": "You pick exploration targets for a search robot. Always answer by calling assign."},
        {"role": "user", "content": "Candidates for agent 0: c0 = frontier 2.1 m away, 40 unknown cells, risk 0.1; "
                                    "c1 = frontier 6.3 m away, 12 unknown cells, risk 0.4. Call assign."}]
OFF = {"chat_template_kwargs": {"enable_thinking": False}}
NONE = {"reasoning_effort": "none"}
CASES = [
    ("super_required_thinkoff", "nvidia/nemotron-3-super-120b-a12b", "required", OFF),
    ("lightning_thinkoff", "nvidia/Nemotron-3_5-Lightning", "auto", OFF),
    ("lightning_effort_none", "nvidia/Nemotron-3_5-Lightning", "auto", NONE),
    ("nano_thinkoff", "nvidia/NVIDIA-Nemotron-3-Nano-30B-A3B", "auto", OFF),
    ("nano_effort_none", "nvidia/NVIDIA-Nemotron-3-Nano-30B-A3B", "auto", NONE),
    ("vl_thinkoff_text", "nvidia/Nemotron-3.5-Super-VL", "auto", OFF),
]

cfg = load_config()
prov = NebiusProvider({**cfg["nebius"], "budget": cfg["budget"], "prices_usd_per_mtok": cfg["prices_usd_per_mtok"]},
                      purpose="fixture")
out = []
for name, model, choice, extra in CASES:
    spec = {"model": model, "max_tokens": 300, "temperature": 0.0, "tool_choice": choice, "extra_body": extra}
    rec = {"case": name, "model": model, "tool_choice": choice, "extra_body": extra}
    try:
        resp, dt, usd = prov.chat(spec, MSGS, TOOLS, "fixture:probe")
        m = resp.choices[0].message
        det = resp.usage.completion_tokens_details
        rec.update(ok=True, latency_s=round(dt, 3), est_usd=usd, finish=resp.choices[0].finish_reason,
                   prompt_tokens=resp.usage.prompt_tokens, completion_tokens=resp.usage.completion_tokens,
                   reasoning_tokens=getattr(det, "reasoning_tokens", None) if det else None,
                   think_in_content="<think>" in (m.content or ""), content=(m.content or "")[:300],
                   reasoning_field=bool(getattr(m, "reasoning_content", None) or getattr(m, "reasoning", None)),
                   tool_calls=[{"name": c.function.name, "arguments": c.function.arguments} for c in m.tool_calls or []])
    except Exception as e:  # noqa: BLE001 - an unsupported flag is a result, not a crash
        rec.update(ok=False, error=f"{type(e).__name__}: {str(e)[:300]}")
    print(json.dumps({k: rec.get(k) for k in ("case", "ok", "latency_s", "finish", "reasoning_tokens",
                                                "think_in_content", "reasoning_field", "tool_calls", "error")}), flush=True)
    out.append(rec)
json.dump(out, open(sys.argv[1], "w"), indent=1)
