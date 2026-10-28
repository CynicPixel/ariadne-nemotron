"""G0 smoke: one live Token Factory call per tier (image + tool on VL, tool on Super), logged without the key."""
import base64, json, os, sys, time
import requests

BASE = "https://api.tokenfactory.nebius.com/v1"
out_dir, png = sys.argv[1], sys.argv[2]
key = os.environ["NEBIUS_API_KEY"]
H = {"Authorization": f"Bearer {key}", "Content-Type": "application/json"}

TOOLS = [{"type": "function", "function": {
    "name": "assign",
    "description": "Assign one exploration candidate to an agent.",
    "parameters": {"type": "object", "properties": {
        "agent": {"type": "string"}, "candidate_index": {"type": "integer"},
        "reason": {"type": "string"}}, "required": ["agent", "candidate_index", "reason"]}}}]
img = base64.b64encode(open(png, "rb").read()).decode()
TEXT = ("This is an occupancy map built by two ground robots (white free, black occupied, grey unknown). "
        "Candidates: 0 = frontier on the left edge, 1 = frontier on the right edge. "
        "Pick one for agent a0 by calling assign; give a one-sentence reason.")

cases = [
    ("vl_image_tool", "nvidia/Nemotron-3.5-Super-VL", {"reasoning_effort": "none", "messages": [{"role": "user", "content": [
        {"type": "text", "text": TEXT},
        {"type": "image_url", "image_url": {"url": f"data:image/png;base64,{img}"}}]}]}),
    ("super_tool", "nvidia/nemotron-3-super-120b-a12b", {"chat_template_kwargs": {"enable_thinking": False},
        "messages": [{"role": "user", "content": TEXT.replace("This is an occupancy map", "Imagine an occupancy map")}]}),
]
log = []
for name, model, extra in cases:
    body = {"model": model, "tools": TOOLS, "tool_choice": "auto", "max_tokens": 400, "temperature": 0.2, **extra}
    t0 = time.time()
    r = requests.post(f"{BASE}/chat/completions", headers=H, json=body, timeout=120)
    dt = time.time() - t0
    rec = {"case": name, "model": model, "http": r.status_code, "latency_s": round(dt, 3),
           "request_extra_keys": sorted(k for k in extra if k != "messages")}
    try:
        j = r.json()
    except ValueError:
        j = {"raw": r.text[:2000]}
    rec["response"] = j
    log.append(rec)
    msg = (j.get("choices") or [{}])[0].get("message", {}) if isinstance(j, dict) else {}
    print(name, model, r.status_code, f"{dt:.2f}s", "usage", j.get("usage") if isinstance(j, dict) else None)
    print("  tool_calls:", json.dumps(msg.get("tool_calls"))[:400])
    print("  content:", (msg.get("content") or "")[:200].replace("\n", " "))
    if r.status_code != 200:
        print("  error:", json.dumps(j)[:400])
json.dump({"base_url": BASE, "image": os.path.relpath(png, "/data/code/ariadne-nemotron"), "calls": log},
          open(os.path.join(out_dir, "g0_calls.json"), "w"), indent=1)
