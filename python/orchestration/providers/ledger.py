"""The Token Factory spend ledger and its hard cap (D312; FM requirement 2026-10-29: the account has a limited credit budget).

One append-only JSON line per model call: ts_utc, model, prompt/completion tokens, est_usd, purpose. Before every call
the ledger is summed; at or above usd_hard the call is refused with BudgetExceeded, which the orchestrator turns into
"no assignment" (C++ falls back to the classical choice), never into a model fallback. est_usd is priced from the
config's $/1M table; a model missing from it is priced at the most expensive known in and out prices.

The file lives OUTSIDE the repo (default ~/.config/ariadne/nebius_spend.jsonl, env NEBIUS_SPEND_LEDGER overrides it:
the orchestrator container runs with HOME=/tmp, so live.sh bind-mounts the host file and sets the variable). Writes
take an exclusive flock and use O_APPEND, so a bench and a live run on the same host can both append.
"""
from __future__ import annotations

import fcntl
import json
import logging
import os
import time
from pathlib import Path

DEFAULT_PATH = "~/.config/ariadne/nebius_spend.jsonl"
log = logging.getLogger("orchestration.ledger")


class BudgetExceeded(RuntimeError):
    """The ledger total is at or above the hard cap: no further paid call is made."""


def ledger_path(cfg_path: str | None = None) -> Path:
    return Path(os.environ.get("NEBIUS_SPEND_LEDGER") or cfg_path or DEFAULT_PATH).expanduser()


class Ledger:
    def __init__(self, path: Path, prices: dict, usd_hard: float = 40.0, usd_warn: float = 25.0):
        if not prices:
            raise ValueError("ledger: no model prices configured; refusing to make unpriced calls")
        self.path, self.prices, self.usd_hard, self.usd_warn = Path(path), prices, float(usd_hard), float(usd_warn)
        self.warned = False

    def price(self, model: str) -> tuple[float, float]:
        """$/1M (in, out) for model; unknown -> the most expensive known in and out prices (FM rule)."""
        p = self.prices.get(model)
        if p is not None:
            return float(p["in"]), float(p["out"])
        return max(float(v["in"]) for v in self.prices.values()), max(float(v["out"]) for v in self.prices.values())

    def est_usd(self, model: str, prompt_tokens: int, completion_tokens: int) -> float:
        pin, pout = self.price(model)
        return (prompt_tokens * pin + completion_tokens * pout) / 1e6

    def total(self) -> float:
        """Sum of est_usd over every line. A line that does not parse is counted at the hard cap's value: an
        unreadable ledger must stop spending, not reset it."""
        if not self.path.exists():
            return 0.0
        s = 0.0
        with open(self.path) as f:
            fcntl.flock(f, fcntl.LOCK_SH)
            for line in f:
                if not line.strip():
                    continue
                try:
                    s += float(json.loads(line)["est_usd"])
                except (ValueError, KeyError, TypeError):
                    s += self.usd_hard
        return s

    def check(self) -> float:
        """Call before every paid request. Returns the current total; raises BudgetExceeded at the hard cap."""
        t = self.total()
        if t >= self.usd_hard:
            raise BudgetExceeded(f"budget_exceeded: ledger USD {t:.4f} >= hard cap {self.usd_hard:.2f} ({self.path})")
        if t >= self.usd_warn and not self.warned:
            self.warned = True
            log.warning("ledger USD %.4f >= warn %.2f (hard cap %.2f)", t, self.usd_warn, self.usd_hard)
            print(f"orchestrator: WARNING spend ledger USD {t:.4f} >= warn {self.usd_warn:.2f}", flush=True)
        return t

    def record(self, model: str, prompt_tokens: int, completion_tokens: int, purpose: str = "",
               latency_s: float | None = None) -> float:
        usd = self.est_usd(model, prompt_tokens, completion_tokens)
        rec = {"ts_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "model": model,
               "prompt_tokens": int(prompt_tokens), "completion_tokens": int(completion_tokens),
               "est_usd": round(usd, 8), "purpose": purpose}
        if latency_s is not None:
            rec["latency_s"] = round(latency_s, 3)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd = os.open(self.path, os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o600)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX)
            os.write(fd, (json.dumps(rec) + "\n").encode())
        finally:
            os.close(fd)
        return usd
