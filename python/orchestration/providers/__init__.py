"""LLM providers for the chooser (D310, D311). The model chain, prices and caps come from configs/llm/*.yaml."""
from __future__ import annotations

from pathlib import Path

import yaml

from .base import Provider
from .ledger import BudgetExceeded

DEFAULT_CONFIG = Path(__file__).resolve().parents[3] / "configs" / "llm" / "nemotron.yaml"


def _merge(base: dict, over: dict) -> dict:
    out = dict(base)
    for k, v in over.items():
        out[k] = _merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    return out


def load_config(path: str | Path | None = None) -> dict:
    """`extends: <file>` (relative to this file) loads that config first and deep-merges this one over it (dicts
    merge; lists and scalars replace), so an ablation config states only what it changes."""
    p = Path(path) if path else DEFAULT_CONFIG
    cfg = yaml.safe_load(p.read_text())
    if cfg.get("extends"):
        base = load_config(p.parent / cfg.pop("extends"))
        base.pop("_path", None)
        cfg = _merge(base, cfg)
    chain = cfg.get("decision", {}).get("chain") or []
    if not chain:
        raise ValueError(f"{p}: decision.chain is empty")
    for s in chain:
        if not s.get("model") or not s.get("provider"):
            raise ValueError(f"{p}: every decision.chain entry needs provider and model")
    cfg["_path"] = str(p)
    return cfg


def make_provider(name: str, cfg: dict, **kw) -> Provider:
    """One provider instance per name; its own section of the config (cfg[name]) plus the shared budget/prices."""
    sect = {**cfg.get(name, {}), "budget": cfg.get("budget", {}), "prices_usd_per_mtok": cfg.get("prices_usd_per_mtok", {})}
    if name == "nebius":
        from .nebius import NebiusProvider
        return NebiusProvider(sect, **kw)
    if name == "bedrock":
        from .bedrock import BedrockProvider
        return BedrockProvider(sect, **kw)
    raise ValueError(f"unknown provider {name!r}")


__all__ = ["BudgetExceeded", "DEFAULT_CONFIG", "Provider", "load_config", "make_provider"]
