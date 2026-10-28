"""LLM providers for the chooser (D310, D311). The model chain, prices and caps come from configs/llm/*.yaml."""
from __future__ import annotations

from pathlib import Path

import yaml

from .base import Provider
from .ledger import BudgetExceeded

DEFAULT_CONFIG = Path(__file__).resolve().parents[3] / "configs" / "llm" / "nemotron.yaml"


def load_config(path: str | Path | None = None) -> dict:
    p = Path(path) if path else DEFAULT_CONFIG
    cfg = yaml.safe_load(p.read_text())
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
