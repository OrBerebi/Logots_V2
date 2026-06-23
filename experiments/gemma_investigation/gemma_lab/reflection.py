"""Run + score the reflection task.

Metrics that matter for swapping the cloud LLM (V1's Claude Haiku) for a local
Gemma:
  - json_validity_rate : fraction of replies that parse as JSON with a valid
                         action. A robot loop can't act on malformed output.
  - action_accuracy    : fraction matching the gold strategic action.
  - latency            : per-call wall time (the reflective loop budget was ~20 s
                         in V1, so even a slow local model has headroom).
"""
from __future__ import annotations

import json
import re
import time
from statistics import median

from . import config
from .backends import Backend
from .scenarios import SCENARIOS

_JSON_RE = re.compile(r"\{.*\}", re.DOTALL)


def parse_action(reply: str) -> tuple[str | None, bool]:
    """Return (action, json_ok). action is None if unparseable/invalid."""
    match = _JSON_RE.search(reply or "")
    if not match:
        return None, False
    try:
        obj = json.loads(match.group(0))
    except (json.JSONDecodeError, ValueError):
        return None, False
    action = obj.get("action") if isinstance(obj, dict) else None
    if action in config.VALID_ACTIONS:
        return action, True
    return None, True  # parsed JSON, but action invalid/missing


def run(backend: Backend) -> dict:
    rows = []
    for s in SCENARIOS:
        t0 = time.perf_counter()
        reply = backend.reflect(s.window)
        latency = time.perf_counter() - t0
        action, json_ok = parse_action(reply)
        rows.append({
            "id": s.id,
            "expected": s.expected_action,
            "predicted": action,
            "json_ok": json_ok,
            "valid_action": action is not None,
            "correct": action == s.expected_action,
            "latency_s": round(latency, 4),
            "tags": ",".join(s.tags),
            "raw": reply,
        })

    n = len(rows)
    lat = [r["latency_s"] for r in rows]
    summary = {
        "task": "reflection",
        "backend": backend.name,
        "model_id": backend.model_id,
        "n": n,
        "json_validity_rate": round(sum(r["valid_action"] for r in rows) / n, 3),
        "action_accuracy": round(sum(r["correct"] for r in rows) / n, 3),
        "latency_mean_s": round(sum(lat) / n, 3),
        "latency_median_s": round(median(lat), 3),
        "latency_max_s": round(max(lat), 3),
    }
    return {"summary": summary, "rows": rows}
