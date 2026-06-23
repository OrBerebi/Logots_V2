"""Run + score the ASR task.

Metrics for judging whether Gemma's native audio can replace V1's Whisper:
  - wer  : word error rate vs the known TTS source text (lower is better).
  - rtf  : real-time factor = processing_time / audio_duration (<1 = faster
           than real time). Matters for keeping up with the voice stream.
"""
from __future__ import annotations

import re
import time
from statistics import median

from .audio import AudioClip, build_dataset
from .backends import Backend

_PUNCT = re.compile(r"[^\w\s]")


def normalize(text: str) -> list[str]:
    """Lowercase, strip punctuation, collapse whitespace -> word list."""
    text = _PUNCT.sub(" ", (text or "").lower())
    return text.split()


def word_error_rate(reference: str, hypothesis: str) -> float:
    """Levenshtein distance over words, divided by reference length."""
    ref, hyp = normalize(reference), normalize(hypothesis)
    if not ref:
        return 0.0 if not hyp else 1.0
    # classic DP edit distance
    prev = list(range(len(hyp) + 1))
    for i, r in enumerate(ref, 1):
        curr = [i]
        for j, h in enumerate(hyp, 1):
            cost = 0 if r == h else 1
            curr.append(min(prev[j] + 1, curr[j - 1] + 1, prev[j - 1] + cost))
        prev = curr
    return prev[-1] / len(ref)


def run(backend: Backend, clips: list[AudioClip] | None = None) -> dict:
    if clips is None:
        clips = build_dataset()

    rows = []
    for c in clips:
        t0 = time.perf_counter()
        hyp = backend.transcribe(c.path)
        latency = time.perf_counter() - t0
        wer = word_error_rate(c.reference, hyp)
        rows.append({
            "id": c.id,
            "reference": c.reference,
            "hypothesis": hyp,
            "wer": round(wer, 3),
            "duration_s": c.duration_s,
            "latency_s": round(latency, 4),
            "rtf": round(latency / c.duration_s, 3) if c.duration_s else None,
        })

    n = len(rows)
    wers = [r["wer"] for r in rows]
    lat = [r["latency_s"] for r in rows]
    rtfs = [r["rtf"] for r in rows if r["rtf"] is not None]
    summary = {
        "task": "asr",
        "backend": backend.name,
        "model_id": backend.model_id,
        "n": n,
        "wer_mean": round(sum(wers) / n, 3),
        "wer_median": round(median(wers), 3),
        "exact_match_rate": round(sum(w == 0 for w in wers) / n, 3),
        "latency_mean_s": round(sum(lat) / n, 3),
        "rtf_mean": round(sum(rtfs) / len(rtfs), 3) if rtfs else None,
    }
    return {"summary": summary, "rows": rows}
