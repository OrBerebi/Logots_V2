"""Aggregate task results into a markdown findings report + CSVs.

Pure formatting: it takes the dicts returned by reflection.run() / asr.run()
and writes outputs/report.md plus per-task result CSVs. No model logic here.
"""
from __future__ import annotations

import csv
import json

from . import config


def _write_csv(path, rows: list[dict]) -> None:
    if not rows:
        return
    # union of keys, stable order from the first row
    keys = list(rows[0].keys())
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)


def _kv_table(summary: dict) -> str:
    lines = ["| metric | value |", "|---|---|"]
    for k, v in summary.items():
        lines.append(f"| {k} | {v} |")
    return "\n".join(lines)


def build(reflection_res: dict, asr_res: dict, *, generated_at: str = "") -> str:
    """Write report.md + CSVs. Returns the markdown string."""
    rsum, asum = reflection_res["summary"], asr_res["summary"]

    _write_csv(config.RESULTS_DIR / "reflection_rows.csv", reflection_res["rows"])
    _write_csv(config.RESULTS_DIR / "asr_rows.csv", asr_res["rows"])
    (config.RESULTS_DIR / "summary.json").write_text(
        json.dumps({"reflection": rsum, "asr": asum}, indent=2)
    )

    backend = rsum["backend"]
    placeholder = (
        "\n> ⚠️  **Backend = `mock`** — these numbers validate the *harness*, "
        "not Gemma. Reflection uses keyword rules; ASR is a stub (WER is "
        "meaningless here). Set `config.BACKEND = \"gemma\"` after HF login for "
        "real results.\n"
        if backend == "mock" else ""
    )

    # worst reflection cases (for quick error inspection)
    misses = [r for r in reflection_res["rows"] if not r["correct"]]
    miss_md = "\n".join(
        f"- `{r['id']}` expected `{r['expected']}`, got `{r['predicted']}`"
        for r in misses
    ) or "- (none)"

    md = f"""# Gemma Investigation — Findings

**Question:** can one local **Gemma 4 E4B** replace BOTH of Logots V1's AI stages
— the reflective decision LLM (was cloud Claude Haiku) and ASR (was Whisper)?

- Backend: `{backend}`  ·  Model: `{rsum['model_id']}`
- Generated: {generated_at or "(unstamped)"}
{placeholder}
---

## 1. Reflection (strategic decision from an experience window)

{_kv_table(rsum)}

**Misclassified scenarios:**
{miss_md}

*Why these metrics:* the robot loop can only act on **valid JSON** (hence
`json_validity_rate`), the choice must be **right** (`action_accuracy`), and V1's
reflective budget was ~20 s/cycle so latency has wide headroom.

---

## 2. ASR (speech → text)

{_kv_table(asum)}

*Why these metrics:* `wer` vs the known TTS source text measures transcription
quality with zero manual labeling; `rtf_mean` (<1 = faster than real time) shows
whether it keeps up with the voice stream.

---

## 3. Interpretation

{_interpretation(backend, rsum, asum)}

---

*Artifacts: `outputs/results/reflection_rows.csv`, `outputs/results/asr_rows.csv`,
`outputs/results/summary.json`, `outputs/audio/*.wav`.*
"""
    config.REPORT_PATH.write_text(md)
    return md


def _interpretation(backend: str, rsum: dict, asum: dict) -> str:
    if backend == "mock":
        return (
            "Harness validated end-to-end: synthetic data → backend → scoring → "
            "report all run. Plug in Gemma (HF login + `BACKEND=\"gemma\"`) to get "
            "real quality numbers and fill in this section."
        )
    bullets = []
    bullets.append(
        f"- **Reflection:** {rsum['action_accuracy']:.0%} action accuracy at "
        f"{rsum['json_validity_rate']:.0%} JSON validity, "
        f"{rsum['latency_mean_s']}s mean latency."
    )
    bullets.append(
        f"- **ASR:** {asum['wer_mean']:.1%} mean WER, "
        f"RTF {asum['rtf_mean']} (×real-time)."
    )
    return "\n".join(bullets)
