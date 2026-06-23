#!/usr/bin/env python3
"""End-to-end driver: build synthetic data, run both tasks, write the report.

    python run_all.py            # uses config.BACKEND (default: "mock")
    BACKEND=gemma python run_all.py   # real Gemma run (needs HF login)

Mirrors what investigate.ipynb does, for headless / CI use.
"""
from __future__ import annotations

import os
from datetime import datetime

from gemma_lab import asr, audio, backends, config, reflection, report

# allow env override without editing config.py
config.BACKEND = os.environ.get("BACKEND", config.BACKEND)


def main() -> None:
    print(f"Backend: {config.BACKEND}  (model: {config.MODEL_ID})")
    backend = backends.get_backend()

    print("• Reflection task ...")
    refl = reflection.run(backend)
    print(f"  json_validity={refl['summary']['json_validity_rate']}  "
          f"action_accuracy={refl['summary']['action_accuracy']}")

    print("• Synthesizing speech + ASR task ...")
    clips = audio.build_dataset()
    print(f"  {len(clips)} clips via voice '{audio.pick_voice() or 'default'}'")
    asr_res = asr.run(backend, clips)
    print(f"  wer_mean={asr_res['summary']['wer_mean']}  "
          f"exact_match={asr_res['summary']['exact_match_rate']}")

    stamp = datetime.now().strftime("%Y-%m-%d %H:%M")
    report.build(refl, asr_res, generated_at=stamp)
    print(f"\n✓ Report written to {config.REPORT_PATH}")


if __name__ == "__main__":
    main()
