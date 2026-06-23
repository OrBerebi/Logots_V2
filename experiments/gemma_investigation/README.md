# Gemma Investigation — reflection + ASR on one local model

A small, reproducible experiment answering a single question for Logots V2:

> Can **one local Gemma 4 E4B** replace **both** of V1's AI stages — the
> reflective decision LLM (V1: cloud **Claude Haiku**) and ASR (V1: **Whisper
> tiny**)?

Gemma 4's small variants (E2B/E4B) are multimodal with **native audio input**,
so a single `any-to-any` Transformers pipeline does strategic reasoning *and*
speech-to-text. This harness measures whether that holds up — with **no robot
and no real recordings**; all inputs are synthetic.

## Why this maps cleanly onto V1

| V1 stage | V1 model | This experiment |
|---|---|---|
| Reflective decision (~20 s loop) | Claude Haiku (cloud API) | Gemma 4 E4B (local), same JSON-action contract |
| `trans_audio_transcribe` (ASR) | Whisper tiny (MLX) | Gemma 4 E4B audio (same model) |

See [`../../archive/docs/logots_v1_guide.md`](../../archive/docs/logots_v1_guide.md) §4.4 (reflection) and §4.2 (ASR).

## Layout

```
gemma_investigation/
├── environment.yml      dedicated conda env (latest transformers for Gemma 4)
├── investigate.ipynb    ⭐ self-contained walk-through: all code + explanations
│                           inline, real outputs baked in (the thing to show Darab)
├── build_notebook.py    regenerates investigate.ipynb from source
├── run_all.py           headless driver (same logic, scripted): data → tasks → report
├── gemma_lab/
│   ├── config.py        model switch, action vocab, prompts, audio contract
│   ├── scenarios.py     synthetic plant-care experience-windows + gold actions
│   ├── audio.py         `say` → 16 kHz/mono/float32 WAV + ground-truth transcripts
│   ├── backends.py      MockBackend (no download) + GemmaBackend (real)
│   ├── reflection.py    run + score (JSON validity, action accuracy, latency)
│   ├── asr.py           run + score (WER, RTF)
│   └── report.py        → outputs/report.md + CSVs
└── outputs/             generated (gitignored)
```

## How to run

**The notebook is the main artifact** — self-contained, every step's code and
explanation inline, with real outputs already baked in, so it reads without
running anything.

### Setup (one-time)

```bash
cd experiments/gemma_investigation
conda env create -f environment.yml          # Gemma 4 needs transformers 5.x
conda activate gemma-lab
python -m ipykernel install --user --name gemma-lab --display-name "Python (gemma-lab)"
```

**No Hugging Face login or license is required — Gemma 4 E4B is a public model.**
(A token is *optional*; it only lifts the anonymous-download rate limit.)

### Run it

- **Notebook:** open `investigate.ipynb`, choose the **Python (gemma-lab)** kernel,
  run top to bottom. The first model-load cell pulls **~16 GB** of weights to
  `~/.cache/huggingface` (once) and uses the M1 Max **MPS** backend.
- **Headless (same logic):** `BACKEND=gemma python run_all.py`
- **No-model smoke test:** `python run_all.py` (a mock backend validates the
  pipeline with no download).

Regenerate the notebook from source with `python build_notebook.py`.

## What the metrics mean

- **`json_validity_rate`** — fraction of reflection replies that parse as JSON
  with a valid action. The robot loop can only act on valid output.
- **`action_accuracy`** — fraction matching the gold strategic action.
- **`wer_mean`** — word error rate vs the known TTS text (lower = better).
- **`rtf_mean`** — real-time factor (`proc_time / audio_len`); `<1` keeps up with
  the voice stream.

## Notes / next steps

- **Action vocabulary** in `config.VALID_ACTIONS` is a placeholder re-theme of
  V1's set — finalize against the real V2 schema (the Or→Asaf interface).
- **Jetson production path (for Or):** Gemma 4 runs on the Orin Nano via
  **llama.cpp** (E2B is the comfortable fit, ~11–14 tok/s); Ollama doesn't
  support Gemma 4 on Orin Nano yet. See the Jetson AI Lab Gemma-4 tutorial.
- The scenarios + utterances are intentionally aligned: the ASR output *is* the
  `voice_transcription` the reflection layer reads, so they're two views of one
  data flow.
```
