"""Central configuration for the Gemma investigation.

Everything tunable lives here so the notebook and scripts stay declarative.
The model is a single switch: Gemma 4 E4B is the target, but Gemma 3n E4B is a
drop-in fallback that already runs on transformers 4.57 and has the *same*
unified capability (multimodal LLM + native USM audio ASR).
"""
from __future__ import annotations

from pathlib import Path

# --- Paths -------------------------------------------------------------------
PKG_DIR = Path(__file__).resolve().parent
ROOT = PKG_DIR.parent                       # experiments/gemma_investigation/
OUTPUTS = ROOT / "outputs"
AUDIO_DIR = OUTPUTS / "audio"
RESULTS_DIR = OUTPUTS / "results"
REPORT_PATH = OUTPUTS / "report.md"

for _d in (OUTPUTS, AUDIO_DIR, RESULTS_DIR):
    _d.mkdir(parents=True, exist_ok=True)

# --- Model -------------------------------------------------------------------
# Switch the backend with BACKEND; switch the weights with MODEL_ID.
#   BACKEND="mock"   -> no download, no auth: validates the whole harness.
#   BACKEND="gemma"  -> real model via HF Transformers (needs HF login + license).
BACKEND = "mock"

# Target weights for the real run. E4B is the edge-grade multimodal+audio model.
MODEL_ID = "google/gemma-4-E4B-it"
# Already-supported fallback if Gemma 4 / latest transformers isn't in place yet:
FALLBACK_MODEL_ID = "google/gemma-3n-E4B-it"

# --- Audio contract (from Gemma audio docs) ----------------------------------
# Gemma's USM audio encoder expects: 16 kHz, mono, float32 in [-1, 1], <= 30 s.
AUDIO_SAMPLE_RATE = 16_000
AUDIO_MAX_SECONDS = 30

# --- Reflection task ---------------------------------------------------------
# Plant-sitter action vocabulary (V2). Mirrors the *shape* of V1's reflective
# VALID_ACTIONS = {get_closer, back_off, play_arm, no_action}, re-themed for
# plant care. Placeholder set -- Asaf/Or to finalize against the real V2 schema.
VALID_ACTIONS = (
    "patrol",          # roam looking for plants / issues
    "approach_plant",  # drive toward a detected plant
    "inspect_plant",   # pan/tilt camera to examine a plant up close
    "report_issue",    # flag a plant that looks unhealthy (wilt / dry / pest)
    "return_to_base",  # head back to the dock
    "no_action",       # hold position
)

# How V1 framed it: read the compressed recent experience window, return ONE
# strategic action as strict JSON. We keep that contract verbatim.
REFLECTION_SYSTEM_PROMPT = (
    "You are the reflective decision layer of Logots, an autonomous home "
    "plant-sitter robot. Every cycle you are given a compressed summary of the "
    "robot's recent multi-modal experience (vision, heard speech, motion). "
    "Choose the single best strategic action.\n\n"
    "Valid actions: " + ", ".join(VALID_ACTIONS) + ".\n\n"
    "Rules:\n"
    "- A clear spoken human command outranks passive visual cues.\n"
    "- Only 'report_issue' when there is an explicit unhealthy-plant signal.\n"
    "- If nothing warrants action, choose 'no_action'.\n\n"
    'Respond with STRICT JSON only, no prose: '
    '{"action": "<one of the valid actions>", "reason": "<short justification>"}'
)

# Canonical ASR instruction (from Gemma audio docs).
ASR_PROMPT = (
    "Transcribe the following speech segment in its original language. "
    "Only output the transcription, with no newlines and no extra commentary."
)
