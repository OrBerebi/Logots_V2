"""Model backends behind one small interface: reflect() and transcribe().

- MockBackend  : zero-dependency stand-in. Reflection uses tiny keyword rules
                 (doubles as a rules baseline); ASR returns a fixed stub. Lets us
                 prove the data -> model -> score -> report pipeline end to end
                 with no model download and no HF auth.
- GemmaBackend : the real thing via HF Transformers, following the Gemma audio
                 docs (a single `any-to-any` pipeline serves BOTH tasks). This is
                 the only piece that needs the gated weights + HF login.
"""
from __future__ import annotations

import json
import re
from abc import ABC, abstractmethod

from . import config


class Backend(ABC):
    name: str
    model_id: str

    @abstractmethod
    def reflect(self, window: str) -> str:
        """Return the model's raw reply to a reflection prompt (expected JSON)."""

    @abstractmethod
    def transcribe(self, audio_path: str) -> str:
        """Return the transcript for a 16 kHz mono wav clip."""


# --------------------------------------------------------------------------- #
# Mock
# --------------------------------------------------------------------------- #
class MockBackend(Backend):
    """Deterministic stand-in. Reflection = keyword rules; ASR = stub."""

    name = "mock"
    model_id = "rules-and-stub"

    # Order matters: first match wins. Mirrors the system-prompt rules.
    _VOICE_RULES = (
        (("come back", "go rest", "enough for today", "kitchen", "dock"), "return_to_base"),
        (("close look", "take a look", "inspect"), "inspect_plant"),
        (("look okay", "look ok", "is the", "how is", "healthy"), "report_issue"),
        (("check the", "go to", "follow me", "water the"), "approach_plant"),
    )
    _VISION_RULES = (
        (("drooping", "pale", "dry", "cracked", "spots", "webbing", "wilt"), "report_issue"),
        (("large bbox", "close", "leaves fill", "fill frame"), "inspect_plant"),
        (("small bbox", "far"), "approach_plant"),
    )

    def reflect(self, window: str) -> str:
        w = window.lower()
        action, reason = "no_action", "no salient cue"

        # A spoken command outranks vision (matches the system prompt).
        heard = self._heard_text(w)
        if heard and not self._is_silence(heard):
            for keys, act in self._VOICE_RULES:
                if any(k in heard for k in keys):
                    action, reason = act, "matched spoken command"
                    break
            else:
                action, reason = "patrol", "speech present but no command"
        else:
            for keys, act in self._VISION_RULES:
                if any(k in w for k in keys):
                    action, reason = act, "visual cue"
                    break
            else:
                if "no plant" in w:
                    action, reason = "patrol", "no plant in view"
                elif "blur" in w or "settling" in w:
                    action, reason = "no_action", "ambiguous frame"

        return json.dumps({"action": action, "reason": reason})

    def transcribe(self, audio_path: str) -> str:  # noqa: ARG002
        # Mock cannot hear; return a clearly-fake stub so WER is obviously a
        # placeholder in the validation report.
        return "(mock backend produced no transcription)"

    @staticmethod
    def _heard_text(window_lower: str) -> str:
        m = re.search(r'heard[^:]*:\s*(.*)', window_lower)
        return m.group(1).strip() if m else ""

    @staticmethod
    def _is_silence(heard: str) -> bool:
        return "silence" in heard or heard in ("", "(silence)")


# --------------------------------------------------------------------------- #
# Gemma (real)
# --------------------------------------------------------------------------- #
class GemmaBackend(Backend):
    """Gemma 4 (or 3n) via a single HF Transformers `any-to-any` pipeline.

    Lazy-loads on first use. Requires: latest `transformers`, the gated weights,
    and a logged-in HF token (`huggingface-cli login`).
    """

    name = "gemma"

    def __init__(self, model_id: str | None = None, max_new_tokens: int = 256):
        self.model_id = model_id or config.MODEL_ID
        self.max_new_tokens = max_new_tokens
        self._pipe = None

    def _pipeline(self):
        if self._pipe is None:
            import torch
            from transformers import pipeline  # imported lazily
            # Load the whole model onto MPS (no device_map="auto", which spilled
            # weights to disk on a 32 GB Mac). bf16 ~16 GB fits in unified memory.
            device = "mps" if torch.backends.mps.is_available() else "cpu"
            self._pipe = pipeline(
                task="any-to-any",
                model=self.model_id,
                dtype=torch.bfloat16,
                device=device,
            )
        return self._pipe

    def reflect(self, window: str) -> str:
        messages = [
            {"role": "system",
             "content": [{"type": "text", "text": config.REFLECTION_SYSTEM_PROMPT}]},
            {"role": "user",
             "content": [{"type": "text", "text": window}]},
        ]
        return self._generate(messages)

    def transcribe(self, audio_path: str) -> str:
        messages = [
            {"role": "user", "content": [
                {"type": "audio", "audio": audio_path},
                {"type": "text", "text": config.ASR_PROMPT},
            ]},
        ]
        return self._generate(messages)

    def _generate(self, messages) -> str:
        out = self._pipeline()(text=messages, max_new_tokens=self.max_new_tokens)
        return _extract_assistant_text(out)


def _extract_assistant_text(out) -> str:
    """Pull the assistant's text out of a transformers chat-pipeline result.

    Handles the common shapes: a list of dicts with `generated_text` that is
    either a plain string or a list-of-messages whose last item is the reply.
    """
    if isinstance(out, list) and out:
        out = out[0]
    gen = out.get("generated_text", out) if isinstance(out, dict) else out
    if isinstance(gen, str):
        return gen.strip()
    if isinstance(gen, list) and gen:
        last = gen[-1]
        content = last.get("content", last) if isinstance(last, dict) else last
        if isinstance(content, list):  # content blocks
            texts = [b.get("text", "") for b in content if isinstance(b, dict)]
            return "\n".join(t for t in texts if t).strip()
        return str(content).strip()
    return str(gen).strip()


def get_backend() -> Backend:
    """Construct the backend named by config.BACKEND."""
    if config.BACKEND == "mock":
        return MockBackend()
    if config.BACKEND == "gemma":
        return GemmaBackend()
    raise ValueError(f"Unknown BACKEND: {config.BACKEND!r}")
