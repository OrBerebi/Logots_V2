"""gemma_lab -- a small, reproducible harness investigating whether a local
Gemma 4 (E4B) can serve BOTH of Logots V1's AI stages:

  * reflection : the strategic decision layer (V1 used cloud Claude Haiku)
  * ASR        : speech -> text (V1 used Whisper tiny)

One Gemma multimodal pipeline does both. Run it with a Mock backend to validate
the harness with no download, then flip config.BACKEND="gemma" for the real run.
"""
from . import asr, audio, backends, config, reflection, report, scenarios

__all__ = ["asr", "audio", "backends", "config", "reflection", "report", "scenarios"]
