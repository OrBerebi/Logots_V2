"""Synthetic speech for the ASR task, via macOS `say`.

We render a fixed set of spoken plant-care commands to WAV. Because we know the
source text, every clip ships with an exact ground-truth transcript -> WER needs
no manual labeling. Clips are normalized to Gemma's audio contract:
16 kHz, mono, float32 in [-1, 1].

The utterances deliberately mirror the voice scenarios in scenarios.py: in the
real robot the ASR output *is* the `voice_transcription` the reflection layer
reads, so this is the same data flowing through both halves of the experiment.
"""
from __future__ import annotations

import shutil
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path

import librosa
import soundfile as sf

from . import config

# Spoken commands -> exact reference transcript (the gold label for WER).
UTTERANCES: tuple[str, ...] = (
    "go check the fern in the living room",
    "come back to the kitchen",
    "take a close look at this one",
    "does this plant look okay to you",
    "okay that's enough for today go rest",
    "water the plant on the windowsill",
    "follow me to the bedroom",
    "how is the basil doing",
)

# Preferred en_US voices, in order; we fall back to the system default voice.
_PREFERRED_VOICES = ("Samantha", "Alex", "Victoria", "Fred")


@dataclass(frozen=True)
class AudioClip:
    id: str
    path: str          # 16 kHz mono float32 wav
    reference: str      # ground-truth transcript
    duration_s: float


def _say_available() -> bool:
    return shutil.which("say") is not None


def pick_voice() -> str | None:
    """First preferred voice that `say` actually has installed, else None."""
    if not _say_available():
        return None
    try:
        listing = subprocess.run(
            ["say", "-v", "?"], capture_output=True, text=True, check=True
        ).stdout
    except Exception:
        return None
    installed = {line.split()[0] for line in listing.splitlines() if line.strip()}
    for v in _PREFERRED_VOICES:
        if v in installed:
            return v
    return None


def synthesize(text: str, out_path: Path, voice: str | None = None) -> float:
    """Render `text` to a 16 kHz mono float32 WAV at out_path. Returns seconds."""
    if not _say_available():
        raise RuntimeError("macOS `say` not found; cannot synthesize speech.")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(suffix=".aiff", delete=False) as tmp:
        raw = Path(tmp.name)
    try:
        cmd = ["say"]
        if voice:
            cmd += ["-v", voice]
        cmd += ["-o", str(raw), text]
        subprocess.run(cmd, check=True, capture_output=True)
        # librosa gives us mono + target sample rate + float32 in one step.
        y, _ = librosa.load(str(raw), sr=config.AUDIO_SAMPLE_RATE, mono=True)
        sf.write(str(out_path), y, config.AUDIO_SAMPLE_RATE, subtype="FLOAT")
        return float(len(y) / config.AUDIO_SAMPLE_RATE)
    finally:
        raw.unlink(missing_ok=True)


def build_dataset(voice: str | None = "auto") -> list[AudioClip]:
    """Synthesize every utterance and return clip records."""
    if voice == "auto":
        voice = pick_voice()
    clips: list[AudioClip] = []
    for i, text in enumerate(UTTERANCES):
        clip_id = f"utt_{i:02d}"
        path = config.AUDIO_DIR / f"{clip_id}.wav"
        dur = synthesize(text, path, voice=voice)
        clips.append(AudioClip(id=clip_id, path=str(path), reference=text,
                               duration_s=round(dur, 2)))
    return clips


if __name__ == "__main__":
    v = pick_voice()
    print(f"Using voice: {v or '(system default)'}")
    for c in build_dataset():
        print(f"  {c.id}  {c.duration_s:>4.1f}s  {c.path}")
