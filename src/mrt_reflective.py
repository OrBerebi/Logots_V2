#!/usr/bin/env python3
"""mrt_reflective — Gemma access: the robot's interpreter.

Maps to the "mrt_reflective" box in docs/v2_architecture/architecture.svg.
Text + images in, one decision (strict JSON) out.

Brain auto-selection follows Darab's GUI convention: llama.cpp/E2B when his
llama-server binary exists (the Jetson, and any machine mirroring it), else the
transformers/E4B pipeline. Override with VOICE_BRAIN=llamacpp|gemma.
Darab's LlamaCppBrain is imported untouched and extended here with vision.
"""
from __future__ import annotations

import json
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from audio_on_demand import MODEL_ID, LlamaCppBrain, LLAMACPP_BIN, LLAMACPP_N_PREDICT


class VisionBrain:
    """transformers any-to-any pipeline (E4B) — the non-llama.cpp fallback."""

    def __init__(self, model_id: str = MODEL_ID):
        self.model_id = model_id
        self._pipe = None

    def _pipeline(self):
        if self._pipe is None:
            import torch
            from transformers import pipeline
            device = "mps" if torch.backends.mps.is_available() else \
                     ("cuda" if torch.cuda.is_available() else "cpu")
            print(f"[brain] loading {self.model_id} on {device} …", flush=True)
            self._pipe = pipeline(task="any-to-any", model=self.model_id,
                                  dtype=torch.bfloat16, device=device)
        return self._pipe

    def ask(self, text: str, images: list | None = None, max_new_tokens: int = 1024,
            schema: dict | None = None) -> str:   # schema: llama.cpp-only, ignored here
        content = [{"type": "image", "image": im} for im in (images or [])]
        content.append({"type": "text", "text": text})
        out = self._pipeline()(text=[{"role": "user", "content": content}],
                               max_new_tokens=max_new_tokens)
        if isinstance(out, list) and out:
            out = out[0]
        gen = out.get("generated_text", out) if isinstance(out, dict) else out
        if isinstance(gen, list) and gen:
            c = gen[-1].get("content", gen[-1]) if isinstance(gen[-1], dict) else gen[-1]
            if isinstance(c, list):
                return "\n".join(b.get("text", "") for b in c if isinstance(b, dict)).strip()
            return str(c).strip()
        return str(gen).strip()


class LlamaCppVisionBrain(LlamaCppBrain):
    """Text+images through Darab's llama-server — his class untouched, extended
    here (our file) with an ask() so the same server that hears can also see.
    Images go through the server's --media-path mechanism, like his audio does."""

    def ask(self, text: str, images: list | None = None, max_new_tokens: int = LLAMACPP_N_PREDICT,
            schema: dict | None = None) -> str:
        import urllib.request
        self._ensure_server()
        os.makedirs(self.media_dir, exist_ok=True)
        names = []
        for i, im in enumerate(images or []):
            name = f"reflective_{i}.jpg"
            im.save(os.path.join(self.media_dir, name), "JPEG", quality=90)
            names.append(name)
        content = [{"type": "image_url", "image_url": {"url": f"file://{n}"}} for n in names]
        content.append({"type": "text", "text": text})
        body = {"messages": [{"role": "user", "content": content}],
                "temperature": 0, "max_tokens": max_new_tokens,
                "reasoning_format": "none"}
        if schema is not None:
            # constrained decoding: the server can only generate JSON matching
            # the schema — a closed "action" enum, free values inside "args"
            body["response_format"] = {"type": "json_object", "schema": schema}
        req = urllib.request.Request(f"http://{self.host}:{self.port}/v1/chat/completions",
                                     data=json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"}, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=180) as r:
                out = json.loads(r.read())["choices"][0]["message"]["content"]
        finally:
            for n in names:
                try: os.unlink(os.path.join(self.media_dir, n))
                except OSError: pass
        return out.rsplit("<channel|>", 1)[-1].strip()   # same reasoning-trace strip as his


def pick_brain():
    choice = os.environ.get("VOICE_BRAIN", "auto")
    if choice == "llamacpp" or (choice == "auto" and os.path.exists(LLAMACPP_BIN)):
        print("[brain] using LlamaCppVisionBrain (llama.cpp — same as the robot)", flush=True)
        return LlamaCppVisionBrain()
    print("[brain] using VisionBrain (transformers)", flush=True)
    return VisionBrain()


def _json_block(reply: str):
    m = re.search(r"\[.*\]|\{.*\}", reply, re.DOTALL)
    if not m:
        raise ValueError(f"no JSON in model reply: {reply[:200]!r}")
    block = m.group(0)
    try:
        return json.loads(block)
    except json.JSONDecodeError:
        pass
    # cheap repairs for the model's common slips, then one more try
    repaired = block.replace("“", "'").replace("”", "'").replace("’", "'")
    repaired = re.sub(r",\s*([}\]])", r"\1", repaired)                       # trailing commas
    repaired = re.sub(r'(?<=[A-Za-z0-9])"(?=[A-Za-z0-9 ])', "'", repaired)   # inner quotes
    try:
        return json.loads(repaired)
    except json.JSONDecodeError as e:
        raise ValueError(f"unparseable JSON from model ({e}); raw reply:\n{reply}") from e


def ask_json(brain, prompt: str, images: list | None = None, schema: dict | None = None):
    """One Gemma call → parsed JSON, with a single corrective retry on bad JSON.
    With a schema (llama.cpp), decoding itself is constrained to match it."""
    try:
        return _json_block(brain.ask(prompt, images=images, schema=schema))
    except ValueError as e:
        print(f"[brain] invalid JSON, retrying once — {str(e)[:120]}", flush=True)
        return _json_block(brain.ask(
            prompt + "\n\nIMPORTANT: output ONLY valid JSON. Do not use any quotation "
                     "marks inside string values.", images=images, schema=schema))
