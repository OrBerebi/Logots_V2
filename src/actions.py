#!/usr/bin/env python3
"""actions — the Or↔Asaf contract: atomic actions + the :8788 endpoint.

Maps to the "ACTIONS" box in docs/v2_architecture/architecture.svg.

The contract (v2 — proposed to Darab in this refactor):
    GET  /latest_action   unchanged — latest decision, dedupe on action_id
    POST /action_done     NEW — the actuator side reports a completed action:
                          {"action_id": N, "pos_x": .., "pos_y": .., "heading": ..}

Changes to the action set vs v1 (audio_on_demand.ACTIONS, untouched there):
    - `initiation` removed — it is a *function* (a routine in functions.py),
      never a single action.
    - narrowed to what the current functions need: inspect (served by OUR
      side: the mart's last row — fresh frame + pose), approach_plant, speak.
    - `finish` added — the terminal action that ends a function's loop.
"""
from __future__ import annotations

import json
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from audio_on_demand import ACTION_PORT

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ACTIONS_MD_PATH = os.path.join(REPO, "knowledge", "actions.md")


import re as _re

_TYPES = {"string": {"type": "string"}, "int": {"type": "integer"}, "number": {"type": "number"}}


def _arg_schema(token: str) -> tuple[str, dict]:
    """'left_pwm:int(-255..255)' → ('left_pwm', {'type':'integer','minimum':-255,...})"""
    m = _re.fullmatch(r"(\w+):(string|int|number)(?:\(([-\d.]+)\.\.([-\d.]+)\))?", token)
    if not m:
        return token, {"type": "string"}          # bare name → free string
    name, typ, lo, hi = m.groups()
    schema = dict(_TYPES[typ])
    if lo is not None:
        num = int if typ == "int" else float
        schema["minimum"], schema["maximum"] = num(lo), num(hi)
    return name, schema


def load_actions() -> tuple[dict, dict, str]:
    """Parse knowledge/actions.md — the single source of truth for the action
    set. One `## name` block per action; its `- args:` line declares the typed
    args. Returns ({name: (arg, ...)}, {name: args JSON schema}, full text).
    The schema is what makes the contract enforced, not just documented: at
    decoding time the LLM can only generate these actions with these args."""
    text = open(ACTIONS_MD_PATH).read()
    actions, schemas, name = {}, {}, None
    for line in text.splitlines():
        m = _re.match(r"##\s+(\w+)\s*$", line)
        if m:
            name = m.group(1)
            actions[name], schemas[name] = (), {"type": "object"}
            continue
        m = _re.match(r"-\s*args:\s*(.+)$", line.strip())
        if m and name:
            tokens = [t.strip() for t in m.group(1).split(",") if t.strip()]
            open_ended = "..." in tokens
            parsed = [_arg_schema(t) for t in tokens if t != "..."]
            actions[name] = tuple(n for n, _ in parsed)
            if not open_ended:
                schemas[name] = {"type": "object",
                                 "properties": {n: s for n, s in parsed},
                                 "required": [n for n, _ in parsed]}
    if not actions:
        raise RuntimeError(f"no actions parsed from {ACTIONS_MD_PATH}")
    # for the prompt: only the ## action blocks — the file header is for humans
    blocks = text[text.index("## "):]
    return actions, schemas, blocks


# Deliberately narrow: only what the current functions need; grows with the .md.
ACTIONS, ACTION_ARG_SCHEMAS, ACTIONS_MD = load_actions()


class ActionsEndpoint:
    """Serves the contract on :8788. Same GET semantics as audio_on_demand's
    ActionServer (payload shape included), plus POST /action_done for the
    completion feedback the functions loop waits on."""

    def __init__(self, port: int = ACTION_PORT):
        self._latest = None
        # Seeded from wall-clock ms, not 1: a long-lived GUI process's
        # ActionReader keeps its `_seen_id` dedup state across separate
        # functions.py restarts, so IDs starting at 1 every run collide with
        # a prior run's id and get silently skipped (found 2026-09-15, see
        # CLAUDE.md Known issues). This makes ids unique across process
        # restarts while staying monotonically increasing within one run.
        self._next_id = int(time.time() * 1000)
        self._done: dict[int, dict] = {}
        self._lock = threading.Lock()
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path != "/latest_action":
                    self.send_error(404); return
                with outer._lock:
                    payload = outer._latest
                if payload is None:
                    self.send_error(503, "no action yet"); return
                body = json.dumps(payload).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_POST(self):
                if self.path != "/action_done":
                    self.send_error(404); return
                try:
                    n = int(self.headers.get("Content-Length", 0))
                    report = json.loads(self.rfile.read(n))
                    aid = int(report["action_id"])
                except (ValueError, KeyError, json.JSONDecodeError):
                    self.send_error(400, "expected JSON with action_id"); return
                with outer._lock:
                    outer._done[aid] = report
                self.send_response(204); self.end_headers()

            def log_message(self, *a):  # quiet
                pass

        self._httpd = ThreadingHTTPServer(("localhost", port), Handler)
        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()
        print(f"[actions] GET http://localhost:{port}/latest_action · "
              f"POST /action_done", flush=True)

    def publish(self, decision: dict) -> dict:
        with self._lock:
            payload = {"action_id": self._next_id,
                       "ts": datetime.now().isoformat(), **decision}
            self._latest, self._next_id = payload, self._next_id + 1
        return payload

    def completion(self, action_id: int) -> dict | None:
        with self._lock:
            return self._done.get(action_id)


def open_endpoint(port: int = ACTION_PORT) -> ActionsEndpoint | None:
    """Bind :8788, or None if taken (the GUI's voice assistant holds it when the
    GUI runs with voice active) — the caller then prints decisions instead."""
    try:
        return ActionsEndpoint(port)
    except OSError:
        print(f"[actions] :{port} already taken (GUI voice assistant?) — "
              f"decisions will be printed only", flush=True)
        return None
