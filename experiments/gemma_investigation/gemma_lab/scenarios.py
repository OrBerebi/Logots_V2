"""Synthetic 'experience-window' scenarios for the reflection task.

Each scenario is a compressed text snapshot of the kind V1's reflective layer
built from the last ~80 rows of mrt_experiences, re-themed for the plant-sitter.
`expected_action` is the gold label we score the model against.

These are hand-authored to exercise the decision rules in config.REFLECTION_
SYSTEM_PROMPT -- including cases where a *spoken command* (the ASR output) must
override passive visual cues, which is exactly where reflection + ASR meet.
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class Scenario:
    id: str
    window: str           # the compressed experience text shown to the model
    expected_action: str  # gold label (must be in config.VALID_ACTIONS)
    tags: tuple[str, ...] = field(default_factory=tuple)


SCENARIOS: tuple[Scenario, ...] = (
    Scenario(
        id="idle_open_floor",
        window=(
            "VISION(last 12 frames): no plant detected, open floor ahead.\n"
            "HEARD: (silence).\n"
            "MOTION: stationary 18s, balanced, at rest."
        ),
        expected_action="patrol",
        tags=("baseline", "no-voice"),
    ),
    Scenario(
        id="plant_far_centered",
        window=(
            "VISION: potted plant detected, small bbox area (far), centered.\n"
            "HEARD: (silence).\n"
            "MOTION: slow forward drift, stable."
        ),
        expected_action="approach_plant",
        tags=("vision", "no-voice"),
    ),
    Scenario(
        id="plant_near_inspect",
        window=(
            "VISION: monstera detected, large bbox area (close), centered, leaves fill frame.\n"
            "HEARD: (silence).\n"
            "MOTION: stopped 4s in front of plant, balanced."
        ),
        expected_action="inspect_plant",
        tags=("vision", "no-voice"),
    ),
    Scenario(
        id="wilted_leaves",
        window=(
            "VISION: basil plant close, leaves drooping and pale, soil looks dry/cracked.\n"
            "HEARD: (silence).\n"
            "MOTION: stopped in front of plant."
        ),
        expected_action="report_issue",
        tags=("vision", "health"),
    ),
    Scenario(
        id="voice_go_check_fern",
        window=(
            "VISION: no plant in current frame, hallway ahead.\n"
            'HEARD (voice_transcription): "go check the fern in the living room".\n'
            "MOTION: stationary, balanced."
        ),
        expected_action="approach_plant",
        tags=("voice", "command"),
    ),
    Scenario(
        id="voice_overrides_vision",
        window=(
            "VISION: potted plant close and centered (would normally inspect).\n"
            'HEARD (voice_transcription): "stop that and come back to the kitchen".\n'
            "MOTION: stopped in front of plant."
        ),
        expected_action="return_to_base",
        tags=("voice", "override"),
    ),
    Scenario(
        id="voice_inspect_command",
        window=(
            "VISION: succulent detected mid-range, slightly left of center.\n"
            'HEARD (voice_transcription): "take a close look at this one".\n'
            "MOTION: slow approach."
        ),
        expected_action="inspect_plant",
        tags=("voice", "command"),
    ),
    Scenario(
        id="voice_report_request",
        window=(
            "VISION: orchid close, flowers present, foliage green.\n"
            'HEARD (voice_transcription): "does this plant look okay to you".\n'
            "MOTION: stopped, balanced."
        ),
        expected_action="report_issue",
        tags=("voice", "health"),
    ),
    Scenario(
        id="chatter_not_command",
        window=(
            "VISION: no plant detected, living room, TV on in background.\n"
            'HEARD (voice_transcription): "...and then we drove all the way to the coast".\n'
            "MOTION: stationary."
        ),
        expected_action="patrol",
        tags=("voice", "distractor"),
    ),
    Scenario(
        id="done_for_now",
        window=(
            "VISION: no plant, near the charging dock.\n"
            'HEARD (voice_transcription): "okay that\'s enough for today, go rest".\n'
            "MOTION: idling near dock."
        ),
        expected_action="return_to_base",
        tags=("voice", "command"),
    ),
    Scenario(
        id="ambiguous_hold",
        window=(
            "VISION: blurry frame, motion blur, nothing identifiable.\n"
            "HEARD: (silence).\n"
            "MOTION: just stopped after a turn, settling."
        ),
        expected_action="no_action",
        tags=("ambiguous",),
    ),
    Scenario(
        id="pest_on_leaf",
        window=(
            "VISION: fiddle-leaf fig close, dark spots and webbing on underside of leaves.\n"
            "HEARD: (silence).\n"
            "MOTION: stopped, camera tilted up at foliage."
        ),
        expected_action="report_issue",
        tags=("vision", "health"),
    ),
)


def as_records() -> list[dict]:
    """Flat dict form for DataFrame / CSV use."""
    return [
        {"id": s.id, "window": s.window, "expected_action": s.expected_action,
         "tags": ",".join(s.tags)}
        for s in SCENARIOS
    ]
