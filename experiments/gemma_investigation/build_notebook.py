"""Build the self-contained walk-through notebook (investigate.ipynb).

All logic is inline in the notebook cells so it reads top-to-bottom with code +
explanations visible (made to show a collaborator). This script just assembles
it; execute the notebook afterwards to bake in real outputs.
"""
import nbformat as nbf

nb = nbf.v4.new_notebook()
cells = []
def md(s):   cells.append(nbf.v4.new_markdown_cell(s))
def code(s): cells.append(nbf.v4.new_code_cell(s))

# ---------------------------------------------------------------- intro
md("""# Logots V2 — One Local Gemma for Reflection **and** Speech Recognition

A self-contained walk-through (for Asaf & Darab).

**Goal:** test whether a *single local* **Gemma 4 E4B** can do **both** AI jobs the
V1 cat-sitter split across two services:

| V1 (cat-sitter) | model used | V2 candidate |
|---|---|---|
| Reflective decision (~20 s loop) | cloud **Claude Haiku** | **Gemma 4 E4B** (local) |
| Speech-to-text (ASR) | **Whisper tiny** | **Gemma 4 E4B** (the *same* model) |

Gemma 4's small variants are multimodal with **native audio input**, so one
pipeline handles strategic reasoning *and* transcription — **offline, no cloud,
no API key**. Everything below runs top-to-bottom; all code is inline.""")

# ---------------------------------------------------------------- how to run
md("""## How to run this

**Environment:** the `gemma-lab` conda env (Python 3.12, `transformers` 5.x,
torch + MPS). Open this notebook with the **`Python (gemma-lab)`** kernel.

**One-time prerequisites**
- `conda env create -f environment.yml`, then register the kernel (see README).
- The first model load downloads **~16 GB** of weights to `~/.cache/huggingface`
  — **no login needed**, Gemma 4 E4B is a *public* model.

**Two ways to run**
1. **This notebook** — run the cells in order (what you're reading).
2. **Headless script** — `BACKEND=gemma python run_all.py` (identical logic,
   factored into the `gemma_lab/` package for scripting/CI).

### Inputs & outputs at a glance

| Stage | Input | Output |
|---|---|---|
| **Reflection** | a short **experience-window** *text* (what the robot recently saw / heard / how it moved) | strict **JSON** `{action, reason}`, action from a fixed vocabulary |
| **ASR** | a **16 kHz mono WAV** clip of speech | the **transcript** text |

In the real robot these connect: ASR turns the mic stream into the `HEARD:` line
that the reflection step reads.""")

code("""import json, re, subprocess, tempfile, time
from pathlib import Path
import torch, librosa, soundfile as sf, pandas as pd
import transformers
transformers.logging.set_verbosity_error()   # keep notebook output clean

CLIPS = Path("outputs/nb_audio"); CLIPS.mkdir(parents=True, exist_ok=True)
print("torch", torch.__version__, "| transformers", transformers.__version__,
      "| MPS available:", torch.backends.mps.is_available())""")

# ---------------------------------------------------------------- step 1 load
md("""## Step 1 — Load Gemma 4 E4B (one model for both tasks)

We build a single Hugging Face `pipeline`:
- **`task="any-to-any"`** — Gemma 4 is multimodal; this task accepts text *and*
  audio inside the chat messages.
- **`device="mps"`** — run on the Apple-Silicon GPU. (On the Jetson this would be
  CUDA / llama.cpp instead.)
- **`dtype=bfloat16`** — ~16 GB in unified memory; fits the 32 GB Mac.

This is the slow cell (loads ~16 GB, ~1 min). Run it once.""")

code("""from transformers import pipeline

MODEL_ID = "google/gemma-4-E4B-it"
device = "mps" if torch.backends.mps.is_available() else "cpu"
pipe = pipeline(task="any-to-any", model=MODEL_ID, dtype=torch.bfloat16, device=device)
print("loaded", MODEL_ID, "on", device)""")

md("""Two small helpers used by both tasks: one to send chat messages to the
model, one to dig the assistant's reply text out of the pipeline result.""")

code('''def assistant_text(out):
    """Pull the assistant's reply text out of a chat-pipeline result."""
    gen = out[0]["generated_text"] if isinstance(out, list) else out["generated_text"]
    if isinstance(gen, str):
        return gen.strip()
    msg = gen[-1]                                   # last message = the reply
    content = msg["content"] if isinstance(msg, dict) else msg
    if isinstance(content, list):                   # list of content blocks
        return "\\n".join(b.get("text", "") for b in content).strip()
    return str(content).strip()

def gemma_chat(messages, max_new_tokens=200):
    out = pipe(text=messages, max_new_tokens=max_new_tokens, do_sample=False)  # greedy = reproducible
    return assistant_text(out)''')

# ---------------------------------------------------------------- step 2 reflection
md('''## Step 2 — Reflection (the strategic decision)

**What it is:** every ~20 s the robot summarizes its recent experience and asks
the model for the single best move. This replaces V1's cloud Claude Haiku call.

**Input** — an *experience window*, e.g.:
```
VISION: monstera detected, close, centered, leaves fill frame.
HEARD: (silence).
MOTION: stopped 4s in front of plant.
```
**Output** — strict JSON: `{"action": "inspect_plant", "reason": "..."}`.

The model must choose from a fixed **action vocabulary**:''')

code("""VALID_ACTIONS = ["patrol", "approach_plant", "inspect_plant",
                 "report_issue", "return_to_base", "no_action"]

example_json = json.dumps({"action": "inspect_plant", "reason": "plant is close and centered"})
SYSTEM_PROMPT = (
    "You are the reflective decision layer of Logots, a home plant-sitter robot.\\n"
    "Given a compressed summary of recent experience (vision, heard speech, motion), "
    "choose ONE strategic action.\\n\\n"
    f"Valid actions: {', '.join(VALID_ACTIONS)}.\\n\\n"
    "Rules:\\n"
    "- A clear spoken human command outranks passive visual cues.\\n"
    "- Use 'report_issue' only when there is an explicit unhealthy-plant signal.\\n"
    "- If nothing warrants action, choose 'no_action'.\\n\\n"
    f"Respond with STRICT JSON only, for example: {example_json}"
)
print(SYSTEM_PROMPT)""")

code('''def reflect(window: str) -> str:
    """experience-window text -> raw model reply (expected JSON)."""
    messages = [
        {"role": "system", "content": [{"type": "text", "text": SYSTEM_PROMPT}]},
        {"role": "user",   "content": [{"type": "text", "text": window}]},
    ]
    return gemma_chat(messages, max_new_tokens=200)

def parse_action(reply: str):
    """Extract the action from the model's JSON reply (None if unparseable)."""
    m = re.search(r"\\{.*\\}", reply, re.DOTALL)
    if not m:
        return None
    try:
        return json.loads(m.group(0)).get("action")
    except json.JSONDecodeError:
        return None''')

md("""### Try one example by hand
See exactly one input → one output.""")

code('''demo_window = (
    "VISION: monstera detected, large bbox area (close), centered, leaves fill frame.\\n"
    "HEARD: (silence).\\n"
    "MOTION: stopped 4s in front of plant, balanced."
)
print("INPUT (experience window):\\n" + demo_window + "\\n")
reply = reflect(demo_window)
print("RAW MODEL OUTPUT:\\n" + reply + "\\n")
print("PARSED ACTION:", parse_action(reply))''')

md("""### Run the full scenario set
12 hand-authored scenarios, each with a "gold" expected action. Several test that
a **spoken command overrides vision** — the place where ASR feeds reflection.""")

code('''SCENARIOS = [
    ("idle_open_floor",
     "VISION: no plant detected, open floor ahead.\\nHEARD: (silence).\\nMOTION: stationary 18s, at rest.",
     "patrol"),
    ("plant_far_centered",
     "VISION: potted plant detected, small bbox area (far), centered.\\nHEARD: (silence).\\nMOTION: slow forward drift.",
     "approach_plant"),
    ("plant_near_inspect",
     "VISION: monstera detected, large bbox area (close), centered, leaves fill frame.\\nHEARD: (silence).\\nMOTION: stopped 4s in front of plant.",
     "inspect_plant"),
    ("wilted_leaves",
     "VISION: basil plant close, leaves drooping and pale, soil looks dry/cracked.\\nHEARD: (silence).\\nMOTION: stopped in front of plant.",
     "report_issue"),
    ("voice_go_check_fern",
     "VISION: no plant in current frame, hallway ahead.\\nHEARD (voice_transcription): \\"go check the fern in the living room\\".\\nMOTION: stationary.",
     "approach_plant"),
    ("voice_overrides_vision",
     "VISION: potted plant close and centered (would normally inspect).\\nHEARD (voice_transcription): \\"stop that and come back to the kitchen\\".\\nMOTION: stopped in front of plant.",
     "return_to_base"),
    ("voice_inspect_command",
     "VISION: succulent detected mid-range, slightly left of center.\\nHEARD (voice_transcription): \\"take a close look at this one\\".\\nMOTION: slow approach.",
     "inspect_plant"),
    ("voice_report_request",
     "VISION: orchid close, flowers present, foliage green.\\nHEARD (voice_transcription): \\"does this plant look okay to you\\".\\nMOTION: stopped.",
     "report_issue"),
    ("chatter_not_command",
     "VISION: no plant detected, living room, TV on in background.\\nHEARD (voice_transcription): \\"...and then we drove all the way to the coast\\".\\nMOTION: stationary.",
     "patrol"),
    ("done_for_now",
     "VISION: no plant, near the charging dock.\\nHEARD (voice_transcription): \\"okay that\\'s enough for today, go rest\\".\\nMOTION: idling near dock.",
     "return_to_base"),
    ("ambiguous_hold",
     "VISION: blurry frame, motion blur, nothing identifiable.\\nHEARD: (silence).\\nMOTION: just stopped after a turn, settling.",
     "no_action"),
    ("pest_on_leaf",
     "VISION: fiddle-leaf fig close, dark spots and webbing on underside of leaves.\\nHEARD: (silence).\\nMOTION: stopped, camera tilted up at foliage.",
     "report_issue"),
]

rows = []
for sid, window, expected in SCENARIOS:
    t0 = time.perf_counter()
    reply = reflect(window)
    dt = time.perf_counter() - t0
    action = parse_action(reply)
    rows.append({"id": sid, "expected": expected, "predicted": action,
                 "correct": action == expected, "valid_json": action is not None,
                 "sec": round(dt, 1)})

refl_df = pd.DataFrame(rows)
refl_df''')

code('''acc   = refl_df["correct"].mean()
valid = refl_df["valid_json"].mean()
print(f"JSON validity : {valid:.0%}   (the robot loop can only act on parseable output)")
print(f"Action match  : {acc:.0%}   ({refl_df['correct'].sum()}/{len(refl_df)})")
print(f"Latency       : {refl_df['sec'][1:].median():.1f}s median (first call includes one-time warmup)")''')

md("""### Reading the reflection result

Two things to notice:

1. **JSON validity is the number that matters for wiring** — every reply must
   parse, because the robot loop acts on `action`. Gemma is reliable here.
2. **"Action match" is lower, but most misses are *ambiguous label boundaries* I
   defined, not model errors.** The action vocabulary has overlapping categories:
   - `patrol` vs `no_action` when the robot is idle,
   - `inspect_plant` vs `report_issue` when examining a plant,
   - `patrol` vs `approach_plant` when the target isn't in view yet.

   In those cases Gemma's choice is usually defensible and my "gold" label just
   encoded an arbitrary tie-break.

**Takeaway for Darab:** the bottleneck isn't the model — it's that the **action
set needs disjoint, well-defined meanings** before an accuracy number is
meaningful. That action set is exactly the V2 command schema on our hardware↔AI
boundary, so it's worth nailing down together.""")

# ---------------------------------------------------------------- step 3 ASR
md("""## Step 3 — Automatic Speech Recognition (ASR)

**What it is:** turn a spoken command into text. V1 used Whisper; here the *same*
Gemma model does it through its audio input.

**Input** — a **16 kHz mono float32 WAV** (Gemma's audio contract, ≤ 30 s).
**Output** — the transcript string.

We synthesize the test speech with macOS `say`, so we know the exact words and
can compute **WER** (word error rate) with zero manual labeling.""")

code('''SR = 16000   # Gemma's audio encoder wants 16 kHz mono

def synthesize(text, path, voice="Samantha"):
    """Render text to a 16 kHz mono float32 WAV with macOS `say`."""
    with tempfile.NamedTemporaryFile(suffix=".aiff", delete=False) as t:
        raw = Path(t.name)
    subprocess.run(["say", "-v", voice, "-o", str(raw), text], check=True)
    y, _ = librosa.load(str(raw), sr=SR, mono=True)        # -> mono, 16 kHz, float32
    sf.write(str(path), y, SR, subtype="FLOAT")
    raw.unlink()
    return len(y) / SR

ASR_PROMPT = "Transcribe the following speech segment. Only output the transcription, no extra commentary."

def transcribe(path):
    """16 kHz wav -> transcript text (same Gemma model, audio in the message)."""
    messages = [{"role": "user", "content": [
        {"type": "audio", "audio": str(path)},
        {"type": "text",  "text": ASR_PROMPT},
    ]}]
    return gemma_chat(messages, max_new_tokens=128)''')

md("""### Try one clip by hand
Synthesize it, listen to it, transcribe it.""")

code('''from IPython.display import Audio, display

demo_text = "go check the fern in the living room"
synthesize(demo_text, CLIPS / "demo.wav")
display(Audio(str(CLIPS / "demo.wav")))
print("REFERENCE :", demo_text)
print("GEMMA ASR :", transcribe(CLIPS / "demo.wav"))''')

md("""### Run all clips + score WER""")

code('''def norm(s):
    return re.sub(r"[^\\w\\s]", " ", s.lower()).split()

def wer(ref, hyp):
    """Word error rate = word-level edit distance / reference length."""
    r, h = norm(ref), norm(hyp)
    if not r:
        return 0.0 if not h else 1.0
    prev = list(range(len(h) + 1))
    for i, rw in enumerate(r, 1):
        cur = [i]
        for j, hw in enumerate(h, 1):
            cur.append(min(prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + (rw != hw)))
        prev = cur
    return prev[-1] / len(r)

UTTERANCES = [
    "go check the fern in the living room",
    "come back to the kitchen",
    "take a close look at this one",
    "does this plant look okay to you",
    "okay that's enough for today go rest",
    "water the plant on the windowsill",
    "follow me to the bedroom",
    "how is the basil doing",
]

arows = []
for i, text in enumerate(UTTERANCES):
    p = CLIPS / f"utt_{i:02d}.wav"
    synthesize(text, p)
    hyp = transcribe(p)
    arows.append({"reference": text, "gemma_asr": hyp, "wer": round(wer(text, hyp), 3)})

asr_df = pd.DataFrame(arows)
asr_df''')

code('''print(f"Mean WER      : {asr_df['wer'].mean():.1%}")
print(f"Exact matches : {(asr_df['wer'] == 0).sum()}/{len(asr_df)}")''')

# ---------------------------------------------------------------- conclusion
md("""## Findings

- **Feasibility: confirmed.** One local Gemma 4 E4B does both jobs on the Mac,
  offline — clean JSON for reflection and strong transcription for ASR.
- **ASR is strong out of the box.** The few word errors are mostly TTS artifacts
  (e.g. *"water the"* heard as *"what are"* — they genuinely sound alike).
- **Reflection produces valid JSON fast** (well within V1's ~20 s budget). Its
  apparent "accuracy" is limited by **ambiguous action definitions**, not model
  capability (see the reading notes above).

**Next steps**
1. Redesign the action vocabulary so categories are **disjoint and well-defined**
   (this is the V2 command schema — a shared Asaf↔Darab decision), then re-measure.
2. Optionally add a few worked examples to the reflection prompt and compare.
3. For the Jetson: same model via **llama.cpp** (E2B is the comfortable Orin-Nano
   fit); this Mac run is the reference.""")

nb.cells = cells
nb.metadata.kernelspec = {"display_name": "Python (gemma-lab)", "language": "python", "name": "gemma-lab"}
nb.metadata.language_info = {"name": "python"}

with open("investigate.ipynb", "w") as f:
    nbf.write(nb, f)
print(f"wrote investigate.ipynb with {len(cells)} cells")
