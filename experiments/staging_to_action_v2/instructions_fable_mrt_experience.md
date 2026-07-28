# Instructions for Fable — build the finalized `mrt_experience` notebook

You are building one self-contained Jupyter notebook that constructs **`mrt_experience`**
for the Logots V2 plant-care robot, consuming Darab's live frame API. This file is the
full spec. Read all of it before writing code.

Author/owner: **Asaf** (data scientist, leads this project). Deliver work his way
(section 9). When a design choice is ambiguous, prefer the simpler option and leave a
short note in the notebook rather than inventing scope.

---

## 1. What `mrt_experience` is

`mrt_experience` is the robot's sense of the **present** — raw sensory reality that a
local LLM (Gemma) later reads to orient and decide. It is **reality only**: nothing
derived, nothing interpreted, nothing that belongs in the robot's `.md` memory.

- It is **not** a stored table or a DBT/batch job. It's a rolling buffer held **in RAM**.
- It is **not** the past. History/knowledge live in `.md` files and a journal (out of
  scope here). `mrt_experience` only tells the LLM what is true *right now*.

The data comes from **Darab's frame API**: his GUI (`src/logots_ui.py`) serves the latest
sensor snapshot over HTTP on `localhost:8787`, in both real-robot and sim (replay) mode.
We consume it with his own client, `src/logots_api.py::get_latest_frame()` — exactly as he
intended, no wrapper, no reading the CSV directly.

---

## 2. The finalized schema — one row = one second

Each `mrt_experience` row is **one second** of reality:

| field           | meaning                                                        |
|-----------------|----------------------------------------------------------------|
| `experience_id` | 1-based consecutive id, one per experience (1, 2, 3, …) — the grain is the *experience*, which today spans one second |
| `ts`       | timestamp of that second                                        |
| `frame`    | **one representative image** for that second (1 fps)            |
| `pos_x`    | robot x position, metres (Darab's `pos_x`)                      |
| `pos_y`    | robot y position, metres (Darab's `pos_y`)                      |
| `heading`  | robot heading, degrees (Darab's `heading`)                     |

Locked decisions (do not change without asking Asaf):

- **Grain = 1 fps.** One representative frame per second — that's the finest detail the
  LLM consumer needs on a slow, static plant scene. Darab's feed arrives at ~10 Hz; keep
  one frame per whole second and drop the rest.
- **Pose is in.** `pos_x, pos_y, heading` come straight from the API. ⚠️ Caveat to state
  in the notebook: pose is a **dead-reckoning estimate** (`PositionEstimator` integrates
  motor PWM + IMU yaw), so it **drifts** and spin-in-place changes heading without
  translation. It is an estimate, not ground truth.
- **No audio.** Audio is NOT part of `mrt_experience`. Plants are silent; the only use of
  sound is talking to the robot, which a separate voice-trigger path will handle (out of
  scope). Do not buffer or stitch audio here.
- **RAM window Y = 30 rows** (= 30 seconds, ~40 MB). Rolling: keep the last 30 seconds;
  drop older rows. This is what the LLM pulls from to orient.

---

## 3. THE BUG TO FIX (most important)

The previous attempt lives at `experiments/staging_to_action/mrt_experience.ipynb`. Its
sampling is **wrong** and you must not reproduce it.

**What it did wrong:** it held a **5-second sliding window** of *all* ~10 fps frames and
emitted an "experience" **every 2 seconds** — the whole current 5 s window. Because the
window (5 s) is larger than the cadence (2 s), **consecutive experiences overlapped by
3 s**. Their frame ranges overlapped and `first_frame_id` jumped 87 → 108 → 129 → …; the
experiences did **not** tile the timeline cleanly. Overlapping, non-consecutive grains.

**What correct looks like:** experiences must be **consecutive and non-overlapping** —
each row covers exactly one distinct second, and row *N*+1 begins where row *N* ends, with
no shared frames and no gaps. This mirrors Darab's own consumer `src/api_demo.py`, which
dedupes on `frame_id` and processes **each frame exactly once** in order.

**How to build it right:** poll the API fast (~100 Hz, well above the ~10 Hz feed) and
dedupe on `frame_id` so no frame is processed twice and none is missed. Bucket incoming
frames by the **integer second** of their timestamp; when the second rolls over, emit one
row for the second that just closed — using one representative frame from that second
(e.g. the last frame seen in it) and that second's `pos_x/pos_y/heading`. Assign `experience_id`
sequentially. The result is a clean 1-fps table with no overlap and no skips.

Verify it (section 8): consecutive rows must have consecutive `experience_id`, and their frames
must be disjoint.

---

## 4. Data source — use Darab's API exactly as intended

- Consume via `get_latest_frame()` from `src/logots_api.py`. Do **not** read the CSV
  directly and do **not** wrap the client in your own class. The returned dict now
  includes `pos_x`, `pos_y`, `heading` alongside `frame_id`, `timestamp`, `image` (RGB
  numpy), the IMU arrays, and motor/servo state.
- `get_latest_frame()` returns only the **latest** frame; polling faster than frames
  arrive returns the same frame again — hence the dedupe-on-`frame_id`.
- The recording to replay is
  `experiments/staging_to_action_v2/session_20260725_155329.csv`
  (343 rows, ~36 s, ~9.4 Hz, **colour** frames, pose columns present and the robot
  actually drives — `pos_y` sweeps ~0 → 1.3 m, `heading` covers ~0–358°).
- The recording loops in sim mode. Detect the loop (when `frame_id` jumps **backwards**)
  to capture exactly **one clean pass**: sync to the recording's start on the first wrap,
  then stop on the next wrap. (On the real robot `frame_id` only climbs, so this never
  fires — note that in the notebook.)

---

## 5. Environment & how to run

- **Notebook kernel / consumer env:** conda env **`gemma-lab`** (has numpy, PIL, IPython,
  pandas; no matplotlib — display images/audio via `IPython.display`, not matplotlib).
- **GUI / server env:** conda env **`logots`** runs Darab's GUI.
- To produce live data for the notebook:
  1. `conda run -n logots python src/logots_ui.py` (launches the GUI; serves the API on
     `:8787`).
  2. In the GUI: click **▶ SIM**, pick
     `experiments/staging_to_action_v2/session_20260725_155329.csv`, leave **LOOP** on.
  3. Run the notebook top-to-bottom with the `gemma-lab` kernel.
- Sim mode can only be started by hand (file dialog) — there is no CLI flag. Assume the
  GUI is already running and serving on `:8787`.
- Execute the notebook against the live API so real outputs are baked in
  (`jupyter nbconvert --to notebook --execute --inplace
  --ExecutePreprocessor.kernel_name=gemma-lab ...`, timeout ≥ 120 s).

---

## 6. Notebook structure (produce roughly these cells, all code + prose inline)

1. **Intro (markdown):** what `mrt_experience` is (section 1), the reality-vs-interpretation
   framing in 2–3 sentences, and that it reads Darab's live API.
2. **How to run (markdown):** section 5, so anyone can reproduce it.
3. **Connect (code):** `get_latest_frame()`, print `frame_id`, `sim_mode`, image shape,
   and the new `pos_x/pos_y/heading` to prove the pose fields arrive.
4. **Frame rate (code):** briefly measure the live rate (~10 Hz) and show `frame_id` gaps
   are 1 when polling ~100 Hz — motivating dedupe.
5. **The 1-fps table (markdown + code):** the `RollingWindow`/table builder — 1 row per
   second, consecutive, non-overlapping, dedup on `frame_id`, Y = 30 rows kept. This is
   the core; get the sampling right (section 3).
6. **Capture one pass (code):** sync-to-start, capture until the loop wraps, build the
   table, and print a summary (rows captured, `experience_id` range, that it ended on wrap).
7. **The table (code):** show the last ~30 rows as a pandas DataFrame:
   `experience_id, ts, pos_x, pos_y, heading` (+ maybe a frame-id range column to prove no
   overlap). Do NOT put raw image arrays in the table.
8. **One record, seen (code):** display one row's `frame` (via `IPython.display`) plus its
   pose, so it's showable.
9. **Recap (markdown):** the schema, the 1-fps/consecutive rule, pose-is-an-estimate,
   no-audio, and the next step (a separate reflective step hands rows to Gemma).

---

## 7. Verification checklist (assert these in the notebook or a final cell)

- `experience_id` is 1,2,3,… **consecutive**, no gaps.
- Consecutive rows are **non-overlapping**: the frames feeding row *N* and row *N*+1 are
  disjoint (e.g. row *N*+1's frame ids are all greater than row *N*'s).
- Roughly one row per second of the pass (~30 rows over ~30 s).
- `pos_x/pos_y/heading` are present and **vary** across the pass (the robot moved).
- Frames are **colour** (R, G, B channels differ).
- Ends cleanly on the loop wrap; no negative time spans.

---

## 8. Deliverable style (Asaf's preferences — follow these)

- **One self-contained notebook.** All code AND explanations inline in the cells — do NOT
  import logic from helper `.py` files. It must read top-to-bottom and be showable to
  Darab, with real outputs baked in.
- **Concise narration.** Markdown cells are 1–3 sentences, plain prose, no walls of text.
- **State clearly how to run it** (inputs, the run command, expected outputs).
- Save the notebook as
  `experiments/staging_to_action_v2/mrt_experience.ipynb`.
- Do NOT commit the recording CSV (it's large; it stays gitignored).

---

## 9. Assets & references

- `src/logots_api.py` — `get_latest_frame()`, the client to use.
- `src/api_demo.py` — Darab's reference consumer; the `frame_id`-dedupe pattern to mirror.
- `src/logots_ui.py` — the GUI/server; `PositionEstimator` (search it) shows how pose is
  computed (dead reckoning), and `FIELDNAMES` shows the row schema.
- `experiments/staging_to_action_v2/session_20260725_155329.csv` — the recording to replay.
- `experiments/staging_to_action/mrt_experience.ipynb` — the previous version. Reference
  only for what NOT to do on sampling (section 3); its window+audio design is superseded.
- `docs/v2_architecture/report.md` — the V2 architecture proposal for background.
