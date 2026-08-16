# Fix Gemma-on-GPU OOM via JetPack in-place upgrade (r36.4.7 → r36.5)

**Status:** paused for Jetson reboot, 2026-08-02.
**Source:** authoritative version of this plan (from the plan-mode session that
did the web research) lives at `/home/logots/.claude/plans/bright-waddling-conway.md`.
This file is a copy kept in the repo so it's easy to find after the reboot.

## Context

`audio_on_demand.py --brain llamacpp` (built in a previous session) works correctly but
**CPU-only**: GPU offload of the Gemma 4 E2B GGUF fails with `cudaMalloc failed: out of
memory` on a single ~1.3 GB tensor buffer, despite ~5 GB free RAM. Web research confirmed
this is a **known NVIDIA memory-allocator bug specific to Jetson Linux r36.4.7** (this
Jetson's exact version, confirmed via `/etc/nv_tegra_release`) — NVIDIA staff acknowledged
it on their forums and confirmed it's **fixed in r36.5** (JetPack 6.2.2). Multiple
independent reports (llama.cpp GitHub discussion #16706, an NVIDIA forum thread about this
exact model on this exact device) match our symptoms precisely. No amount of flag-tuning
in llama.cpp fixes an allocator bug — the real fix is the OS update.

The upgrade path is an **in-place APT upgrade**, not a full reflash: edit the L4T apt
source file to point at `r36.5`, then `apt update && apt dist-upgrade`. This preserves
existing config, packages, and data.

**Two real constraints raised the risk profile, and shape every step below:**
- **No OS-level battery visibility on this device** — `/sys/class/power_supply/` is empty
  (confirmed). Whatever is giving "~4 hours left" is external (a UPS/power bank) and
  invisible to Linux. This is a hard manual checkpoint the user owns, not something
  Claude can gate on automatically.
- **Headless/remote-only access** (NoMachine at 192.168.68.114:4000). If the upgrade
  breaks something and the robot doesn't come back on the network, recovery needs physical
  access (or NVIDIA SDK Manager over USB from the MacBook) — not doable remotely.
  SSH (confirmed active+enabled) is a lighter-weight fallback than NoMachine's virtual
  desktop and more likely to survive a driver/display hiccup, so it's the primary channel
  to watch across the reboot.

Given these two constraints, a mid-upgrade power loss (corrupting the kernel/bootloader
write) is the single worst outcome — worse than any GPU speed question. **The plan is
sequenced so nothing irreversible happens until AC power is explicitly confirmed.**

## Facts confirmed on this machine already
- Exact repo file to edit: `/etc/apt/sources.list.d/nvidia-l4t-apt-source.list` (3 lines,
  all currently `r36.4`, content read and confirmed).
- SSH: `active`/`enabled` right now — a second, independent way back in besides NoMachine.
- Swap: 3.7 GiB total / 2.9 GiB free — fine for an APT package upgrade (this is not the
  from-source kernel-patch route, which needed 8 GB swap; that route is explicitly NOT
  being used here since the APT path is NVIDIA's own officially documented fix).
- Disk: 412 GB free — plenty of headroom for downloaded packages.
- Currently running/relevant to stop cleanly first: nothing found running as of the last
  check (`logots_ui.py`, `audio_on_demand.py` were both cleaned up) — re-check at
  execution time in case something was started since.

## Steps

### 0. Hard gate — confirm AC/wall power
Do not proceed past this point on battery alone. Confirm the robot is on wall power (or
the UPS is charging/topped up) as the very first action before touching any file.

### 1. Pre-flight snapshot (read-only, reversible)
- `cat /etc/nv_tegra_release` and `dpkg -l | grep nvidia-l4t > ~/pkg_before_r36.4.7.txt`
  — baseline for comparison/rollback reference.
- `sudo cp /etc/apt/sources.list.d/nvidia-l4t-apt-source.list{,.bak}` — one-line, trivially
  reversible backup of the file we're about to edit.
- Confirm no GUI/audio pipeline processes are running (`ps aux | grep -E "logots_ui|audio_on_demand"`);
  stop them cleanly if so.
- Open (or confirm) an **SSH session** to the Jetson in a separate window, kept open
  independent of NoMachine, specifically to watch the reboot.

### 2. Edit the apt source (reversible via the .bak from step 1)
In `/etc/apt/sources.list.d/nvidia-l4t-apt-source.list`, change all three `r36.4` → `r36.5`:
```
deb https://repo.download.nvidia.com/jetson/common r36.5 main
deb https://repo.download.nvidia.com/jetson/t234 r36.5 main
deb https://repo.download.nvidia.com/jetson/ffmpeg r36.5 main
```

### 3. Update + upgrade
```
sudo apt update
sudo apt dist-upgrade
```
Expect this to pull in new `nvidia-l4t-core`, `nvidia-l4t-kernel`, `nvidia-l4t-cuda`, etc.
— can take 15-45+ min depending on network speed. If prompted about a modified config file
during the upgrade, default to **keep the currently-installed version** for anything that
looks related to our custom I2C/camera device-tree setup (jetson-io.py-applied overlays)
rather than accepting the maintainer's version — CLAUDE.md flags that config as hard-won.
If apt reports broken/held packages afterward:
```
sudo apt install --fix-broken -o Dpkg::Options::="--force-overwrite"
```

### 4. Reboot
```
sudo reboot
```
Watch the SSH session (not just NoMachine) for the host to come back — expect ~1-3 min.

### 5. Post-reboot verification, in order (each gates the next)
1. **Does it come back at all?** SSH in, then check NoMachine.
2. **Version confirmed:** `cat /etc/nv_tegra_release` → should show `R36 (release), REVISION: 5.x`.
3. **Hardware still works** (the things CLAUDE.md calls hard-won, so check first before
   declaring victory): `i2cdetect -y -r 1` still shows `0x08`; briefly launch
   `conda run -n logots python src/logots_ui.py` and confirm camera feed, IMU, and Arduino
   connection all still initialize with no new errors, then close it.
4. **The actual bug — GPU load test:** rerun the exact command that failed last session,
   *without* `CUDA_VISIBLE_DEVICES=""` this time:
   ```
   ./build/bin/llama-mtmd-cli -m <model.gguf> --mmproj <mmproj.gguf> \
     --audio experiments/audio_on_demand/recordings/tts_water_ficus.wav \
     -sys "$BRAIN_PROMPT" -p " " -n 500 -c 4096 --temp 0 --jinja --gpu-layers 999
   ```
   Confirm no `cudaMalloc failed: out of memory` and it actually loads on CUDA0.
5. **If GPU load succeeds:** rerun `audio_on_demand.py --brain llamacpp --once` on all 3
   test wavs (same ones from last session) and compare latency to the CPU-only baseline
   (75-145s) — expect a large drop, closer to Asaf's ~11-14 tok/s estimate. Remove the
   `CUDA_VISIBLE_DEVICES=""` env override in `LlamaCppBrain.decide()` (src/audio_on_demand.py)
   once GPU offload is confirmed working, and update the class's docstring/comment
   accordingly (it currently documents the CPU-only workaround as necessary).

### 6. If something goes wrong
- **apt dist-upgrade fails/partial, before reboot:** don't reboot yet. Try
  `sudo dpkg --configure -a` then `sudo apt install --fix-broken`, report exact errors.
- **Doesn't come back after reboot (neither SSH nor NoMachine) within ~5 min:** this needs
  physical access to the robot, or NVIDIA SDK Manager recovery over USB from the MacBook.
- **Rollback if upgraded-but-broken:** restore `nvidia-l4t-apt-source.list` from `.bak`,
  `sudo apt update`, and reinstall pinned versions from `pkg_before_r36.4.7.txt` — messy but
  possible. Full reflash via SDK Manager is the last-resort safety net, and is physical/USB,
  not something doable remotely.

## Verification checklist
- [x] Step 0 gate: AC power explicitly confirmed before any file is touched.
- [x] `/etc/nv_tegra_release` shows R36.5.x post-reboot. (R36.5.0, confirmed 2026-08-02)
- [x] I2C/camera/GUI still initialize cleanly. **Two regressions found and fixed post-upgrade:**
      camera (boot loader `DEFAULT` reset to `primary`, losing the `JetsonIO` overlay entry — fixed via
      `jetson-io.py` → "Configure for compatible hardware"), and mic/speaker (I2S pins 12/35/38/40 found
      `unused` — turned out unrelated to this upgrade, a pre-existing drift from a Jun 4 manual pin edit;
      fixed via `jetson-io.py` → "Configure header pins manually", re-adding the `i2s2` pin group). Full
      details in CLAUDE.md "Known issues". User confirmed all sensors working in the GUI after both fixes.
- [x] `llama-mtmd-cli --gpu-layers 999` loads the Gemma GGUF on CUDA0 without OOM. Confirmed via
      `tegrastats`: GR3D_FREQ sustained 85-99%, GPU temp 53°C→59°C, power draw ~1.9W→7-8.5W during the
      run — real GPU compute, not a silent CPU fallback.
- [x] `audio_on_demand.py` GPU latency improves over CPU baseline. One wav tested
      (`tts_water_ficus.wav`): ~30s end-to-end vs. the 75-145s CPU-only baseline.

## Remaining (paused here, 2026-08-02, to continue another day)
- [x] Remove the `CUDA_VISIBLE_DEVICES=""` override in `LlamaCppBrain.decide()`
      (`src/audio_on_demand.py`) now that GPU offload is confirmed working. Done 2026-08-16 as
      part of the persistent-`llama-server` latency rewrite (see
      `.claude/plans/refactored-mixing-pie.md`) — GPU offload is now the default, no override.
- [x] Update that class's docstring — it still says GPU offload OOMs and explains the CPU-only
      workaround, which is no longer true. Done 2026-08-16, same rewrite.
- [x] Benchmark all 4 test wavs against the new persistent-server path. Done 2026-08-16:
      `tts_water_ficus.wav` → `water_plant` [2.2s], `tts_question.wav` → `inspect_plant` [1.9s],
      `tts_impossible.wav` → `speak` refusal [2.8s], `tts_combined.wav` → `water_plant` [2.0s] —
      all via `--brain llamacpp --once` in the `logots-audio` conda env, down from ~20-30s.
      Getting under these numbers required one more fix beyond the persistent server itself:
      this GGUF's chain-of-thought reasoning trace was ~300 tokens (~10s) per call for no
      accuracy benefit on this schema-constrained task — `--reasoning off` on the llama-server
      startup cut warm latency from ~12-30s to ~1-2s per call.
- [ ] Review the full diff on `src/audio_on_demand.py` (still uncommitted) and commit.

## Quick resume command
```bash
git -C /home/logots/Desktop/Logots_V2 diff src/audio_on_demand.py
```
to pick the WIP-code context back up alongside this plan.
