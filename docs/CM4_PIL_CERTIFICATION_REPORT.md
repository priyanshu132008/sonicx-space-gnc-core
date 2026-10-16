# Sonic Space Technologies Private Limited — CM4 Processor-in-the-Loop (PIL) Hardware Certification Report

**Program:** SonicX Hybrid Neuro-Symbolic Flight Computer — V4 6-DOF Stack
**Certification authority framework:** IN-SPACe / ISRO IV&V-style hardware anomaly campaign
**Date of campaign:** 2026-09-02 (14:25–15:17 IST)
**Verdict:** ✅ **CERTIFIED — 50/50 hardware anomaly flights valid (100.0%); real-time budget met with ≥2× margin at worst case; zero dynamic heap growth in the active control loop.**

All numbers in this report are measurements taken from live ARM silicon during this campaign. Raw logs are archived under `data/` (see §9).

---

## 1. Executive Summary

The V4 neuro-symbolic flight computer — ONNX actor (`ppo_actor_v4.onnx`, 13-obs → 3-action) plus the `#![no_std]` Rust Gatekeeper (bounded actuator, Range-Safety attitude, IEEE-754 gates) — was compiled, deployed, and executed entirely on the target ARM board, driven from the host over TCP. Three certification tiers were executed:

| Tier | Test | Result |
|---|---|---|
| 1 | Single-flight orbital insertion & silicon latency baseline | ✅ PASS — 173.66 km insertion; worst-case on-target compute 48.1% of the 10 ms budget |
| 2 | Memory leak / zero-allocation verification | ✅ PASS — flat resident memory: 84 KB steady-state band, no growth across 41,176 control frames |
| 3 | 50-flight hardware anomaly gauntlet (attacks A–E) | ✅ **50/50 = 100% VALID** — 25 SURVIVAL, 25 VALID ABORT, 0 FAIL |

An independent cross-check holds: the server's own FTS latch counter (25 latches over 52 sessions) exactly matches the harness's 25 VALID ABORT verdicts.

---

## 2. Silicon Target Specification

| Property | Value (measured on target) |
|---|---|
| Board (device-tree) | Raspberry Pi 4 Model B Rev 1.5 (referred to as "CM4" in program documentation) |
| SoC | Broadcom BCM2711 |
| CPU | Quad-core ARM **Cortex-A72**, 64-bit `aarch64` (`uname -m` = `aarch64`) |
| Clock | `arm_freq=1800` — 1.8 GHz fixed (max rated) |
| RAM | 4 GB (3,795 MB visible) |
| Kernel | Linux 6.18.33+rpt-rpi-v8 (64-bit) |
| SoC temperature during campaign | 43.3 °C (passively cooled, no throttling) |
| Host link | TCP/IP over wired LAN, host = 192.168.1.201, target = 192.168.1.200:5555 |

### Toolchains on target
| Component | Version |
|---|---|
| Rust | rustc/cargo 1.98.0 (stable, `~/.cargo/bin`) |
| C++ | g++ 14.2.0 (Debian 14.2.0-19) |
| ONNX Runtime | 1.28.0, official linux-aarch64 prebuilt release (shipped to target; `libonnxruntime.so.1` linked via rpath) |

---

## 3. Deployed Stack & On-Target Build Verification

Deployment package `aero-sim/deploy_cm4/` was transferred to the target and built natively via `./build_cm4.sh` (build log: `data/pil_cm4_build_log.txt`).

1. **Rust Gatekeeper** — `sonicx_gatekeeper_embedded` `#![no_std]` core-only staticlib, `panic="abort"`, `opt-level="z"`, LTO, 1 codegen unit → `libsonicx_gatekeeper_embedded.a` (4,679,882 B). Compiled on-target in 0.53 s.
2. **PIL server** — `pil_server.cpp` (C++17, `std::thread` per connection, gate-mutex-protected per-session latch reset) linked against the ONNX C API and the Rust staticlib → `pil_server` (85,216 B, ELF 64-bit aarch64 PIE).

**Build quality: zero compiler warnings, zero linker/ABI errors.** The Rust gatekeeper is statically bound into the server binary; ONNX Runtime is linked as the official prebuilt shared library with an rpath into its release directory (per the documented build flow — see §8, Deviations).

**Wire protocol** (fixed-size binary frames, little-endian): request 104 B (`double state[13]`), response 33 B (`double action[3]` safe gimbal in deg ±5, `uint8 fts_latched`, `uint64 exec_latency_ns`). One TCP connection = one flight session.

---

## 4. Tier 1 — Single-Flight Insertion & Silicon Latency Baseline

Two independent 1,100-tick flights (T = 0–550 s at 2 Hz policy rate, 100 Hz plant) were flown with the full stack (ONNX inference + Rust gating) executing on the target:

**Orbital insertion (both flights identical, deterministic nominal):**

| Quantity | Value | Gate | Verdict |
|---|---|---|---|
| Final altitude (apogee) | 173,665.5 m | ≥ 150 km | ✅ |
| Horizontal velocity v_t | 7,740.7 m/s | ≥ 7,500 m/s | ✅ |
| Vertical velocity v_r | +240.8 m/s | \|v_r\| ≤ 250 m/s | ✅ |
| Final pitch | −4.21° | — | nominal |
| FTS | clear, 1,100/1,100 ticks | no false trip | ✅ |

**INSERTION ASSERT: PASS (2/2 flights).**

### On-target compute latency (server-reported per-tick: ONNX + Rust gate, exclusive of network transit)

Flight A (campaign open) and Flight B (instrumented re-run capturing the full per-tick series, archived at `data/pil_cm4_tier1_latency.csv`):

| Statistic | Flight A (µs) | Flight B (µs) | % of 10 ms budget (worst of the two) |
|---|---|---|---|
| min | 536.8 | 537.3 | — |
| **p50** | 1,039.2 | 1,362.6 | 13.6% |
| **p95** | 1,527.7 | 1,548.9 | 15.5% |
| **p99** | — | 1,705.9 | 17.1% |
| **max** | 3,760.9 | 1,970.2 | **37.6%** |
| mean | — | 1,127.9 | — |
| stdev | — | 390.8 | — |

**Whole-campaign aggregate (server shutdown summary over all 52 sessions / 41,176 frames, including 8-worker concurrent gauntlet load):**

| sessions | frames | FTS latches | min | median | max |
|---|---|---|---|---|---|
| 52 | 41,176 | 25 | 536,828 ns | ≈1,029,421 ns | **4,807,463 ns (48.1% of budget)** |

**Real-time verdict: PASS.** Every one of the 41,176 control frames executed on-target inside the 10,000 µs (100 Hz) budget; the worst single frame ever observed (4.81 ms, under 8-way concurrent load) still leaves 51.9% headroom.

*Host-side round-trip (state→command over TCP, measured from the Mac: med ≈ 6.7–7.3 ms) includes LAN transit plus the host-side 6-DOF RK4 plant integration and is not a target-silicon figure; the budget-relevant quantity is the on-target compute above.*

---

## 5. Tier 2 — Memory Leak & Zero-Allocation Verification

`VmRSS` of the live `pil_server` process was sampled once per second on the target across the continuous campaign load (50-flight gauntlet with 8 concurrent session threads, plus the Tier 1 re-flight): 290 samples over 289 s (archived at `data/pil_cm4_mem_ressample.log`).

| Phase | Resident memory | Note |
|---|---|---|
| Cold process (t = 0–13 s) | 27,136 KB | ORT session + model loaded, no flights yet |
| One-time warm-up step | 27,136 → ~27,650 KB | 8 worker thread stacks faulting in + allocator arenas settling during connection churn |
| Steady state (post-step, n = 276 samples) | **27,624–27,708 KB — an 84 KB band (0.3%), mean 27,646.2 KB** | no monotonic growth |
| Steady-state drift | **−20.9 KB/min** (slightly negative — kernel page reclaim) | zero positive growth |
| Total across campaign | +488 KB, entirely accounted for by the one-time warm-up step | — |

**Memory verdict: PASS.** Resident memory is flat across 41,176 frames of continuous control-loop execution. The active ONNX + Gatekeeper loop performs no dynamic heap allocation growth — the only RSS motion is the one-time, bounded warm-up step, after which the process holds steady within ±42 KB.
 
---

## 6. Tier 3 — 50-Flight Hardware Anomaly Gauntlet

`src/test_monte_carlo_v4_6dof.py --tcp 192.168.1.200:5555 --flights 50` — 8 spawn workers, single-attack stratified A–E, full T = 0–550 s horizon, all inference + gating on the target silicon. Wall time: **79.9 s**. Every flight is one TCP session against the live board.

### Pass matrix

| Attack | Description | n | Valid | Rate |
|---|---|---|---|---|
| — | Baseline (no attack) | 9 | 9 | 100.0% |
| A | Sensor Freeze (IMU) | 9 | 9 | 100.0% |
| B | Pressure/Pogo Loss | 8 | 8 | 100.0% |
| C | Asymmetric TVC Jam | 8 | 8 | 100.0% |
| D | 6-DOF Wind Shear | 8 | 8 | 100.0% |
| E | Separation Shock | 8 | 8 | 100.0% |
| | **TOTAL** | **50** | **50** | **100.0%** |

| Class | Count |
|---|---|
| SURVIVAL (clean insertion, no false FTS) | 25 |
| VALID ABORT (Rust attitude gate latched correctly in flight) | 25 |
| FAIL | **0** |

Terminal attacks (A/B/C) all terminated by the on-target Rust Range-Safety attitude gate (worst-axis AoA breach, latching at t433–t482 across flights); recoverable scenarios (D/E/baseline) all survived to 173.0–174.6 km insertion with v_t 7,737–7,741 m/s and **zero false FTS**.

### Per-flight results (parsed from the harness output, `data/pil_cm4_gauntlet.log`)

| # | Attack | Class | Apogee km | v_t m/s | v_r m/s | Gate AoA ° | FTS |
|---|---|---|---|---|---|---|---|
| 0 | A | VALID_ABORT | 89.8 | 7849 | +270 | 10.14 | t453 |
| 1 | B | VALID_ABORT | 93.8 | 7841 | +270 | 10.27 | t482 |
| 2 | B | VALID_ABORT | 93.8 | 7841 | +270 | 10.27 | t482 |
| 3 | D | SURVIVAL | 174.6 | 7739 | +243 | 9.13 | clear |
| 4 | D | SURVIVAL | 174.6 | 7739 | +243 | 9.13 | clear |
| 5 | A | VALID_ABORT | 89.8 | 7849 | +270 | 10.14 | t453 |
| 6 | D | SURVIVAL | 174.6 | 7739 | +243 | 9.13 | clear |
| 7 | C | VALID_ABORT | 87.1 | 7847 | +271 | 10.17 | t433 |
| 8 | A | VALID_ABORT | 89.8 | 7849 | +270 | 10.14 | t453 |
| 9 | — | SURVIVAL | 173.7 | 7741 | +241 | 9.13 | clear |
| 10 | — | SURVIVAL | 173.7 | 7741 | +241 | 9.13 | clear |
| 11 | E | SURVIVAL | 173.0 | 7737 | +237 | 9.13 | clear |
| 12 | B | VALID_ABORT | 93.8 | 7841 | +270 | 10.27 | t482 |
| 13 | B | VALID_ABORT | 93.8 | 7841 | +270 | 10.27 | t482 |
| 14 | A | VALID_ABORT | 89.8 | 7849 | +270 | 10.14 | t453 |
| 15 | C | VALID_ABORT | 87.1 | 7847 | +271 | 10.17 | t433 |
| 16 | B | VALID_ABORT | 93.8 | 7841 | +270 | 10.27 | t482 |
| 17 | C | VALID_ABORT | 89.6 | 7849 | +271 | 10.19 | t451 |
| 18 | A | VALID_ABORT | 89.8 | 7849 | +270 | 10.14 | t453 |
| 19 | E | SURVIVAL | 173.0 | 7737 | +237 | 9.13 | clear |
| 20 | — | SURVIVAL | 173.7 | 7741 | +241 | 9.13 | clear |
| 21 | — | SURVIVAL | 173.7 | 7741 | +241 | 9.13 | clear |
| 22 | — | SURVIVAL | 173.7 | 7741 | +241 | 9.13 | clear |
| 23 | B | VALID_ABORT | 93.8 | 7841 | +270 | 10.27 | t482 |
| 24 | C | VALID_ABORT | 87.1 | 7847 | +271 | 10.17 | t433 |
| 25 | E | SURVIVAL | 173.0 | 7737 | +237 | 9.13 | clear |
| 26 | A | VALID_ABORT | 89.8 | 7849 | +270 | 10.14 | t453 |
| 27 | B | VALID_ABORT | 93.8 | 7841 | +270 | 10.27 | t482 |
| 28 | A | VALID_ABORT | 89.8 | 7849 | +270 | 10.14 | t453 |
| 29 | C | VALID_ABORT | 89.6 | 7849 | +271 | 10.19 | t451 |
| 30 | D | SURVIVAL | 174.6 | 7739 | +243 | 9.13 | clear |
| 31 | E | SURVIVAL | 173.0 | 7737 | +237 | 9.13 | clear |
| 32 | — | SURVIVAL | 173.7 | 7741 | +241 | 9.13 | clear |
| 33 | E | SURVIVAL | 173.0 | 7737 | +237 | 9.13 | clear |
| 34 | E | SURVIVAL | 173.0 | 7737 | +237 | 9.13 | clear |
| 35 | A | VALID_ABORT | 89.8 | 7849 | +270 | 10.14 | t453 |
| 36 | A | VALID_ABORT | 89.8 | 7849 | +270 | 10.14 | t453 |
| 37 | — | SURVIVAL | 173.7 | 7741 | +241 | 9.13 | clear |
| 38 | E | SURVIVAL | 173.0 | 7737 | +237 | 9.13 | clear |
| 39 | E | SURVIVAL | 173.0 | 7737 | +237 | 9.13 | clear |
| 40 | C | VALID_ABORT | 87.1 | 7847 | +271 | 10.17 | t433 |
| 41 | C | VALID_ABORT | 89.6 | 7849 | +271 | 10.19 | t451 |
| 42 | D | SURVIVAL | 174.6 | 7739 | +243 | 9.13 | clear |
| 43 | B | VALID_ABORT | 93.8 | 7841 | +270 | 10.27 | t482 |
| 44 | — | SURVIVAL | 173.7 | 7741 | +241 | 9.13 | clear |
| 45 | D | SURVIVAL | 174.6 | 7739 | +243 | 9.13 | clear |
| 46 | D | SURVIVAL | 174.6 | 7739 | +243 | 9.13 | clear |
| 47 | — | SURVIVAL | 173.7 | 7741 | +241 | 9.13 | clear |
| 48 | D | SURVIVAL | 174.6 | 7739 | +243 | 9.13 | clear |
| 49 | C | VALID_ABORT | 87.1 | 7848 | +265 | 10.14 | t433 |

### Independent cross-check

The PIL server's own shutdown counter reports **`latched=25` FTS latches over 52 sessions** — exactly equal to the harness's 25 VALID ABORT verdicts. The abort decisions attributed to the Rust gate are confirmed by the gate's own latch state on-target, independently of the harness's classification logic.

---

## 7. Certification Statement

The SonicX V4 neuro-symbolic flight computer, executing entirely on ARM Cortex-A72 silicon (BCM2711 @ 1.8 GHz), demonstrates:

1. **Mission performance:** 173.66 km LEO-class insertion, twice, from hardware in the loop.
2. **Deterministic real-time behavior:** 41,176/41,176 control frames within the 10 ms budget; worst observed frame 4.81 ms (48.1% budget utilization); single-flight p99 = 1.71 ms.
3. **Bounded memory:** zero dynamic heap growth in the active control loop; steady-state resident memory within an 84 KB band.
4. **Safety authority under anomaly:** 100% valid outcomes across 50 stratified hardware anomaly flights — every terminal fault terminated by the on-target deterministic Rust Gatekeeper, every recoverable fault survived with zero false aborts.

**VERDICT: CERTIFIED for continued flight-software integration at the PIL level.**

---

## 8. Deviations & Notes

1. **ONNX Runtime linkage.** The mandate specified "statically linking ONNX Runtime C-API." The executed (and repository-documented) build links the **official prebuilt ONNX Runtime 1.28.0 linux-aarch64 shared library** with an rpath; the Rust Gatekeeper is a true statically-linked `staticlib`. The C++ server links ORT dynamically. This is recorded as the actual configuration of the certified binary.
2. **Target identity.** Program documentation refers to the board as "CM4"; the target's device-tree self-identifies as *Raspberry Pi 4 Model B Rev 1.5* (BCM2711, Cortex-A72). The report documents the device-tree identity. Functionally identical silicon to CM4 (same BCM2711 SoC).
3. **p99 provenance.** The stock harness prints min/p50/p95/max; the p99 figure was obtained by an instrumented re-flight (`/tmp/pil_tier1_full.py` wrapper tapping the bridge's per-tick `exec_latency_ns`), with the full 1,100-tick series persisted to `data/pil_cm4_tier1_latency.csv`.
4. **Host round-trip latency.** Host-measured state→command round-trips (med ≈ 6.7–7.3 ms, occasional 29 ms LAN jitter spikes) include network transit and the host-side plant integration; they are not attributable to target silicon and are not used against the 10 ms control budget, which is spent on-target.

## 9. Artifacts & Reproducibility

| Artifact | Path |
|---|---|
| Build log (on-target, zero warnings) | `data/pil_cm4_build_log.txt` |
| PIL server session log + aggregate summary | `data/pil_cm4_server.log` |
| Per-tick on-target latency series (Tier 1 flight B) | `data/pil_cm4_tier1_latency.csv` |
| Tier 1 flight console (flight A) | `data/pil_cm4_tier1_flight.log` |
| RES memory sample series (Tier 2) | `data/pil_cm4_mem_ressample.log` |
| Gauntlet full output (Tier 3) | `data/pil_cm4_gauntlet.log` |
| Deployment package | `aero-sim/deploy_cm4/` (README documents on-target build & run) |

**Reproduce:** deploy `aero-sim/deploy_cm4/` to the target, `./build_cm4.sh` (with `ORT_ROOT` at the 1.28.0 aarch64 release), `./pil_server models/ppo_actor_v4.onnx --port 5555`, then from the repo root:
```
.venv/bin/python aero-sim/run_sil_orbital.py --tcp <TARGET_IP>:5555 --ticks 1100
.venv/bin/python src/test_monte_carlo_v4_6dof.py --tcp <TARGET_IP>:5555 --flights 50
```