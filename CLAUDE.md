# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Control application for the Delta X 2 robot (a delta-arm seeding robot), written in Rust with a Slint UI. Ported from a C++/Qt codebase. Deployment target is a Raspberry Pi (3, 4 or 5) with the official 7'' touch display (fixed 800x480 window), communicating with the robot firmware over a serial port using G-code.

Daily operation is **touch-only** (kiosk mode, no keyboard/mouse, stdout invisible): anything the operator needs at runtime — error messages, diagnostics, reconnect, stop/pause — must be available in the UI with finger-sized touch targets. Administrative tasks (editing `config.toml`, deployment, console diagnostics) are done over SSH or a local terminal, so keyboard-driven setup is acceptable there.

## Commands

```bash
cargo build            # build (build.rs compiles ui/appwindow.slint via slint-build)
cargo run              # run the app; reads config.toml from the current working directory
cargo build --release  # release build for deployment
cargo test             # no tests exist yet; test infrastructure is standard cargo
cargo doc --open       # internal API docs (code is heavily doc-commented)
```

The app requires `config.toml` in the working directory at startup (it exits with an error otherwise) and attempts a serial connection immediately, but keeps running with a "Disconnected" status if the robot is unreachable.

On Raspberry Pi, run with `SLINT_BACKEND=linuxkms` (see the Raspberry Pi chapter of the manual).

## Architecture

Three layers, glued together in `src/main.rs`:

1. **UI (Slint)** — `ui/*.slint`, compiled at build time by `build.rs` and included in Rust via `slint::include_modules!()`. `ui/appwindow.slint` is the root component: it owns a `ScreenState` enum and switches between screens (Main → Calibration / Configuration / the three-step seeding flow ConfirmPlate → ConfirmSeed → Seeding) with conditional `if` elements. All Rust↔UI communication goes through properties and callbacks declared on `AppWindow`; sub-screens forward their events up to it.

2. **Robot logic** — `src/robot.rs` (`DeltaRobot`). Generates G-code, tracks the head position logically (`actual_x/y/z/cart` — position is not read back from hardware except implicitly via homing reset), and enforces software safety limits *before* sending any move command. All config structs (`Config`, `Plate`, `SerialConfig`, `RobotConfig`, `UIConfig`) live here and deserialize from `config.toml` via serde.

3. **Serial transport** — `src/serial.rs` (`SerialCommunication`). Thin wrapper over the `serialport` crate with a 10ms read timeout; raw bytes only, no protocol knowledge.

`src/lib.rs` exposes `robot` and `serial` as a library; `src/main.rs` is the binary that wires the two together. Threading model: all robot I/O runs on a dedicated worker thread that owns `DeltaRobot`. UI callbacks send `RobotCommand` enum values over an mpsc channel (processed strictly sequentially); the worker pushes results back with `slint::Weak::upgrade_in_event_loop`. Stop/pause/continue for a seeding job do NOT go through the channel — the worker is busy inside `seed_plate` then — they drive a shared `SeedingControl` (`src/robot.rs`) that the seeding loop checks between pots. Abort is a job-id watermark: the UI assigns each queued job an increasing id and a stop records the id it targets, so a stop can neither be erased while the job still waits in the queue nor leak into a later job. The worker only clears a leftover pause when it dequeues a job (`begin_job`).

The seeding flow is two-step in the UI: `plate-selected` only records the chosen `Plate`; the job is sent to the worker by `start-seeding`, fired from the ConfirmSeed screen's OK button.

## Documentation

The user/administrator manual lives in `docs/`: `manual.tex` (main file + title page), `preamble.tex` (packages, colors, box/listing styles), and one file per chapter in `chapters/`. Build with `latexmk -pdf manual.tex` (or `pdflatex` twice) inside `docs/`; keep the committed `manual.pdf` in sync after edits. The former `deltax2_gcode.md` and `raspberry_pi.md` were absorbed into `chapters/gcode.tex` and `chapters/raspberrypi.tex` — update those, not markdown files.

## Robot Protocol Essentials

Full reference in `docs/chapters/gcode.tex` (manual Appendix A).

- Connection handshake: send `IsDelta\n`, expect `YesDelta` (checked case-insensitively) within 2s.
- Synchronization: every command is sent with a `FEEDBACK:ok` suffix and the code blocks in `wait_for_ok()` until `ok` arrives (per-command timeouts: 2s for mode switches, 5s for moves, 10s for homing).
- Jog moves wrap the `G0` in `G91` (relative) … `G90` (absolute) mode switches.
- `G28` homes to the top; Z0 is the top of the workspace, so working Z values are negative.
- The "cart" (rotation) axis is currently software-state only — `move_cart`/`home_cart` update internal tracking without sending hardware commands. `seed_pot()` is a placeholder awaiting the real per-pot move/tool sequence.

## Configuration

`config.toml` defines the serial port, kiosk mode, safety limits (`[robot] limit_min/limit_max`), and an array of `[[plates]]` describing seeding tray geometries (pot grid size, spacing, origin). Plate names populate the UI selection list at startup; adding a tray type is a config-only change. The `Config` structs in `src/robot.rs` must stay in sync with this file's schema.
