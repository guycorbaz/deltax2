# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Control application for the Delta X 2 robot (a delta-arm seeding robot), written in Rust with a Slint UI. Ported from a C++/Qt codebase. Targets a Raspberry Pi with a 7'' touch screen (fixed 800x480 window), communicating with the robot firmware over a serial port using G-code.

## Commands

```bash
cargo build            # build (build.rs compiles ui/appwindow.slint via slint-build)
cargo run              # run the app; reads config.toml from the current working directory
cargo build --release  # release build for deployment
cargo test             # no tests exist yet; test infrastructure is standard cargo
cargo doc --open       # internal API docs (code is heavily doc-commented)
```

The app requires `config.toml` in the working directory at startup (it exits with an error otherwise) and attempts a serial connection immediately, but keeps running with a "Disconnected" status if the robot is unreachable.

On Raspberry Pi, run with `SLINT_BACKEND=linuxkms` (see `documentation/raspberry_pi.md`).

## Architecture

Three layers, glued together in `src/main.rs`:

1. **UI (Slint)** — `ui/*.slint`, compiled at build time by `build.rs` and included in Rust via `slint::include_modules!()`. `ui/appwindow.slint` is the root component: it owns a `ScreenState` enum and switches between screens (Main → Calibration / Configuration / the three-step seeding flow ConfirmPlate → ConfirmSeed → Seeding) with conditional `if` elements. All Rust↔UI communication goes through properties and callbacks declared on `AppWindow`; sub-screens forward their events up to it.

2. **Robot logic** — `src/robot.rs` (`DeltaRobot`). Generates G-code, tracks the head position logically (`actual_x/y/z/cart` — position is not read back from hardware except implicitly via homing reset), and enforces software safety limits *before* sending any move command. All config structs (`Config`, `Plate`, `SerialConfig`, `RobotConfig`, `UIConfig`) live here and deserialize from `config.toml` via serde.

3. **Serial transport** — `src/serial.rs` (`SerialCommunication`). Thin wrapper over the `serialport` crate with a 10ms read timeout; raw bytes only, no protocol knowledge.

`src/lib.rs` exposes `robot` and `serial` as a library; `src/main.rs` is the binary that wires UI callbacks to `DeltaRobot` methods. The robot is shared across callbacks as `Rc<RefCell<DeltaRobot>>` — everything runs single-threaded on the Slint event loop, so long robot operations (homing, seeding a whole plate) currently block the UI.

## Robot Protocol Essentials

Full reference in `documentation/deltax2_gcode.md`.

- Connection handshake: send `IsDelta\n`, expect `YesDelta` (checked case-insensitively) within 2s.
- Synchronization: every command is sent with a `FEEDBACK:ok` suffix and the code blocks in `wait_for_ok()` until `ok` arrives (per-command timeouts: 2s for mode switches, 5s for moves, 10s for homing).
- Jog moves wrap the `G0` in `G91` (relative) … `G90` (absolute) mode switches.
- `G28` homes to the top; Z0 is the top of the workspace, so working Z values are negative.
- The "cart" (rotation) axis is currently software-state only — `move_cart`/`home_cart` update internal tracking without sending hardware commands. `seed_pot()` is a placeholder awaiting the real per-pot move/tool sequence.

## Configuration

`config.toml` defines the serial port, kiosk mode, safety limits (`[robot] limit_min/limit_max`), and an array of `[[plates]]` describing seeding tray geometries (pot grid size, spacing, origin). Plate names populate the UI selection list at startup; adding a tray type is a config-only change. The `Config` structs in `src/robot.rs` must stay in sync with this file's schema.
