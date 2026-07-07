---
project_name: 'deltax2'
user_name: 'Guy'
date: '2026-07-03'
sections_completed: ['technology_stack', 'language_rules', 'framework_rules', 'testing_rules', 'quality_rules', 'workflow_rules', 'critical_rules']
existing_patterns_found: 19
status: 'complete'
rule_count: 57
optimized_for_llm: true
---

# Project Context for AI Agents

_This file contains critical rules and patterns that AI agents must follow when implementing code in this project. Focus on unobvious details that agents might otherwise miss._

Hardware reference: [Delta X 2 Basic Kit](https://docs.deltaxrobot.com/products/deltax2/deltax2_basic_kit/) — official product documentation for the robot this application controls.

---

## Technology Stack & Versions

_Authoritative versions live in `Cargo.toml` / committed `Cargo.lock`; numbers here are a snapshot as of 2026-07._

- **Rust, edition 2024** — MSRV declared in `Cargo.toml` (`rust-version`, currently 1.90 — driven by the dependency tree, not the edition).
- **Dependencies** — caret bounds, not pins; `Cargo.lock` is authoritative. Do not bump anything unless explicitly asked. Invariant: `slint` and `slint-build` must be the exact same version — bump both together or not at all.
- **Slint (UI)** — compiled at build time by `build.rs`; never loaded at runtime.
- **No async runtime — deliberate.** Blocking I/O on a dedicated worker thread + `std::sync::mpsc`. Never introduce tokio/async-std/`tokio-serial`; the G-code protocol is strictly sequential.
- **serialport with `default-features = false`** (no libudev) — port enumeration is degraded on Linux; the port path always comes from `config.toml`. Do not build features relying on `available_ports()`; do not re-enable default features (aarch64/Pi build impact).
- **Errors** — `anyhow::Result` + `.context()` everywhere; deliberate: no `thiserror`, no custom error enums. Never make logic or tests depend on error message text. Errors must reach the UI, never just stdout — stdout is invisible in kiosk operation (see Critical Implementation Rules).
- **Quality gate (no CI — the agent is the CI)** — before considering any change done: `cargo build && cargo clippy --all-targets -- -D warnings && cargo fmt --check && cargo test` (default rustfmt/clippy configs; keep the tree warning-free).
- **Target** — Raspberry Pi 3, 4 or 5 (64-bit OS, `aarch64-unknown-linux-gnu`) + official Raspberry Pi Touch Display (7'', 800×480), fixed-size window, touch-only kiosk — no keyboard, no mouse. Pi 3 is the performance floor: keep the app lean, no heavy per-frame work. `SLINT_BACKEND=linuxkms` on the Pi only (desktop dev uses the default backend). Release builds compile natively on the Pi; no cross-compilation setup exists.

## Critical Implementation Rules

### Language-Specific Rules (Rust)

- **Rustdoc is enforced:** `src/lib.rs` has `#![warn(missing_docs)]` — every public item needs `///` docs (`# Arguments` / `# Errors` / `# Returns` where relevant); modules open with `//!`. The `-D warnings` gate makes violations build errors.
- **The worker thread must never panic** — a panic silently kills robot control while the touch UI keeps running. Enforced by `#![warn(clippy::unwrap_used, clippy::expect_used)]` in the lib. Also: no direct indexing (use `.get()` + fallback), keep the worker loop as `while let Ok(cmd) = rx.recv()`. `unwrap()` is fine inside `#[cfg(test)]`.
- **`DeltaRobot` is owned exclusively by the worker thread** (`spawn_robot_worker`, `src/main.rs`). Never wrap it in `Arc<Mutex<…>>`, never call it from the UI thread. A new robot operation = a new `RobotCommand` variant, handled sequentially in the worker loop. Sole exception: the `SeedingControl` methods (stop/pause/continue). Abort is a job-id watermark (`request_abort`/`should_abort` with UI-assigned increasing job ids) — never revert it to a resettable flag; only a leftover pause is cleared at dequeue (`begin_job`).
- **A failed `tx.send` to the worker means the worker is dead** — surface it in the UI; never silence it with `let _ =`.
- **A multi-command G-code sequence that fails midway leaves tracked position and firmware mode unreliable** (e.g. `move_axis`'s G91→G0→G90). On such errors, mark the state desynchronized and require re-homing before the next move — do not just bubble the message up.
- **Atomics use `Ordering::SeqCst`** — project convention; do not downgrade to `Relaxed`/`Acquire`.
- **Reuse the coordinate types** (`Coord2D`, `Coord3D`, `IntCoord2D` in `src/robot.rs`): `f32` for mm, `i32` for pot counts. No tuples, no new coordinate structs, no `f64`. Never compare f32 coordinates with `==`/`!=` — bound comparisons or explicit tolerance only.
- **The config structs in `src/robot.rs` ARE the `config.toml` schema.** Any change updates the struct, `config.toml`, and `docs/chapters/configuration.tex` (then rebuild `manual.pdf`) — same change. New fields need `#[serde(default)]` with a safe value: a parse failure bricks deployed Pis (the app exits at startup).
- **lib/bin split:** reusable logic lives in the library (`robot`, `serial`); `src/main.rs` is glue only. New robot behavior goes in the lib.
- **Decision logic is exposed as pure functions before the anyhow wrapping** (limit checks, pot-position math, `has_ok_line`) so it is testable without hardware. Sequencing logic (protocol handshakes, ok-waiting, seeding loop) is tested through the `Transport` trait (`src/serial.rs`) with a hand-written scripted mock — never against the concrete `SerialCommunication`, never against a real port. `Transport::read_data` contract: `Ok(empty)` = nothing yet (poll timeout), `Err` = link lost.

### Framework-Specific Rules (Slint)

- **Thread affinity:** Slint types are not `Send`. The UI is mutated ONLY from the event loop. From the worker thread: `slint::Weak<AppWindow>` + `upgrade_in_event_loop(move |ui| { … })` — exact API, no other mechanism. Applies to models too: `Rc<VecModel>` is not `Send`; never mutate a model from the worker.
- **A UI callback body is one `tx.send(RobotCommand::…)` and nothing else** — no robot I/O, no waiting, no sleeps in callbacks. Sole exception: stop/pause/continue drive the shared `SeedingControl` directly.
- **UI types are generated:** `AppWindow`, `ScreenState`, … are generated into OUT_DIR by `build.rs` and pulled in via `slint::include_modules!()` (`src/main.rs`). They do not exist in `src/` — don't search for them there, don't create Rust twins. Single `slint_build::compile("ui/appwindow.slint")` entry point — never add a second.
- **A new `.slint` component must be imported (directly or transitively) from `ui/appwindow.slint`**, or it is silently not compiled. `.slint` errors surface at `cargo build`, not runtime; `cargo build` is the only automated UI validation — visual rendering is human validation.
- **Naming:** kebab-case in `.slint` ↔ snake_case in Rust (`status-text` ↔ `set_status_text`, `move-x` ↔ `on_move_x`). Never grep a kebab-case name in `src/` or a snake_case name in `ui/`. UI files: lowercase concatenated, one per screen (`confirmplate.slint`).
- **Rust↔UI contract lives on `AppWindow` only:** properties/callbacks declared on the root; sub-screens forward events up. Do not wire callbacks on sub-components from Rust.
- **Displayed values are formatted Rust-side** (`format!("{:.3}", x)`) and exposed as `string` properties — don't convert to numeric properties or move formatting into `.slint`. Lists: `[string]` ↔ `Rc<VecModel<SharedString>>`. Exception: values that belong inside a *translatable* template (e.g. "Pot X / Y") are pushed as numbers and formatted in `.slint` with `@tr`, so the surrounding words can be translated — see the Strings rule.
- **Navigation is state-driven:** the `ScreenState` enum + conditional `if` blocks decide what is visible. New screen = new variant + conditional block + navigation callbacks. No router, no dynamic loading, no parallel visibility booleans, no Slint `global` for app state. `kiosk-mode` drives `no-frame`.
- **Async feedback:** any action that sends a `RobotCommand` must immediately show a pending state ("Stopping…", "Homing…", disabled button) and settle only on worker confirmation. Optimistic navigation (Stop → Main, deliberate: perceived responsiveness on a keyboardless kiosk — don't "fix" it into a blocking wait) is allowed only if the pending state stays visible on the destination screen. The UI must never look idle while the robot moves.
- **The status bar is not a diagnostic channel:** the one-line elided status text takes short transient states only. Errors requiring operator action need a dedicated touch surface (persistent banner/dialog, full text, action button). Nothing the operator needs may exist only on stdout (existing anti-pattern: `list-com-ports`).
- **Design system:** shared visual tokens live in `ui/theme.slint` (a `global Theme` of colour/type/metric CONSTANTS — light theme, green accent; not app state) and reusable components in `ui/widgets.slint`. Use `Theme.*` for colours/sizes and the `AppButton` component (variants: `primary` filled green, `danger` filled red, default secondary; optional recoloured `icon`) instead of raw std-widgets `Button` or hard-coded hex. New screens set `background: Theme.bg`; panels are cards (`Theme.surface` + `Theme.border` + `Theme.radius`). The `Ok` confirm button is `primary`. Keep everything ≥60px (`Theme.touch`).
- **Touch targets:** existing buttons are 60px tall (160×60 main screen, 70×60 seeding) — match that (60px is the shared height across every screen); ≥60px for primary/destructive actions (gloved operator, ~133 ppi). Never place Stop adjacent to Pause/Continue without spacing. No hover states, no keyboard shortcuts, no scroll-only access to critical actions.
- **Screen content area is 800×440, not 480:** the 40px status bar sits below the content in the root `VerticalLayout`. Sub-screens use absolute positioning, so any fixed-height content (axis pads, lists) must be laid out to fit 440px and stay clear of the footer (`parent.height - 60 - 15`). Designing against 480px overlaps the footer — the mistake fixed in issue #25.
- **Strings:** `.slint` labels use `@tr("…")`. Operator-facing text pushed from Rust goes through *status/error codes*, not literals: Rust sets a `StatusKind`/`ErrorKind` enum (+ a dynamic `status-arg`/`error-arg` for a port, error detail, plate name) and the `.slint` `status-line`/`error-line` functions turn it into translated text with `@tr` (issue #18). Add a message = new enum variant + a branch in that function; keep dynamic data in the arg, never baked into the template. Console/SSH mirrors (`println!`) stay English on purpose (admin context). Don't reintroduce raw English UI strings or a Rust-side translation mechanism.
- **The seeding flow is two-step by design:** `plate-selected` records the choice; the job starts only from `start-seeding` (ConfirmSeed OK). Do not merge these steps.

### Testing Rules

- **No tests exist yet — this is debt, not a norm.** Any new pure logic ships with tests in the same change. Debt repayment priority (descending risk):
  1. limit checks (physical safety — extract the pure decision function first, see Rust rules),
  2. plate/pot geometry in `seed_plate` (one wrong offset = a whole tray seeded wrong),
  3. `create_mv_command` G-code formatting (wire contract with the firmware),
  4. `config.toml` deserialization,
  5. `SeedingControl` flag semantics (testable today, no seam needed).
- **Safety first:** software-limit and plate/pot-geometry tests take precedence over all other tests. `seed_pot` must not leave its placeholder state without tests proving every generated position respects the limits.
- **Characterization before change:** before modifying untested code, first write tests locking in the current correct behavior (the wire contract with the firmware is asserted byte for byte in `src/robot.rs` tests).
- **Standard harness only:** inline `#[cfg(test)] mod tests` in the file under test, run by `cargo test` (no `tests/` directory). No test dev-dependencies (no mockall/proptest/rstest); the mock transport (`MockTransport` in `src/robot.rs` tests) is hand-written — scripted responses released per write, implementing the `Transport` trait. (`unwrap()` in tests: see Rust rules.)
- **Tests never touch hardware:** never open a real serial port, never depend on a connected robot or a display — headless on any machine. Doc-tests count in `cargo test`: any doc example touching hardware is marked `no_run`; pure examples (geometry, G-code) stay runnable.
- **Fixtures:** build `Plate`/`Config` via `toml::from_str` on inline TOML snippets — never by reading the repo's `config.toml` (no CWD dependency; also exercises `#[serde(default)]`). A `#[cfg(test)] fn test_plate() -> Plate` helper per module suffices.
- **No coverage theater:** expected values are hand-computed literals — never derived by repeating the formula under test. Limit tests must cover the boundaries: exactly at `limit_min`/`limit_max` and an epsilon beyond, both signs (Z is negative) — the tests pin down the inclusive/exclusive semantics.
- **f32 assertions use an explicit tolerance with a message:** `assert!((a - b).abs() < 0.01, "got {a}, want {b}")` — 0.01 mm is far below mechanical precision. Never `assert_eq!` on floats. Exception: serialized G-code is asserted by exact string equality (`assert_eq!(cmd, "G0 X10.0000 FEEDBACK:ok\n")`) — `{:.4}` makes it deterministic; never parse it back into floats.
- **Never assert on anyhow error message text.** Test the pure decision functions instead (see Rust rules).
- **Timeout tests go through the `Clock` seam, never real `sleep`s** (slow, flaky). `DeltaRobot` is generic over a `Clock` (`src/robot.rs`, issue #19): production uses `SystemClock` (`Instant` + `thread::sleep`); tests inject `MockClock`, whose time advances only when `sleep` is called, so a "no `ok` ever arrives" wait reaches its timeout instantly and the class (2s/5s/10s) is asserted on the virtual clock. Build robots for timeout tests with `DeltaRobot::with_transport_and_clock`.
- **UI testing: see Slint rules** — `cargo build` is the only automated UI validation; visual rendering is human-validated.

### Code Quality & Style Rules

- **Enforcement lives in the toolchain, not in prose:** the quality gate (see Technology Stack) + lint attributes in `src/lib.rs` (`missing_docs`, `clippy::unwrap_used`, `clippy::expect_used`). Default rustfmt/clippy configs — do not add config files to tune them.
- **Comment style:** `///` rustdoc on public items (see Rust rules); inline `//` comments explain the *why* (constraints, safety reasoning), matching the existing density — this codebase is deliberately heavily commented.
- **Naming:** follow existing patterns — `actual_*` for tracked state, `limit_*` for bounds; kebab↔snake across the UI boundary (see Slint rules).

### Development Workflow Rules

- **`cargo run` requires `config.toml` in the current working directory** — the app exits otherwise. Run from the repo root.
- **The quality gate passes before any commit:** `cargo build && cargo clippy --all-targets -- -D warnings && cargo fmt --check && cargo test`.
- **Editing any `docs/chapters/*.tex` requires rebuilding the committed manual** (`latexmk -pdf manual.tex` inside `docs/`) — `manual.pdf` must stay in sync.
- **Release builds compile natively on the Pi** — nothing in the repo may assume a cross-compilation toolchain.
- **Desktop dev uses the default Slint backend**; `SLINT_BACKEND=linuxkms` is Pi-only.

### Critical Don't-Miss Rules

- **stdout/stderr are invisible in production** (kiosk, no terminal). `println!` is SSH-debug only. Anything the operator needs — errors, diagnostics, reconnect, progress — must reach the UI on a touch surface (see Slint rules). Existing violation, do not imitate: `list-com-ports` prints to console only.
- **Every hardware movement goes through `DeltaRobot`** — its limit check *before sending* is the last software barrier before physical damage. Never write G-code to the serial port directly; never bypass `move_axis`/`home_xyz`.
- **Position is logical dead reckoning, never read back from hardware.** `actual_x/y/z/cart` is reset only by homing. Any code path that moves hardware without updating tracking (or vice versa) silently breaks the safety limits. After a failed multi-command sequence the state is desynchronized (see Rust rules).
- **Z0 is the TOP of the workspace** (homed position); working Z values are negative (`limit_max.z = 0.0` in config). Getting the sign wrong drives the head down into the tray.
- **Software-only placeholders look like hardware control but aren't:** `move_cart`/`home_cart` update internal state only; `seed_pot()` is empty. Mark any new placeholder as such in its rustdoc (`Note: software-state only, no hardware command`); never assume these already move hardware.
- **Plate geometry from `config.toml` is currently NOT validated at load** — a zero/negative grid or an out-of-limits origin surfaces only mid-seeding. When touching config loading, add fail-fast validation at startup with a UI-visible error — never let it be discovered at pot 47.
- **The robot protocol has fixed semantics** (full reference: `docs/chapters/gcode.tex`, manual Appendix A): handshake `IsDelta`→`YesDelta` (2 s), every command carries `FEEDBACK:ok` and is awaited (2 s mode switches / 5 s moves / 10 s homing), jogs wrapped in `G91`…`G90`. Do not invent G-code — check the manual appendix first.

---

## Usage Guidelines

**For AI Agents:**

- Read this file before implementing any code
- Follow ALL rules exactly as documented
- When in doubt, prefer the more restrictive option
- Update this file if new patterns emerge

**For Humans:**

- Keep this file lean and focused on agent needs
- Update when the technology stack or patterns change
- Review periodically for outdated rules (version numbers are a dated snapshot; `Cargo.toml`/`Cargo.lock` are authoritative)
- Remove rules that become obvious over time

Last Updated: 2026-07-03
