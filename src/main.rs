//! Main entry point for the DeltaX2 application.
//!
//! This binary assembles the Slint UI and the Rust robot control library.
//! All robot I/O runs on a dedicated worker thread so the touch UI stays
//! responsive during long operations (homing, seeding a whole plate).
//!
//! Threading model:
//! - The UI thread runs the Slint event loop and never talks to the serial
//!   port directly. Callbacks send `RobotCommand`s over an mpsc channel.
//! - The worker thread owns `DeltaRobot` and processes commands one at a
//!   time. It pushes results back to the UI with `Weak::upgrade_in_event_loop`.
//! - Stop/pause/continue bypass the channel: they drive a shared
//!   `SeedingControl` that the seeding loop checks between pots, so they
//!   take effect even while the worker is busy running a job. Every queued
//!   job carries an increasing id and a stop records the id it targets
//!   (abort watermark), so a stop can neither be erased while the job still
//!   waits in the queue nor leak into a later job.

use deltax2::SerialCommunication;
use deltax2::robot::{Axis, Config, Coord3D, DeltaRobot, Plate, SeedOutcome, SeedingControl};
use slint::ComponentHandle;
use std::cell::{Cell, RefCell};
use std::rc::Rc;
use std::sync::{Arc, mpsc};

slint::include_modules!();

/// Commands sent from the UI thread to the robot worker thread.
///
/// Each command is processed to completion before the next one is taken,
/// so the worker never interleaves two hardware operations.
enum RobotCommand {
    /// Open the serial port and perform the `IsDelta` handshake.
    Connect { port: String, baud_rate: u32 },
    /// Jog one axis by a relative displacement in mm.
    MoveAxis(Axis, f32),
    /// Update the rotation axis state by a relative amount.
    MoveCart(f32),
    /// Run the `G28` mechanical homing sequence.
    HomeXyz,
    /// Reset the rotation axis state.
    HomeCart,
    /// Run the automated seeding sequence for the given plate. The id was
    /// assigned by the UI at queue time and is matched against the abort
    /// watermark in `SeedingControl`.
    SeedPlate { plate: Plate, job_id: u64 },
}

/// Application entry point.
///
/// Initializes the UI, loads `config.toml`, spawns the robot worker thread,
/// wires all UI callbacks, and runs the Slint event loop. The initial serial
/// connection is performed asynchronously by the worker so the window
/// appears immediately even when the robot is off.
///
/// # Errors
///
/// Returns an error if the UI fails to initialize or if the mandatory
/// configuration file cannot be loaded or parsed.
fn main() -> anyhow::Result<()> {
    // Initialize the Slint window
    let ui = AppWindow::new()?;

    // Load configuration from config.toml
    // We expect this file to be in the same directory as the executable.
    let config_path = "config.toml";
    let config_text = std::fs::read_to_string(config_path)
        .map_err(|e| anyhow::anyhow!("Failed to read config file {}: {}", config_path, e))?;
    // Deserialize the TOML content into our Config struct
    let config: Config = toml::from_str(&config_text)
        .map_err(|e| anyhow::anyhow!("Failed to parse config: {}", e))?;
    let config = Rc::new(config);

    // Apply UI settings from configuration (e.g., Kiosk mode for RPi)
    if config.ui.kiosk_mode {
        ui.set_kiosk_mode(true);
    }

    // Validate the plate geometry against the software limits up front: a
    // bad config.toml entry must surface at startup, not at pot 47 of a
    // seeding job. Invalid plates are excluded from the selection list so a
    // job can never start on one.
    let mut plates: Vec<Plate> = Vec::new();
    let mut config_errors: Vec<String> = Vec::new();
    for plate in &config.plates {
        match plate.validate(&config.robot) {
            Ok(()) => plates.push(plate.clone()),
            Err(e) => config_errors.push(e),
        }
    }
    let plates = Rc::new(plates);
    for error in &config_errors {
        println!("Config error: {}", error);
    }

    // Populate the plate selection model in the UI.
    // This allows the user to choose between different tray types in the 'Confirm Plate' screen.
    let plate_names: Vec<slint::SharedString> =
        plates.iter().map(|p| p.name.clone().into()).collect();
    let plate_names_model = Rc::new(slint::VecModel::from(plate_names));
    ui.set_plate_names(plate_names_model.into());

    // Shared pause/abort flags for the seeding job. The UI thread writes
    // them; the worker's seeding loop reads them between pots.
    let control = Arc::new(SeedingControl::new());

    // Spawn the worker thread that owns the robot and the serial port.
    let tx = spawn_robot_worker(
        ui.as_weak(),
        control.clone(),
        config.robot.limit_min.clone(),
        config.robot.limit_max.clone(),
    );

    // Kick off the initial connection asynchronously. The UI shows
    // "Connecting..." meanwhile instead of freezing for the handshake timeout.
    // A config error takes precedence in the status bar so the operator sees
    // it at least until the connection result overwrites it (a persistent
    // error surface is tracked in issue #8).
    if let Some(error) = config_errors.first() {
        ui.set_status_text(format!("Config error: {}", error).into());
    } else {
        ui.set_status_text("Connecting...".into());
    }
    ui.set_is_connected(false);
    dispatch(
        &tx,
        RobotCommand::Connect {
            port: config.serial.port.clone(),
            baud_rate: config.serial.baud_rate,
        },
        &ui.as_weak(),
    );

    // --- Movement Handlers ---
    // Each jog button click becomes a queued command for the worker.

    let t = tx.clone();
    let uh = ui.as_weak();
    ui.on_move_x(move |d| {
        dispatch(&t, RobotCommand::MoveAxis(Axis::X, d), &uh);
    });

    let t = tx.clone();
    let uh = ui.as_weak();
    ui.on_move_y(move |d| {
        dispatch(&t, RobotCommand::MoveAxis(Axis::Y, d), &uh);
    });

    let t = tx.clone();
    let uh = ui.as_weak();
    ui.on_move_z(move |d| {
        dispatch(&t, RobotCommand::MoveAxis(Axis::Z, d), &uh);
    });

    let t = tx.clone();
    let uh = ui.as_weak();
    ui.on_move_cart(move |d| {
        dispatch(&t, RobotCommand::MoveCart(d), &uh);
    });

    // --- Homing Handlers ---

    let t = tx.clone();
    let uh = ui.as_weak();
    ui.on_home_xyz(move || {
        dispatch(&t, RobotCommand::HomeXyz, &uh);
    });

    let t = tx.clone();
    let uh = ui.as_weak();
    ui.on_home_cart(move || {
        dispatch(&t, RobotCommand::HomeCart, &uh);
    });

    // --- System / Utility Handlers ---

    // --- Serial port management (Configuration screen) ---
    // The port list and reconnect action live on a touch surface, because
    // stdout is invisible in kiosk mode. Enumeration is degraded on Linux
    // (no libudev), so the configured port is always kept in the list as the
    // authoritative fallback the operator can reconnect to.

    let configured_port = config.serial.port.clone();
    let baud_rate = config.serial.baud_rate;

    // Seed the picker with the configured port and an initial enumeration so
    // the Configuration screen is usable before the first Refresh.
    ui.set_selected_port(configured_port.clone().into());
    ui.set_com_ports(Rc::new(slint::VecModel::from(enumerate_ports(&configured_port))).into());

    let uh = ui.as_weak();
    let cfg_port = configured_port.clone();
    ui.on_refresh_com_ports(move || {
        if let Some(ui) = uh.upgrade() {
            ui.set_com_ports(Rc::new(slint::VecModel::from(enumerate_ports(&cfg_port))).into());
        }
    });

    let t = tx.clone();
    let uh = ui.as_weak();
    ui.on_reconnect(move || {
        if let Some(ui) = uh.upgrade() {
            let port = ui.get_selected_port().to_string();
            if port.is_empty() {
                return;
            }
            // Immediate pending feedback: the worker settles it to
            // "Connected" / "... failed" when the handshake returns.
            ui.set_is_connected(false);
            ui.set_status_text(format!("Connecting to {}...", port).into());
            dispatch(&t, RobotCommand::Connect { port, baud_rate }, &uh);
        }
    });

    // --- Seeding Flow ---
    // Selecting a plate only records the choice; the job starts when the
    // user confirms on the 'Load the seeds' screen (start-seeding).

    let selected_plate: Rc<RefCell<Option<Plate>>> = Rc::new(RefCell::new(None));

    let sel = selected_plate.clone();
    let valid_plates = plates.clone();
    let uh = ui.as_weak();
    ui.on_plate_selected(move |id| {
        // Index into the validated list backing the UI model, not the raw
        // config, so the indices always line up with what is displayed.
        if let Some(plate) = valid_plates.get(id as usize) {
            *sel.borrow_mut() = Some(plate.clone());
            if let Some(ui) = uh.upgrade() {
                ui.set_status_text(format!("Selected plate: {}", plate.name).into());
            }
        }
    });

    // Monotonic id of the last queued seeding job. Lives on the UI thread
    // only; the worker learns the id through the SeedPlate command.
    let last_job_id: Rc<Cell<u64>> = Rc::new(Cell::new(0));

    let sel = selected_plate.clone();
    let t = tx.clone();
    let uh = ui.as_weak();
    let job = last_job_id.clone();
    ui.on_start_seeding(move || {
        let plate = sel.borrow().clone();
        if let Some(plate) = plate {
            if let Some(ui) = uh.upgrade() {
                ui.set_seeding_progress("Starting...".into());
                // Fresh job: clear any pause state left visible from before.
                ui.set_seeding_paused(false);
                ui.set_seeding_transition(false);
            }
            let job_id = job.get() + 1;
            job.set(job_id);
            dispatch(&t, RobotCommand::SeedPlate { plate, job_id }, &uh);
        } else if let Some(ui) = uh.upgrade() {
            // Should not happen through the normal flow, but guard anyway.
            ui.set_status_text("No plate selected".into());
        }
    });

    // Stop/pause/continue drive the shared control directly instead of
    // sending commands: the worker is busy inside seed_plate at this point
    // and would not see a queued command until the job ended.

    let c = control.clone();
    let uh = ui.as_weak();
    let job = last_job_id.clone();
    ui.on_stop_seeding(move || {
        // Abort the last queued job — whether it is already running or
        // still waiting in the command queue.
        c.request_abort(job.get());
        if let Some(ui) = uh.upgrade() {
            ui.set_status_text("Stopping...".into());
        }
    });

    let c = control.clone();
    let uh = ui.as_weak();
    ui.on_pause_seeding(move || {
        c.request_pause();
        if let Some(ui) = uh.upgrade() {
            ui.set_status_text("Seeding paused".into());
        }
    });

    let c = control.clone();
    let uh = ui.as_weak();
    ui.on_continue_seeding(move || {
        c.resume();
        if let Some(ui) = uh.upgrade() {
            ui.set_status_text("Seeding resumed".into());
        }
    });

    // Start the Slint event loop. This blocks until the window is closed.
    ui.run()?;

    Ok(())
}

/// Builds the serial-port list shown on the Configuration screen.
///
/// The configured port always comes first and is guaranteed present, even
/// when OS enumeration returns nothing (degraded on Linux without libudev):
/// it is the authoritative path the operator must always be able to
/// reconnect to. Any additionally detected ports follow, deduplicated.
fn enumerate_ports(configured_port: &str) -> Vec<slint::SharedString> {
    let mut ports: Vec<slint::SharedString> = vec![configured_port.into()];
    for p in SerialCommunication::list_ports() {
        if p != configured_port {
            ports.push(p.into());
        }
    }
    ports
}

/// Spawns the robot worker thread and returns the command sender.
///
/// The worker owns the `DeltaRobot` (and through it the serial port) for its
/// whole lifetime, processing commands sequentially until every sender is
/// dropped (i.e., when the UI shuts down).
fn spawn_robot_worker(
    ui: slint::Weak<AppWindow>,
    control: Arc<SeedingControl>,
    limit_min: Coord3D,
    limit_max: Coord3D,
) -> mpsc::Sender<RobotCommand> {
    let (tx, rx) = mpsc::channel::<RobotCommand>();

    std::thread::spawn(move || {
        let mut robot = DeltaRobot::new();
        robot.set_limits(limit_min, limit_max);

        while let Ok(cmd) = rx.recv() {
            match cmd {
                RobotCommand::Connect { port, baud_rate } => {
                    // Release any handle held from a previous session before
                    // reopening — a reconnect may target a different port, and
                    // we must not leak the old one.
                    robot.disconnect();
                    match robot.connect(&port, baud_rate) {
                        Ok(()) => set_status(&ui, "Connected".into(), Some(true)),
                        Err(e) => set_status(
                            &ui,
                            format!("Connection to {} failed: {}", port, e),
                            Some(false),
                        ),
                    }
                }

                RobotCommand::MoveAxis(axis, d) => {
                    match robot.move_axis(axis, d) {
                        Ok(()) => set_status(&ui, "Move OK".into(), None),
                        Err(e) => set_status(&ui, format!("Error: {}", e), None),
                    }
                    push_position(&ui, &robot);
                }

                RobotCommand::MoveCart(d) => {
                    if let Err(e) = robot.move_cart(d) {
                        set_status(&ui, format!("Error moving Cart: {}", e), None);
                    }
                    push_position(&ui, &robot);
                }

                RobotCommand::HomeXyz => {
                    set_status(&ui, "Homing...".into(), None);
                    match robot.home_xyz() {
                        Ok(()) => set_status(&ui, "Homing complete".into(), None),
                        Err(e) => set_status(&ui, format!("Error homing XYZ: {}", e), None),
                    }
                    push_position(&ui, &robot);
                }

                RobotCommand::HomeCart => {
                    if let Err(e) = robot.home_cart() {
                        set_status(&ui, format!("Error homing Cart: {}", e), None);
                    }
                    push_position(&ui, &robot);
                }

                RobotCommand::SeedPlate { plate, job_id } => {
                    // Clear a pause left over from a previous job. A stop
                    // needs no clearing: the abort watermark only affects
                    // job ids at or below it, so a stop registered for this
                    // very job (while it waited in the queue) still holds.
                    control.begin_job();
                    set_status(&ui, format!("Seeding plate: {}", plate.name), None);

                    let progress_ui = ui.clone();
                    let pause_ui = ui.clone();
                    let result = robot.seed_plate(
                        &plate,
                        job_id,
                        &control,
                        move |done, total| {
                            let _ = progress_ui.upgrade_in_event_loop(move |ui| {
                                ui.set_seeding_progress(format!("Pot {} / {}", done, total).into());
                            });
                        },
                        // Worker-confirmed pause transitions: clear the pending
                        // flag and reflect the real paused state (issue #17).
                        move |paused| {
                            let _ = pause_ui.upgrade_in_event_loop(move |ui| {
                                ui.set_seeding_paused(paused);
                                ui.set_seeding_transition(false);
                            });
                        },
                    );

                    match result {
                        Ok(SeedOutcome::Completed) => {
                            set_status(&ui, "Seeding complete".into(), None);
                            leave_seeding_screen(&ui);
                        }
                        Ok(SeedOutcome::Aborted) => {
                            // The stop button already navigated back to Main.
                            set_status(&ui, "Seeding stopped".into(), None);
                        }
                        Err(e) => {
                            // A job that died mid-plate needs operator action
                            // (inspect the tray, re-home, retry) — full text on
                            // the persistent banner, not the elided status bar.
                            set_status(&ui, "Seeding failed".into(), None);
                            set_error(&ui, format!("Seeding failed: {}", e));
                            leave_seeding_screen(&ui);
                        }
                    }
                    push_position(&ui, &robot);
                }
            }
        }
    });

    tx
}

/// Operator-facing message shown when the robot worker thread can no longer
/// be reached (a command send failed). The worker owns the only handle to the
/// serial port, so once it is gone the robot is uncontrollable until restart.
const WORKER_DEAD_MSG: &str = "Robot control has stopped unexpectedly. Restart the application — the robot cannot be controlled until then.";

/// Sends a command to the worker from the UI thread, raising the persistent
/// error surface if the worker thread has died (`send` returns `Err` once the
/// receiver is gone). Without this, a dead worker would leave the UI running
/// with no symptom while every button silently does nothing.
///
/// Must be called on the UI thread (it is, from Slint callbacks), so it
/// touches the error property directly.
fn dispatch(tx: &mpsc::Sender<RobotCommand>, cmd: RobotCommand, ui: &slint::Weak<AppWindow>) {
    if tx.send(cmd).is_err() {
        eprintln!("{}", WORKER_DEAD_MSG);
        if let Some(ui) = ui.upgrade() {
            ui.set_error_text(WORKER_DEAD_MSG.into());
        }
    }
}

/// Raises the persistent, full-text error surface (a dismissible banner) for
/// failures that require operator attention, distinct from the transient
/// one-line status bar. Mirrors to the console for SSH debugging.
///
/// Safe to call from the worker thread: the UI mutation is scheduled onto
/// the Slint event loop.
fn set_error(ui: &slint::Weak<AppWindow>, msg: String) {
    println!("{}", msg);
    let _ = ui.upgrade_in_event_loop(move |ui| {
        ui.set_error_text(msg.into());
    });
}

/// Shows a status message in the UI status bar (and mirrors it to the
/// console for SSH debugging). Optionally updates the connection indicator.
///
/// Safe to call from the worker thread: the UI mutation is scheduled onto
/// the Slint event loop.
fn set_status(ui: &slint::Weak<AppWindow>, msg: String, connected: Option<bool>) {
    println!("{}", msg);
    let _ = ui.upgrade_in_event_loop(move |ui| {
        ui.set_status_text(msg.into());
        if let Some(c) = connected {
            ui.set_is_connected(c);
        }
    });
}

/// Pushes the robot's current logical position and connection state to the
/// UI after each processed command, so a link lost during a command flips
/// the connection LED without any extra plumbing.
///
/// Safe to call from the worker thread.
fn push_position(ui: &slint::Weak<AppWindow>, robot: &DeltaRobot) {
    let (x, y, z, cart) = robot.get_position();
    let connected = robot.is_connected();
    let _ = ui.upgrade_in_event_loop(move |ui| {
        ui.set_head_x(format!("{:.3}", x).into());
        ui.set_head_y(format!("{:.3}", y).into());
        ui.set_head_z(format!("{:.3}", z).into());
        ui.set_head_cart(format!("{:.3}", cart).into());
        ui.set_is_connected(connected);
    });
}

/// Returns the UI to the main screen if it is still showing the seeding
/// screen (used when a job finishes or fails on its own; a user stop has
/// already navigated away).
fn leave_seeding_screen(ui: &slint::Weak<AppWindow>) {
    let _ = ui.upgrade_in_event_loop(|ui| {
        if ui.get_current_screen() == ScreenState::Seeding {
            ui.set_current_screen(ScreenState::Main);
        }
    });
}
