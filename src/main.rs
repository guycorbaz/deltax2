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
//! - Stop/pause/continue bypass the channel: they set atomic flags in a
//!   shared `SeedingControl` that the seeding loop checks between pots, so
//!   they take effect even while the worker is busy running a job.

use slint::ComponentHandle;
use std::cell::RefCell;
use std::rc::Rc;
use std::sync::atomic::Ordering;
use std::sync::{mpsc, Arc};
use deltax2::robot::{Axis, Config, Coord3D, DeltaRobot, Plate, SeedOutcome, SeedingControl};
use deltax2::SerialCommunication;

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
    /// Run the automated seeding sequence for the given plate.
    SeedPlate(Plate),
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

    // Populate the plate selection model in the UI.
    // This allows the user to choose between different tray types in the 'Confirm Plate' screen.
    let plate_names: Vec<slint::SharedString> =
        config.plates.iter().map(|p| p.name.clone().into()).collect();
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
    ui.set_status_text("Connecting...".into());
    ui.set_is_connected(false);
    let _ = tx.send(RobotCommand::Connect {
        port: config.serial.port.clone(),
        baud_rate: config.serial.baud_rate,
    });

    // --- Movement Handlers ---
    // Each jog button click becomes a queued command for the worker.

    let t = tx.clone();
    ui.on_move_x(move |d| { let _ = t.send(RobotCommand::MoveAxis(Axis::X, d)); });

    let t = tx.clone();
    ui.on_move_y(move |d| { let _ = t.send(RobotCommand::MoveAxis(Axis::Y, d)); });

    let t = tx.clone();
    ui.on_move_z(move |d| { let _ = t.send(RobotCommand::MoveAxis(Axis::Z, d)); });

    let t = tx.clone();
    ui.on_move_cart(move |d| { let _ = t.send(RobotCommand::MoveCart(d)); });

    // --- Homing Handlers ---

    let t = tx.clone();
    ui.on_home_xyz(move || { let _ = t.send(RobotCommand::HomeXyz); });

    let t = tx.clone();
    ui.on_home_cart(move || { let _ = t.send(RobotCommand::HomeCart); });

    // --- System / Utility Handlers ---

    ui.on_list_com_ports(move || {
        // Enumerate detected serial ports and print them to console for debugging.
        let ports = SerialCommunication::list_ports();
        println!("Available ports: {:?}", ports);
    });

    // --- Seeding Flow ---
    // Selecting a plate only records the choice; the job starts when the
    // user confirms on the 'Load the seeds' screen (start-seeding).

    let selected_plate: Rc<RefCell<Option<Plate>>> = Rc::new(RefCell::new(None));

    let sel = selected_plate.clone();
    let cfg = config.clone();
    let uh = ui.as_weak();
    ui.on_plate_selected(move |id| {
        if let Some(plate) = cfg.plates.get(id as usize) {
            *sel.borrow_mut() = Some(plate.clone());
            if let Some(ui) = uh.upgrade() {
                ui.set_status_text(format!("Selected plate: {}", plate.name).into());
            }
        }
    });

    let sel = selected_plate.clone();
    let t = tx.clone();
    let uh = ui.as_weak();
    ui.on_start_seeding(move || {
        let plate = sel.borrow().clone();
        if let Some(plate) = plate {
            if let Some(ui) = uh.upgrade() {
                ui.set_seeding_progress("Starting...".into());
            }
            let _ = t.send(RobotCommand::SeedPlate(plate));
        } else if let Some(ui) = uh.upgrade() {
            // Should not happen through the normal flow, but guard anyway.
            ui.set_status_text("No plate selected".into());
        }
    });

    // Stop/pause/continue write the shared flags directly instead of sending
    // commands: the worker is busy inside seed_plate at this point and would
    // not see a queued command until the job ended.

    let c = control.clone();
    let uh = ui.as_weak();
    ui.on_stop_seeding(move || {
        c.abort.store(true, Ordering::SeqCst);
        if let Some(ui) = uh.upgrade() {
            ui.set_status_text("Stopping...".into());
        }
    });

    let c = control.clone();
    let uh = ui.as_weak();
    ui.on_pause_seeding(move || {
        c.pause.store(true, Ordering::SeqCst);
        if let Some(ui) = uh.upgrade() {
            ui.set_status_text("Seeding paused".into());
        }
    });

    let c = control.clone();
    let uh = ui.as_weak();
    ui.on_continue_seeding(move || {
        c.pause.store(false, Ordering::SeqCst);
        if let Some(ui) = uh.upgrade() {
            ui.set_status_text("Seeding resumed".into());
        }
    });

    // Start the Slint event loop. This blocks until the window is closed.
    ui.run()?;

    Ok(())
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

                RobotCommand::SeedPlate(plate) => {
                    // Clear any stop/pause left over from a previous job.
                    // Done here (not when the UI queues the job) so a stop
                    // aimed at a still-running job cannot be erased early.
                    control.reset();
                    set_status(&ui, format!("Seeding plate: {}", plate.name), None);

                    let progress_ui = ui.clone();
                    let result = robot.seed_plate(&plate, &control, move |done, total| {
                        let _ = progress_ui.upgrade_in_event_loop(move |ui| {
                            ui.set_seeding_progress(format!("Pot {} / {}", done, total).into());
                        });
                    });

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
                            set_status(&ui, format!("Error seeding plate: {}", e), None);
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

/// Pushes the robot's current logical position to the UI coordinate readouts.
///
/// Safe to call from the worker thread.
fn push_position(ui: &slint::Weak<AppWindow>, robot: &DeltaRobot) {
    let (x, y, z, cart) = robot.get_position();
    let _ = ui.upgrade_in_event_loop(move |ui| {
        ui.set_head_x(format!("{:.3}", x).into());
        ui.set_head_y(format!("{:.3}", y).into());
        ui.set_head_z(format!("{:.3}", z).into());
        ui.set_head_cart(format!("{:.3}", cart).into());
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
