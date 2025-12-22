//! Main entry point for the DeltaX2 application.
//!
//! This binary assembles the Slint UI and the Rust robot control library.
//! It handles configuration loading, event callbacks from the UI, and
//! state updates from the robot hardware. It acts as the "glue" between
//! the user interface and the underlying hardware control logic.

use slint::ComponentHandle;
use std::rc::Rc;
use std::cell::RefCell;
use deltax2::robot::{DeltaRobot, Axis, Config};
use deltax2::SerialCommunication;

slint::include_modules!();

/// Application entry point.
///
/// This function performs the following initialization steps:
/// 1. Initializes the Slint `AppWindow`.
/// 2. Loads the system configuration from `config.toml`.
/// 3. Registers UI models (like the list of available seeding plates).
/// 4. Sets up the `DeltaRobot` controller and attempts an initial connection.
/// 5. Hooks up all UI callbacks to their respective Rust handlers.
/// 6. Starts the main Slint event loop.
///
/// # Errors
///
/// Returns an error if the UI fails to initialize or if the mandatory
/// configuration file cannot be loaded or parsed.
fn main() -> anyhow::Result<()> {
    // Initialize the Slint window
    let ui = AppWindow::new()?;
    let ui_handle = ui.as_weak();

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
    let plate_names: Vec<slint::SharedString> = config.plates.iter().map(|p| p.name.clone().into()).collect();
    let plate_names_model = Rc::new(slint::VecModel::from(plate_names));
    ui.set_plate_names(plate_names_model.into());

    // Initialize the robot control interface. 
    // We wrap it in Rc<RefCell<...>> because it needs to be shared across multiple UI callbacks.
    let robot = Rc::new(RefCell::new(DeltaRobot::new()));
    
    // Apply safety limits from the configuration file to the robot controller.
    robot.borrow_mut().set_limits(config.robot.limit_min.clone(), config.robot.limit_max.clone());

    // Attempt initial connection to the robot hardware using the configured serial port.
    if let Err(e) = robot.borrow_mut().connect(&config.serial.port, config.serial.baud_rate) {
        let msg = format!("Initial connection to {} failed: {}", config.serial.port, e);
        eprintln!("{}", msg);
        // Feedback to the user via the custom status bar in the UI.
        ui.set_status_text(msg.into());
        ui.set_is_connected(false);
    } else {
        ui.set_status_text("Connected".into());
        ui.set_is_connected(true);
    }

    // --- Movement Handlers ---
    // These link Slint UI signals (callbacks) to Rust logic calls for manual jogging.

    let r = robot.clone(); let uh = ui_handle.clone();
    ui.on_move_x(move |d| handle_move(&uh, &r, Axis::X, d));

    let r = robot.clone(); let uh = ui_handle.clone();
    ui.on_move_y(move |d| handle_move(&uh, &r, Axis::Y, d));

    let r = robot.clone(); let uh = ui_handle.clone();
    ui.on_move_z(move |d| handle_move(&uh, &r, Axis::Z, d));

    let r = robot.clone(); let uh = ui_handle.clone();
    ui.on_move_cart(move |displacement| {
        // Handle rotation moves. Note: move_cart currently only updates internal state.
        if let Err(e) = r.borrow_mut().move_cart(displacement) {
            let msg = format!("Error moving Cart: {}", e);
            eprintln!("{}", msg);
            if let Some(ui) = uh.upgrade() { ui.set_status_text(msg.into()); }
        }
        update_position(&uh, &r.borrow());
    });

    // --- Homing Handlers ---
    // These trigger the G28 mechanical homing sequence.

    let r = robot.clone(); let uh = ui_handle.clone();
    ui.on_home_xyz(move || {
        if let Err(e) = r.borrow_mut().home_xyz() {
            let msg = format!("Error homing XYZ: {}", e);
            eprintln!("{}", msg);
            if let Some(ui) = uh.upgrade() { ui.set_status_text(msg.into()); }
        } else {
            if let Some(ui) = uh.upgrade() { ui.set_status_text("Homing complete".into()); }
        }
        // Always refresh position display after homing
        update_position(&uh, &r.borrow());
    });

    let r = robot.clone(); let uh = ui_handle.clone();
    ui.on_home_cart(move || {
        if let Err(e) = r.borrow_mut().home_cart() {
            let msg = format!("Error homing Cart: {}", e);
            eprintln!("{}", msg);
            if let Some(ui) = uh.upgrade() { ui.set_status_text(msg.into()); }
        }
        update_position(&uh, &r.borrow());
    });

    // --- System / Utility Handlers ---

    ui.on_list_com_ports(move || {
        // Enumerate detected serial ports and print them to console for debugging.
        let ports = SerialCommunication::list_ports();
        println!("Available ports: {:?}", ports);
    });

    // Process plate selection from the 'Confirm Plate' screen.
    let robot_seed = robot.clone();
    let config_seed = config.clone();
    let uh_seed = ui_handle.clone();
    ui.on_plate_selected(move |id| {
        let index = id as usize;
        // Lookup the selected plate configuration
        if let Some(plate) = config_seed.plates.get(index) {
            let msg = format!("Seeding plate: {}", plate.name);
            println!("{}", msg);
            if let Some(ui) = uh_seed.upgrade() { ui.set_status_text(msg.into()); }
            
            // Start the semi-automated seeding process for the whole plate.
            if let Err(e) = robot_seed.borrow_mut().seed_plate(plate) {
                let err_msg = format!("Error seeding plate: {}", e);
                eprintln!("{}", err_msg);
                if let Some(ui) = uh_seed.upgrade() { ui.set_status_text(err_msg.into()); }
            } else {
                if let Some(ui) = uh_seed.upgrade() { ui.set_status_text("Seeding complete".into()); }
            }
        }
    });

    // Start the Slint event loop. This blocks until the window is closed.
    ui.run()?;

    Ok(())
}

/// Helper function to handle axis movement and provide consistent UI state feedback.
///
/// # Arguments
///
/// * `ui_handle` - A weak reference to the main window for status updates.
/// * `robot` - Shared reference to the robot controller.
/// * `axis` - The target axis to move.
/// * `d` - The relative displacement in mm.
fn handle_move(ui_handle: &slint::Weak<AppWindow>, robot: &Rc<RefCell<DeltaRobot>>, axis: Axis, d: f32) {
    if let Err(e) = robot.borrow_mut().move_axis(axis, d) {
        let msg = format!("Error: {}", e);
        eprintln!("{}", msg);
        if let Some(ui) = ui_handle.upgrade() {
            ui.set_status_text(msg.into());
        }
    } else {
        // Successful move
        if let Some(ui) = ui_handle.upgrade() {
            ui.set_status_text("Move OK".into());
        }
    }
    // Refresh the coordinate readouts in the UI
    update_position(ui_handle, &robot.borrow());
}

/// Updates the position displays in the UI with current robot coordinates.
///
/// # Arguments
///
/// * `ui_handle` - A weak reference to the main window.
/// * `robot` - Immutable reference to the robot controller.
fn update_position(ui_handle: &slint::Weak<AppWindow>, robot: &DeltaRobot) {
    if let Some(ui) = ui_handle.upgrade() {
        let (x, y, z, cart) = robot.get_position();
        // Update the bound Slint properties; these will automatically refresh the UI display.
        ui.set_head_x(format!("{:.3}", x).into());
        ui.set_head_y(format!("{:.3}", y).into());
        ui.set_head_z(format!("{:.3}", z).into());
        ui.set_head_cart(format!("{:.3}", cart).into());
    }
}
