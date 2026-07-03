//! Robot control and configuration module.
//!
//! This module defines the high-level `DeltaRobot` interface, coordinate structures,
//! and configuration types required to operate the Delta X 2 robot.
//! It handles G-code generation, safety limit enforcement, and synchronization
//! with the physical hardware.

use crate::serial::SerialCommunication;
use anyhow::Result;
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

/// Represents a 2D coordinate (X, Y).
/// Typically used for pot locations or planar movements.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Coord2D {
    /// Planar X coordinate in mm.
    pub x: f32,
    /// Planar Y coordinate in mm (the vertical axis is Z, see [`Coord3D`]).
    pub y: f32,
}

/// Represents a 3D coordinate (X, Y, Z).
/// Used for robot head positions and plate dimensions.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Coord3D {
    /// X coordinate in mm.
    pub x: f32,
    /// Y coordinate in mm.
    pub y: f32,
    /// Z (height) coordinate in mm.
    pub z: f32,
}

/// Represents a 2D integer coordinate.
/// Typically used for discrete counts, such as the number of pots in a grid.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IntCoord2D {
    /// Number of items in the X direction.
    pub x: i32,
    /// Number of items in the Y direction.
    pub y: i32,
}

/// Configuration for a seeding plate.
///
/// This struct defines the geometry and layout of a plate used in the seeding process.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Plate {
    /// Human-readable name of the plate.
    pub name: String,
    /// Dimensions of the plate in mm (X, Y, Z).
    pub plate_size: Coord3D,
    /// Coordinates (X, Y) of the center of the first pot.
    pub first_pot: Coord2D,
    /// Spacing between pot centers in X and Y directions.
    pub pot_distance: Coord2D,
    /// Number of pots in the X and Y grid.
    pub nb_pot: IntCoord2D,
}

/// Serial port communication settings.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SerialConfig {
    /// The port identifier (e.g., "/dev/ttys001").
    pub port: String,
    /// The communication speed (e.g., 115200).
    pub baud_rate: u32,
}

/// Robot-specific safety and motion configuration.
///
/// Defines the boundaries within which the robot is allowed to move.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotConfig {
    /// Minimum allowed coordinates (X, Y, Z) in mm.
    pub limit_min: Coord3D,
    /// Maximum allowed coordinates (X, Y, Z) in mm.
    pub limit_max: Coord3D,
}

/// User interface behavior configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UIConfig {
    /// If true, the application will run without window decorations.
    pub kiosk_mode: bool,
}

/// Root configuration structure for the DeltaX2 application.
///
/// This struct is usually deserialized from a `config.toml` file.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Config {
    /// Serial connection parameters.
    pub serial: SerialConfig,
    /// User interface settings.
    pub ui: UIConfig,
    /// Robot safety limits.
    pub robot: RobotConfig,
    /// List of pre-defined seeding plates.
    pub plates: Vec<Plate>,
}

impl Default for RobotConfig {
    /// Provides default safety limits suitable for a basic Delta X 2 setup.
    fn default() -> Self {
        Self {
            limit_min: Coord3D {
                x: -200.0,
                y: -200.0,
                z: -100.0,
            },
            limit_max: Coord3D {
                x: 200.0,
                y: 200.0,
                z: 100.0,
            },
        }
    }
}

impl Default for Plate {
    /// Provides a default plate configuration (e.g., a standard seedling tray).
    fn default() -> Self {
        Self {
            name: "Default Plate".to_string(),
            plate_size: Coord3D {
                x: 500.0,
                y: 700.0,
                z: 40.0,
            },
            first_pot: Coord2D { x: 0.0, y: 0.0 },
            pot_distance: Coord2D { x: 10.0, y: 10.0 },
            nb_pot: IntCoord2D { x: 8, y: 12 },
        }
    }
}

impl Plate {
    /// Checks that the plate geometry is coherent and that every pot
    /// position lies within the robot's software limits.
    ///
    /// Intended to run right after loading `config.toml`, so that a bad
    /// entry surfaces at startup instead of in the middle of a seeding job.
    /// Pot positions decrease from `first_pot` along both axes, so checking
    /// the first and the last pot covers the whole grid.
    ///
    /// # Arguments
    ///
    /// * `limits` - The robot's software limits the pot positions must respect.
    ///
    /// # Errors
    ///
    /// Returns an operator-readable description of the first problem found.
    pub fn validate(&self, limits: &RobotConfig) -> Result<(), String> {
        if self.nb_pot.x < 1 || self.nb_pot.y < 1 {
            return Err(format!(
                "plate '{}': pot grid must be at least 1x1 (got {}x{})",
                self.name, self.nb_pot.x, self.nb_pot.y
            ));
        }
        if self.plate_size.x <= 0.0 || self.plate_size.y <= 0.0 || self.plate_size.z <= 0.0 {
            return Err(format!(
                "plate '{}': plate_size must be positive",
                self.name
            ));
        }
        // A spacing is only meaningful (and required) when there is more
        // than one pot along that axis.
        if (self.nb_pot.x > 1 && self.pot_distance.x <= 0.0)
            || (self.nb_pot.y > 1 && self.pot_distance.y <= 0.0)
        {
            return Err(format!(
                "plate '{}': pot_distance must be positive",
                self.name
            ));
        }

        let last_x = self.first_pot.x - (self.nb_pot.x - 1) as f32 * self.pot_distance.x;
        let last_y = self.first_pot.y - (self.nb_pot.y - 1) as f32 * self.pot_distance.y;
        let checks = [
            (
                "X",
                self.first_pot.x,
                limits.limit_min.x,
                limits.limit_max.x,
            ),
            ("X", last_x, limits.limit_min.x, limits.limit_max.x),
            (
                "Y",
                self.first_pot.y,
                limits.limit_min.y,
                limits.limit_max.y,
            ),
            ("Y", last_y, limits.limit_min.y, limits.limit_max.y),
        ];
        for (axis, value, min, max) in checks {
            if value < min || value > max {
                return Err(format!(
                    "plate '{}': pot position {}={:.1} is outside the software limits [{:.1}, {:.1}]",
                    self.name, axis, value, min, max
                ));
            }
        }
        Ok(())
    }
}

/// Enumerates the physical axes of the robot.
#[derive(Debug, Clone, Copy)]
pub enum Axis {
    /// Horizontal X axis.
    X,
    /// Horizontal Y axis.
    Y,
    /// Vertical Z axis (0 at the homed top position, negative downwards).
    Z,
}

/// Thread-safe controls used to pause or abort a seeding job.
///
/// The UI thread drives these directly (not via the command channel), so a
/// stop request takes effect even while the worker thread is busy executing
/// the seeding loop.
///
/// Abort uses a job-id watermark instead of a resettable flag: the UI
/// assigns every queued job an increasing id, and a stop request records
/// the id it targets. A job aborts when its id is at or below the
/// watermark. This way a stop aimed at a job still waiting in the command
/// queue cannot be erased when the worker dequeues it, and a stop aimed at
/// a finished job cannot leak into the next one.
pub struct SeedingControl {
    /// Highest job id a stop has been requested for (0 = none).
    abort_up_to: AtomicU64,
    /// When set, the seeding loop waits before the next pot until cleared.
    pause: AtomicBool,
}

impl SeedingControl {
    /// Creates a new control block: nothing aborted, not paused.
    pub fn new() -> Self {
        Self {
            abort_up_to: AtomicU64::new(0),
            pause: AtomicBool::new(false),
        }
    }

    /// Requests the abort of `job_id` and every job queued before it.
    ///
    /// The watermark only ever moves forward, so concurrent stop requests
    /// cannot lower it.
    pub fn request_abort(&self, job_id: u64) {
        self.abort_up_to.fetch_max(job_id, Ordering::SeqCst);
    }

    /// Returns `true` if the job with `job_id` should stop.
    pub fn should_abort(&self, job_id: u64) -> bool {
        self.abort_up_to.load(Ordering::SeqCst) >= job_id
    }

    /// Suspends the seeding loop before the next pot.
    pub fn request_pause(&self) {
        self.pause.store(true, Ordering::SeqCst);
    }

    /// Resumes a paused seeding loop.
    pub fn resume(&self) {
        self.pause.store(false, Ordering::SeqCst);
    }

    /// Returns `true` if a pause is currently requested.
    pub fn is_paused(&self) -> bool {
        self.pause.load(Ordering::SeqCst)
    }

    /// Called by the worker when it dequeues a job: clears a pause left
    /// over from a previous job. The abort watermark needs no clearing —
    /// it only affects jobs with ids at or below it.
    pub fn begin_job(&self) {
        self.pause.store(false, Ordering::SeqCst);
    }
}

impl Default for SeedingControl {
    fn default() -> Self {
        Self::new()
    }
}

/// How a seeding job ended (when it did not fail with an error).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SeedOutcome {
    /// Every pot of the plate was processed.
    Completed,
    /// The job was stopped by the user before finishing.
    Aborted,
}

/// Interface for controlling a Delta X 2 robot.
///
/// This struct manages state tracking (where the head is), G-code generation,
/// and safety limit enforcement. It uses `SerialCommunication` to talk to the hardware.
pub struct DeltaRobot {
    /// The serial communication backend.
    serial: SerialCommunication,
    /// Tracked X position in mm.
    actual_x: f32,
    /// Tracked Y position in mm.
    actual_y: f32,
    /// Tracked Z position in mm.
    actual_z: f32,
    /// Tracked rotation/carriage position (if applicable).
    actual_cart: f32,
    /// Lower movement boundaries.
    limit_min: Coord3D,
    /// Upper movement boundaries.
    limit_max: Coord3D,
}

// Note: X2 uses G28 for homing, G90/G91 for modes, and the FEEDBACK parameter for synchronization.

impl Default for DeltaRobot {
    fn default() -> Self {
        Self::new()
    }
}

impl DeltaRobot {
    /// Creates a new `DeltaRobot` instance.
    ///
    /// The initial position is assumed to be at (0, 0, 0) (not homed).
    /// Default safety limits are applied until `set_limits` is called.
    pub fn new() -> Self {
        Self {
            serial: SerialCommunication::new(),
            actual_x: 0.0,
            actual_y: 0.0,
            actual_z: 0.0,
            actual_cart: 0.0,
            limit_min: Coord3D {
                x: -200.0,
                y: -200.0,
                z: -100.0,
            },
            limit_max: Coord3D {
                x: 200.0,
                y: 200.0,
                z: 100.0,
            },
        }
    }

    /// Sets the software safety limits for the robot's movement.
    ///
    /// These limits are checked before any move command is sent to the hardware.
    ///
    /// # Arguments
    ///
    /// * `min` - The minimum permitted coordinates.
    /// * `max` - The maximum permitted coordinates.
    pub fn set_limits(&mut self, min: Coord3D, max: Coord3D) {
        self.limit_min = min;
        self.limit_max = max;
    }

    /// Connects to the robot hardware and verifies its identity.
    ///
    /// It sends the `IsDelta` command and waits for a `YESDELTA` response.
    ///
    /// # Arguments
    ///
    /// * `port` - The name of the serial port.
    /// * `baud_rate` - The baud rate for the connection.
    ///
    /// # Errors
    ///
    /// Returns an error if:
    /// - The serial port cannot be opened.
    /// - The robot does not respond within 2 seconds.
    /// - The robot responds with something other than `YESDELTA`.
    ///
    /// On handshake failure the port is closed again, so the device node is
    /// left free for a later reconnect attempt.
    pub fn connect(&mut self, port: &str, baud_rate: u32) -> Result<()> {
        self.serial.open(port, baud_rate)?;

        match self.handshake() {
            Ok(()) => Ok(()),
            Err(e) => {
                // Do not hold the OS handle on a device that failed the
                // handshake — it may not be the robot at all.
                self.serial.close();
                Err(e.context(format!("Device on {} did not answer IsDelta", port)))
            }
        }
    }

    /// Sends the `IsDelta` identity query and waits for the `YesDelta` answer.
    ///
    /// # Errors
    ///
    /// Returns an error if writing fails or no valid answer arrives within
    /// 2 seconds.
    fn handshake(&mut self) -> Result<()> {
        // Verify it's a Delta Robot by sending an identity query
        self.serial.write_data(b"IsDelta\n")?;

        let mut response = String::new();
        let start = std::time::Instant::now();

        // Poll for response with a 2-second timeout
        while start.elapsed() < std::time::Duration::from_secs(2) {
            if let Ok(data) = self.serial.read_data() {
                response.push_str(&String::from_utf8_lossy(&data));
                // Documentation says it returns 'YesDelta'
                if response.to_uppercase().contains("YESDELTA") {
                    return Ok(());
                }
            }
            // Small sleep to prevent 100% CPU usage during polling
            std::thread::sleep(std::time::Duration::from_millis(50));
        }
        Err(anyhow::anyhow!("no YesDelta response within 2s"))
    }

    /// Disconnects from the robot, closing the serial port.
    ///
    /// The logical position tracking is left untouched; a reconnect should
    /// be followed by homing before trusting any movement (see issue #9).
    pub fn disconnect(&mut self) {
        self.serial.close();
    }

    /// Formats a G0 command string with the specified axis and displacement.
    ///
    /// It appends `FEEDBACK:ok` to help with command execution synchronization.
    ///
    /// # Arguments
    ///
    /// * `axis` - The axis label (e.g., "X").
    /// * `displacement` - The amount to move in mm.
    fn create_mv_command(&self, axis: &str, displacement: f32) -> String {
        format!(
            "G0 {}{:.4} FEEDBACK:ok\n",
            axis.to_uppercase(),
            displacement
        )
    }

    /// Waits for the 'ok' string in the serial stream.
    ///
    /// This is used to ensure the robot has finished executing a command before
    /// sending the next one or updating the UI.
    ///
    /// # Arguments
    ///
    /// * `timeout_secs` - The maximum time to wait in seconds.
    ///
    /// # Errors
    ///
    /// Returns an error if the timeout is reached or if serial communication is lost.
    fn wait_for_ok(&mut self, timeout_secs: u64) -> Result<()> {
        let mut line = String::new();
        let start = std::time::Instant::now();

        while start.elapsed() < std::time::Duration::from_secs(timeout_secs) {
            if let Ok(data) = self.serial.read_data() {
                line.push_str(&String::from_utf8_lossy(&data));
                // Check if the feedback we requested has arrived
                if line.to_uppercase().contains("OK") {
                    return Ok(());
                }
            }
            // Sleep briefly to yield execution
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        Err(anyhow::anyhow!("Timeout waiting for 'ok' from robot"))
    }

    /// Moves the specified axis by a relative displacement.
    ///
    /// This method performs several steps:
    /// 1. Verifies that the move is within safety limits.
    /// 2. Switches the robot to relative positioning mode (`G91`).
    /// 3. Sends the movement command.
    /// 4. Switches the robot back to absolute positioning mode (`G90`).
    ///
    /// Each mode switch and movement is synchronized using the `ok` feedback.
    ///
    /// # Arguments
    ///
    /// * `axis` - The `Axis` to move (X, Y, or Z).
    /// * `displacement` - The relative distance to move in mm.
    ///
    /// # Errors
    ///
    /// Returns an error if:
    /// - The move exceeds safety boundaries.
    /// - Serial communication fails.
    /// - The robot fails to acknowledge any part of the command sequence.
    pub fn move_axis(&mut self, axis: Axis, displacement: f32) -> Result<()> {
        // Determine boundaries and current values based on the selected axis
        let (axis_str, current_val, min, max) = match axis {
            Axis::X => ("X", self.actual_x, self.limit_min.x, self.limit_max.x),
            Axis::Y => ("Y", self.actual_y, self.limit_min.y, self.limit_max.y),
            Axis::Z => ("Z", self.actual_z, self.limit_min.z, self.limit_max.z),
        };

        // Safety check before sending anything to hardware
        if current_val + displacement < min || current_val + displacement > max {
            return Err(anyhow::anyhow!(
                "Movement out of safety limits ({:.2} to {:.2}) for axis {}",
                min,
                max,
                axis_str
            ));
        }

        let cmd = self.create_mv_command(axis_str, displacement);

        // Ensure we are in relative mode for jog-style moves
        self.serial.write_data(b"G91 FEEDBACK:ok\n")?;
        self.wait_for_ok(2)?;

        // Execute the actual move
        self.serial.write_data(cmd.as_bytes())?;
        self.wait_for_ok(5)?;

        // Switch back to absolute (the default state for most G-code apps)
        self.serial.write_data(b"G90 FEEDBACK:ok\n")?;
        self.wait_for_ok(2)?;

        // Update our internal tracking of the robot's position
        match axis {
            Axis::X => self.actual_x += displacement,
            Axis::Y => self.actual_y += displacement,
            Axis::Z => self.actual_z += displacement,
        }

        Ok(())
    }

    /// Updates the internal state for the rotation axis (cart).
    ///
    /// **Note: software-state only, no hardware command is sent.**
    /// Real cart-axis control is tracked in issue #7.
    pub fn move_cart(&mut self, cart: f32) -> Result<()> {
        self.actual_cart += cart;
        Ok(())
    }

    /// Homes the X, Y, and Z axes using the `G28` command.
    ///
    /// Homing moves the robot to its mechanical endstops at the top.
    /// After success, internal coordinates for these axes are reset to 0.0.
    ///
    /// # Errors
    ///
    /// Returns an error if the homing command fails or times out (default 10s).
    pub fn home_xyz(&mut self) -> Result<()> {
        self.serial.write_data(b"G28 FEEDBACK:ok\n")?;
        self.wait_for_ok(10)?;

        // Homing successful, reset logical coordinates
        self.actual_x = 0.0;
        self.actual_y = 0.0;
        self.actual_z = 0.0;
        Ok(())
    }

    /// Resets the internal state for the rotation axis to 0.0.
    ///
    /// **Note: software-state only, no hardware command is sent.**
    /// Real cart-axis control is tracked in issue #7.
    pub fn home_cart(&mut self) -> Result<()> {
        self.actual_cart = 0.0;
        Ok(())
    }

    /// Automates the seeding process for an entire tray.
    ///
    /// It homes the robot first, then iterates through every pot position
    /// defined in the `Plate` structure, calling `seed_pot` for each.
    ///
    /// The loop is cooperative: between pots it honors the pause state and
    /// the abort watermark in `control` (see [`SeedingControl`]), and
    /// reports progress through the `progress` callback. This method blocks
    /// for the duration of the job and is intended to run on the robot
    /// worker thread, with the controls driven from the UI thread.
    ///
    /// # Arguments
    ///
    /// * `plate` - The plate geometry definition.
    /// * `job_id` - The id the UI assigned to this job when queuing it.
    /// * `control` - Shared pause/abort controls checked before each pot.
    /// * `progress` - Called after each pot with (pots done, total pots).
    ///
    /// # Errors
    ///
    /// Returns an error if any step in the process (homing, movement) fails.
    /// A user-requested stop is not an error: it yields `Ok(SeedOutcome::Aborted)`.
    pub fn seed_plate(
        &mut self,
        plate: &Plate,
        job_id: u64,
        control: &SeedingControl,
        mut progress: impl FnMut(i32, i32),
    ) -> Result<SeedOutcome> {
        self.home_cart()?;
        self.home_xyz()?;

        let total = plate.nb_pot.x * plate.nb_pot.y;
        let mut done = 0;

        // Iterate through the grid of pots
        for x in 0..plate.nb_pot.x {
            for y in 0..plate.nb_pot.y {
                // Hold here while paused; a stop request also ends the pause wait.
                while control.is_paused() && !control.should_abort(job_id) {
                    std::thread::sleep(std::time::Duration::from_millis(100));
                }
                if control.should_abort(job_id) {
                    return Ok(SeedOutcome::Aborted);
                }

                // Calculate position relative to the first pot
                let pot_x = plate.first_pot.x - (x as f32 * plate.pot_distance.x);
                let pot_y = plate.first_pot.y - (y as f32 * plate.pot_distance.y);
                self.seed_pot(pot_x, pot_y)?;

                done += 1;
                progress(done, total);
            }
        }
        Ok(SeedOutcome::Completed)
    }

    /// Performs the action required to seed a single pot.
    ///
    /// **Note: empty placeholder, no hardware command is sent.**
    /// The real per-pot sequence (move to (x, y), lower Z, actuate tool,
    /// raise Z) is tracked in issue #7.
    fn seed_pot(&mut self, _x: f32, _y: f32) -> Result<()> {
        // Implementation for seeding a pot (e.g., move to coordinates and toggle tool)
        Ok(())
    }

    /// Retrieves the current logical position of the robot head.
    ///
    /// # Returns
    ///
    /// A tuple containing (X, Y, Z, Cart) coordinates in mm.
    pub fn get_position(&self) -> (f32, f32, f32, f32) {
        (
            self.actual_x,
            self.actual_y,
            self.actual_z,
            self.actual_cart,
        )
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;

    /// Software limits matching the shipped `config.toml`.
    fn test_limits() -> RobotConfig {
        toml::from_str(
            r#"
            limit_min = { x = -160.0, y = -160.0, z = -200.0 }
            limit_max = { x = 160.0, y = 160.0, z = 0.0 }
            "#,
        )
        .unwrap()
    }

    /// The "77pots" plate from the shipped `config.toml`.
    fn test_plate() -> Plate {
        toml::from_str(
            r#"
            name = "77pots"
            plate_size = { x = 500, y = 700, z = 40 }
            first_pot = { x = 7, y = 24 }
            pot_distance = { x = 10, y = 12 }
            nb_pot = { x = 7, y = 11 }
            "#,
        )
        .unwrap()
    }

    #[test]
    fn valid_plate_passes() {
        // Extremes, computed by hand: last pot X = 7 - 6*10 = -53,
        // last pot Y = 24 - 10*12 = -96 — all within [-160, 160].
        assert!(test_plate().validate(&test_limits()).is_ok());
    }

    #[test]
    fn zero_pot_grid_is_rejected() {
        let mut plate = test_plate();
        plate.nb_pot.x = 0;
        assert!(plate.validate(&test_limits()).is_err());
    }

    #[test]
    fn negative_pot_distance_is_rejected() {
        let mut plate = test_plate();
        plate.pot_distance.y = -12.0;
        assert!(plate.validate(&test_limits()).is_err());
    }

    #[test]
    fn single_pot_needs_no_distance() {
        let mut plate = test_plate();
        plate.nb_pot = IntCoord2D { x: 1, y: 1 };
        plate.pot_distance = Coord2D { x: 0.0, y: 0.0 };
        assert!(plate.validate(&test_limits()).is_ok());
    }

    #[test]
    fn first_pot_outside_limits_is_rejected() {
        let mut plate = test_plate();
        plate.first_pot.x = 200.0; // beyond limit_max.x = 160
        assert!(plate.validate(&test_limits()).is_err());
    }

    #[test]
    fn last_pot_outside_limits_is_rejected() {
        let mut plate = test_plate();
        // 7 columns spaced 30 mm: last pot X = 7 - 6*30 = -173 < -160.
        plate.pot_distance.x = 30.0;
        assert!(plate.validate(&test_limits()).is_err());
    }

    #[test]
    fn zero_plate_size_is_rejected() {
        let mut plate = test_plate();
        plate.plate_size.z = 0.0;
        assert!(plate.validate(&test_limits()).is_err());
    }

    #[test]
    fn fresh_control_neither_aborts_nor_pauses() {
        let control = SeedingControl::new();
        assert!(!control.should_abort(1));
        assert!(!control.is_paused());
    }

    #[test]
    fn abort_targets_its_job_and_earlier_ones() {
        let control = SeedingControl::new();
        control.request_abort(2);
        assert!(control.should_abort(1));
        assert!(control.should_abort(2));
        assert!(!control.should_abort(3));
    }

    #[test]
    fn stop_on_queued_job_survives_dequeue() {
        // Regression for the race where a stop pressed while the job was
        // still in the command queue was erased at dequeue time.
        let control = SeedingControl::new();
        control.request_abort(1); // UI: stop pressed before the worker dequeues
        control.begin_job(); // worker: dequeues job 1
        assert!(control.should_abort(1));
    }

    #[test]
    fn abort_watermark_never_moves_backwards() {
        let control = SeedingControl::new();
        control.request_abort(5);
        control.request_abort(3);
        assert!(control.should_abort(5));
    }

    #[test]
    fn begin_job_clears_leftover_pause() {
        let control = SeedingControl::new();
        control.request_pause();
        control.begin_job();
        assert!(!control.is_paused());
    }

    #[test]
    fn pause_and_resume_roundtrip() {
        let control = SeedingControl::new();
        control.request_pause();
        assert!(control.is_paused());
        control.resume();
        assert!(!control.is_paused());
    }
}
