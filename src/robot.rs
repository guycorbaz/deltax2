//! Robot control and configuration module.
//!
//! This module defines the high-level `DeltaRobot` interface, coordinate structures,
//! and configuration types required to operate the Delta X 2 robot.
//! It handles G-code generation, safety limit enforcement, and synchronization
//! with the physical hardware.

use crate::serial::{SerialCommunication, Transport};
use anyhow::Result;
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::{Duration, Instant};

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

/// Returns `true` when `buffer` contains a line that is exactly the `ok`
/// acknowledgement (case-insensitive, surrounding whitespace ignored).
///
/// This is deliberately strict: an echoed command such as
/// `G0 X10 FEEDBACK:ok` must NOT count as an acknowledgement — only a line
/// of its own saying `ok` does. The trailing chunk after the last newline is
/// also considered, so a firmware that answers `Ok` without a final newline
/// is still recognized.
fn has_ok_line(buffer: &str) -> bool {
    buffer.lines().any(|l| l.trim().eq_ignore_ascii_case("ok"))
}

/// Parses a firmware `Position` reply into the Cartesian `(x, y, z)` triplet.
///
/// The reply format is `X:<val> Y:<val> Z:<val> W:<val> U:<val>` (see the
/// G-code appendix). Parsing scans each line for `KEY:VALUE` tokens and
/// succeeds only when `X`, `Y` and `Z` are all present and numeric; any other
/// axis (`W`, `U`) is ignored, since only the Cartesian axes are resynced
/// (issue #9). Returns `None` for a line that lacks a full X/Y/Z triplet, so a
/// partial or unrelated line is never mistaken for a position.
fn parse_position(text: &str) -> Option<(f32, f32, f32)> {
    for line in text.lines() {
        let (mut x, mut y, mut z) = (None, None, None);
        for token in line.split_whitespace() {
            if let Some((key, value)) = token.split_once(':') {
                let Ok(parsed) = value.parse::<f32>() else {
                    continue;
                };
                match key.to_ascii_uppercase().as_str() {
                    "X" => x = Some(parsed),
                    "Y" => y = Some(parsed),
                    "Z" => z = Some(parsed),
                    _ => {}
                }
            }
        }
        if let (Some(x), Some(y), Some(z)) = (x, y, z) {
            return Some((x, y, z));
        }
    }
    None
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

/// Time-source seam for the polling/timeout loops, so their behavior can be
/// tested deterministically without real `sleep`s (issue #19).
///
/// Production uses [`SystemClock`] (the monotonic wall clock + `thread::sleep`);
/// tests inject a virtual clock whose time advances only when `sleep` is called,
/// so a "no `ok` ever arrives" timeout completes instantly.
pub trait Clock {
    /// Monotonic time elapsed since this clock was created.
    fn elapsed(&self) -> Duration;
    /// Between polls: really sleeps (production) or virtually advances time
    /// (tests) by `dur`.
    fn sleep(&self, dur: Duration);
}

/// Production [`Clock`] backed by [`Instant`] and `std::thread::sleep`.
pub struct SystemClock {
    start: Instant,
}

impl SystemClock {
    /// Creates a clock whose origin is now.
    pub fn new() -> Self {
        Self {
            start: Instant::now(),
        }
    }
}

impl Default for SystemClock {
    fn default() -> Self {
        Self::new()
    }
}

impl Clock for SystemClock {
    fn elapsed(&self) -> Duration {
        self.start.elapsed()
    }
    fn sleep(&self, dur: Duration) {
        std::thread::sleep(dur);
    }
}

/// Interface for controlling a Delta X 2 robot.
///
/// This struct manages state tracking (where the head is), G-code generation,
/// and safety limit enforcement. It talks to the hardware through any
/// [`Transport`] implementation — [`SerialCommunication`] in production, a
/// scripted mock in tests — and reads time through a [`Clock`] seam.
pub struct DeltaRobot<T: Transport = SerialCommunication, C: Clock = SystemClock> {
    /// The byte-level transport towards the robot.
    transport: T,
    /// Time source for the polling/timeout loops.
    clock: C,
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
    /// True when a failed command sequence left the tracked position or the
    /// firmware positioning mode unreliable. While set, every move is
    /// refused; a successful homing (`G28`) clears it.
    desynchronized: bool,
}

// Note: X2 uses G28 for homing, G90/G91 for modes, and the FEEDBACK parameter for synchronization.

impl Default for DeltaRobot {
    fn default() -> Self {
        Self::new()
    }
}

impl DeltaRobot {
    /// Creates a new `DeltaRobot` instance backed by a real serial port.
    ///
    /// The initial position is assumed to be at (0, 0, 0) (not homed).
    /// Default safety limits are applied until `set_limits` is called.
    pub fn new() -> Self {
        Self::with_transport(SerialCommunication::new())
    }
}

impl<T: Transport> DeltaRobot<T, SystemClock> {
    /// Creates a `DeltaRobot` driving the given transport with the real system
    /// clock.
    ///
    /// Production code uses [`DeltaRobot::new`]; tests inject a scripted
    /// mock here so no real serial port is ever opened. Tests that exercise
    /// timeout behavior use [`DeltaRobot::with_transport_and_clock`] to also
    /// inject a virtual clock.
    pub fn with_transport(transport: T) -> Self {
        Self::with_transport_and_clock(transport, SystemClock::new())
    }
}

impl<T: Transport, C: Clock> DeltaRobot<T, C> {
    /// Creates a `DeltaRobot` from a transport and a [`Clock`].
    pub fn with_transport_and_clock(transport: T, clock: C) -> Self {
        Self {
            transport,
            clock,
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
            desynchronized: false,
        }
    }

    /// Returns `true` when a failed command sequence left the tracked
    /// position or the firmware positioning mode unreliable.
    ///
    /// While desynchronized, every move is refused; a successful homing
    /// ([`Self::home_xyz`]) clears the state.
    pub fn is_desynchronized(&self) -> bool {
        self.desynchronized
    }

    /// Gives tests access to the underlying (mock) transport.
    #[cfg(test)]
    fn transport_ref(&self) -> &T {
        &self.transport
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
    /// It sends the `IsDelta` command and waits for a `YESDELTA` response, then
    /// resyncs the tracked position from the firmware (issue #9) so a robot
    /// moved by hand while disconnected is not driven against a stale position.
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
        self.transport.open(port, baud_rate)?;

        match self.handshake() {
            Ok(()) => {
                // The robot may have been moved by hand while disconnected, so
                // the dead-reckoned position cannot be trusted. Read the real
                // one back from the firmware (issue #9). If that fails, leave
                // the state desynchronized so a homing is required before any
                // jog — never let the software limits guard a stale position.
                if self.query_position().is_err() {
                    self.desynchronized = true;
                }
                Ok(())
            }
            Err(e) => {
                // Do not hold the OS handle on a device that failed the
                // handshake — it may not be the robot at all.
                self.transport.close();
                Err(e.context(format!("Device on {} did not answer IsDelta", port)))
            }
        }
    }

    /// Reads the head position back from the firmware with the `Position`
    /// command and adopts it as the tracked X/Y/Z, clearing the
    /// desynchronized state on success (issue #9).
    ///
    /// Only the Cartesian axes are resynced; they are the ones the software
    /// limits guard. The rotation ("cart") axis stays software-only until real
    /// cart control lands (issue #7), so any `W`/`U` fields are ignored.
    ///
    /// # Errors
    ///
    /// Returns an error if the write fails, the link is lost (the port is then
    /// closed), or no parseable `X: Y: Z:` line arrives within 2 seconds.
    fn query_position(&mut self) -> Result<()> {
        // A stale line from before must not be read as the current position.
        self.transport.flush_input();
        self.transport.write_data(b"Position\n")?;

        let mut buffer = String::new();
        let start = self.clock.elapsed();
        while self.clock.elapsed() - start < Duration::from_secs(2) {
            match self.transport.read_data() {
                Ok(data) => {
                    if !data.is_empty() {
                        buffer.push_str(&String::from_utf8_lossy(&data));
                        // Parse only the portion up to the last newline, so a
                        // partial trailing read cannot be mistaken for a
                        // complete coordinate line.
                        let complete = buffer.rfind('\n').map_or("", |i| &buffer[..i]);
                        if let Some((x, y, z)) = parse_position(complete) {
                            self.actual_x = x;
                            self.actual_y = y;
                            self.actual_z = z;
                            self.desynchronized = false;
                            return Ok(());
                        }
                    }
                }
                Err(e) => {
                    self.transport.close();
                    self.desynchronized = true;
                    return Err(e.context("serial link lost while reading position"));
                }
            }
            self.clock.sleep(Duration::from_millis(10));
        }
        Err(anyhow::anyhow!("no position response within 2s"))
    }

    /// Sends the `IsDelta` identity query and waits for the `YesDelta` answer.
    ///
    /// # Errors
    ///
    /// Returns an error if writing fails or no valid answer arrives within
    /// 2 seconds.
    fn handshake(&mut self) -> Result<()> {
        // Stale bytes from a previous session must not satisfy the handshake.
        self.transport.flush_input();
        // Verify it's a Delta Robot by sending an identity query
        self.transport.write_data(b"IsDelta\n")?;

        let mut response = String::new();
        let start = self.clock.elapsed();

        // Poll for response with a 2-second timeout
        while self.clock.elapsed() - start < Duration::from_secs(2) {
            match self.transport.read_data() {
                Ok(data) => {
                    response.push_str(&String::from_utf8_lossy(&data));
                    // Documentation says it returns 'YesDelta'
                    if response.to_uppercase().contains("YESDELTA") {
                        return Ok(());
                    }
                }
                // A read error means the link itself failed — no point
                // polling until the timeout.
                Err(e) => return Err(e.context("serial link lost during handshake")),
            }
            // Small sleep to prevent 100% CPU usage during polling
            self.clock.sleep(Duration::from_millis(50));
        }
        Err(anyhow::anyhow!("no YesDelta response within 2s"))
    }

    /// Disconnects from the robot, closing the serial port.
    ///
    /// The logical position tracking is left untouched; the next
    /// [`connect`](Self::connect) resyncs it from the firmware (or, failing
    /// that, requires homing before the next move) — see issue #9.
    pub fn disconnect(&mut self) {
        self.transport.close();
    }

    /// Returns `true` while the transport towards the robot is open.
    ///
    /// This reflects the last known link state; a lost link is only
    /// detected when a command fails on it.
    pub fn is_connected(&self) -> bool {
        self.transport.is_open()
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

    /// Waits for an `ok` acknowledgement line in the serial stream.
    ///
    /// This is used to ensure the robot has finished executing a command before
    /// sending the next one or updating the UI. Matching is line-exact (see
    /// [`has_ok_line`]) so an echoed `FEEDBACK:ok` suffix cannot satisfy the
    /// wait for the wrong command.
    ///
    /// # Arguments
    ///
    /// * `timeout_secs` - The maximum time to wait in seconds.
    ///
    /// # Errors
    ///
    /// Returns an error if the timeout is reached or if the serial link is
    /// lost. A lost link (a read *error*, as opposed to an empty poll)
    /// closes the port — so the connection state is truthful — and marks
    /// the position as unknown.
    fn wait_for_ok(&mut self, timeout_secs: u64) -> Result<()> {
        let mut buffer = String::new();
        let start = self.clock.elapsed();

        while self.clock.elapsed() - start < Duration::from_secs(timeout_secs) {
            match self.transport.read_data() {
                Ok(data) => {
                    if !data.is_empty() {
                        buffer.push_str(&String::from_utf8_lossy(&data));
                        // Check if the acknowledgement we requested has arrived
                        if has_ok_line(&buffer) {
                            return Ok(());
                        }
                    }
                }
                Err(e) => {
                    self.transport.close();
                    self.desynchronized = true;
                    return Err(e.context("serial link lost while waiting for 'ok'"));
                }
            }
            // Sleep briefly to yield execution
            self.clock.sleep(Duration::from_millis(10));
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
    /// - The robot state is desynchronized (homing required first).
    /// - The move exceeds safety boundaries.
    /// - Serial communication fails.
    /// - The robot fails to acknowledge any part of the command sequence.
    ///
    /// If the sequence fails after the first command was sent, the firmware
    /// may be stuck in relative mode and the head may or may not have moved:
    /// a best-effort `G90` restore is attempted, the state is marked
    /// desynchronized, and homing is required before the next move.
    pub fn move_axis(&mut self, axis: Axis, displacement: f32) -> Result<()> {
        if self.desynchronized {
            return Err(anyhow::anyhow!(
                "Robot state is desynchronized after a failed command — homing (G28) required before moving"
            ));
        }

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

        // Leftover input (stale echoes, late acknowledgements) must not be
        // mistaken for this sequence's acknowledgements.
        self.transport.flush_input();

        let sequence = (|| -> Result<()> {
            // Ensure we are in relative mode for jog-style moves
            self.transport.write_data(b"G91 FEEDBACK:ok\n")?;
            self.wait_for_ok(2)?;

            // Execute the actual move
            self.transport.write_data(cmd.as_bytes())?;
            self.wait_for_ok(5)?;

            // Switch back to absolute (the default state for most G-code apps)
            self.transport.write_data(b"G90 FEEDBACK:ok\n")?;
            self.wait_for_ok(2)?;
            Ok(())
        })();

        match sequence {
            Ok(()) => {
                // Update our internal tracking of the robot's position
                match axis {
                    Axis::X => self.actual_x += displacement,
                    Axis::Y => self.actual_y += displacement,
                    Axis::Z => self.actual_z += displacement,
                }
                Ok(())
            }
            Err(e) => {
                // The sequence stopped partway: the firmware may be stuck in
                // relative mode and the head may or may not have moved, so
                // both the mode and the tracked position are unreliable.
                // Best-effort attempt to restore absolute mode, then require
                // homing before any further motion.
                self.desynchronized = true;
                let _ = self.transport.write_data(b"G90 FEEDBACK:ok\n");
                let _ = self.wait_for_ok(2);
                Err(e.context(
                    "move failed mid-sequence; state desynchronized, homing (G28) required",
                ))
            }
        }
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
        // Do not let stale input acknowledge the homing command.
        self.transport.flush_input();
        self.transport.write_data(b"G28 FEEDBACK:ok\n")?;
        if let Err(e) = self.wait_for_ok(10) {
            // An unacknowledged homing leaves the position unknown.
            self.desynchronized = true;
            return Err(e.context("homing failed; state desynchronized"));
        }

        // Homing successful: the head sits at the mechanical origin, so the
        // logical coordinates are trustworthy again.
        self.actual_x = 0.0;
        self.actual_y = 0.0;
        self.actual_z = 0.0;
        self.desynchronized = false;
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
    /// A pause requested from the UI only takes effect once the current pot
    /// finishes and the loop reaches the wait below. `on_pause` reports that
    /// transition — `true` when the loop actually parks on a pause, `false`
    /// when it leaves the wait (resumed or aborted) — so the UI can show a
    /// "pausing…" pending state until the pause is genuinely in effect,
    /// following the async-feedback rule.
    ///
    /// # Arguments
    ///
    /// * `plate` - The plate geometry definition.
    /// * `job_id` - The id the UI assigned to this job when queuing it.
    /// * `control` - Shared pause/abort controls checked before each pot.
    /// * `progress` - Called after each pot with (pots done, total pots).
    /// * `on_pause` - Called with `true` when the loop parks on a pause and
    ///   `false` when it leaves that wait, so the UI reflects the real state.
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
        mut on_pause: impl FnMut(bool),
    ) -> Result<SeedOutcome> {
        self.home_cart()?;
        self.home_xyz()?;

        let total = plate.nb_pot.x * plate.nb_pot.y;
        let mut done = 0;

        // Iterate through the grid of pots
        for x in 0..plate.nb_pot.x {
            for y in 0..plate.nb_pot.y {
                // Hold here while paused; a stop request also ends the pause
                // wait. Announce the paused state exactly once on entry and
                // clear it once on exit, so the UI sees the real transition
                // (not just the button tap) even though the request arrived
                // mid-pot.
                let mut announced_pause = false;
                while control.is_paused() && !control.should_abort(job_id) {
                    if !announced_pause {
                        on_pause(true);
                        announced_pause = true;
                    }
                    std::thread::sleep(std::time::Duration::from_millis(100));
                }
                if announced_pause {
                    on_pause(false);
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
    use std::collections::VecDeque;

    /// Scripted [`Transport`] for tests: each successful write releases the
    /// next scripted response chunk, which the following reads return. No
    /// real port is ever opened.
    struct MockTransport {
        /// Response chunks not yet released (one per future write).
        script: VecDeque<Vec<u8>>,
        /// Released chunks waiting to be read.
        pending: VecDeque<Vec<u8>>,
        /// Everything successfully written, concatenated.
        written: Vec<u8>,
        /// 1-based index of a write that must fail, if any.
        fail_write_at: Option<usize>,
        /// When set, every read fails as a lost link.
        fail_reads: bool,
        writes: usize,
        open: bool,
    }

    impl MockTransport {
        /// A transport that answers the nth write with the nth response.
        fn scripted(responses: &[&str]) -> Self {
            Self {
                script: responses.iter().map(|r| r.as_bytes().to_vec()).collect(),
                pending: VecDeque::new(),
                written: Vec::new(),
                fail_write_at: None,
                fail_reads: false,
                writes: 0,
                open: true,
            }
        }

        fn written_str(&self) -> String {
            String::from_utf8(self.written.clone()).unwrap()
        }
    }

    impl Transport for MockTransport {
        fn open(&mut self, _port_name: &str, _baud_rate: u32) -> Result<()> {
            self.open = true;
            Ok(())
        }

        fn write_data(&mut self, data: &[u8]) -> Result<()> {
            self.writes += 1;
            if self.fail_write_at == Some(self.writes) {
                return Err(anyhow::anyhow!("mock write failure"));
            }
            if !self.open {
                return Err(anyhow::anyhow!("mock transport not open"));
            }
            self.written.extend_from_slice(data);
            if let Some(response) = self.script.pop_front() {
                self.pending.push_back(response);
            }
            Ok(())
        }

        fn read_data(&mut self) -> Result<Vec<u8>> {
            if self.fail_reads {
                return Err(anyhow::anyhow!("mock link lost"));
            }
            if !self.open {
                return Err(anyhow::anyhow!("mock transport not open"));
            }
            Ok(self.pending.pop_front().unwrap_or_default())
        }

        fn flush_input(&mut self) {
            self.pending.clear();
        }

        fn close(&mut self) {
            self.open = false;
        }

        fn is_open(&self) -> bool {
            self.open
        }
    }

    /// Virtual [`Clock`] for timeout tests: time advances only when `sleep` is
    /// called, so a "no response ever arrives" loop reaches its timeout
    /// instantly and without any real sleeping. The shared counter lets a test
    /// read how far virtual time advanced.
    #[derive(Clone)]
    struct MockClock {
        now: std::rc::Rc<std::cell::Cell<Duration>>,
    }

    impl MockClock {
        fn new() -> Self {
            Self {
                now: std::rc::Rc::new(std::cell::Cell::new(Duration::ZERO)),
            }
        }
        fn virtual_elapsed(&self) -> Duration {
            self.now.get()
        }
    }

    impl Clock for MockClock {
        fn elapsed(&self) -> Duration {
            self.now.get()
        }
        fn sleep(&self, dur: Duration) {
            self.now.set(self.now.get() + dur);
        }
    }

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

    /// Property (issue #21): for *any* plate that `validate` accepts, every pot
    /// the seeding loop will visit must lie within the software limits — i.e.
    /// checking only the first and last pot (as `validate` does) genuinely
    /// covers the whole grid. This sweeps a wide grid of geometries by hand (no
    /// `proptest` dependency) and, using the same pot formula as `seed_plate`,
    /// asserts the safety invariant on every accepted plate. It also asserts
    /// the sweep exercises both accepted and rejected plates, so the property
    /// is not vacuous.
    #[test]
    fn every_pot_of_a_validated_plate_is_within_limits() {
        let limits = test_limits();
        let nb_values = [1i32, 2, 8, 15];
        let dist_values = [0.0f32, 1.0, 12.0, 40.0];
        let first_values = [-160.0f32, -70.0, 0.0, 24.0, 70.0, 160.0];

        let mut accepted = 0u32;
        let mut rejected = 0u32;

        for &nx in &nb_values {
            for &ny in &nb_values {
                for &dx in &dist_values {
                    for &dy in &dist_values {
                        for &fx in &first_values {
                            for &fy in &first_values {
                                let mut plate = test_plate();
                                plate.nb_pot = IntCoord2D { x: nx, y: ny };
                                plate.pot_distance = Coord2D { x: dx, y: dy };
                                plate.first_pot = Coord2D { x: fx, y: fy };

                                if plate.validate(&limits).is_err() {
                                    rejected += 1;
                                    continue;
                                }
                                accepted += 1;

                                // Same formula the seeding loop uses.
                                for i in 0..plate.nb_pot.x {
                                    let px = plate.first_pot.x - i as f32 * plate.pot_distance.x;
                                    assert!(
                                        px >= limits.limit_min.x && px <= limits.limit_max.x,
                                        "accepted plate has pot X={px} outside [{}, {}] (nx={nx} dx={dx} fx={fx})",
                                        limits.limit_min.x,
                                        limits.limit_max.x
                                    );
                                }
                                for j in 0..plate.nb_pot.y {
                                    let py = plate.first_pot.y - j as f32 * plate.pot_distance.y;
                                    assert!(
                                        py >= limits.limit_min.y && py <= limits.limit_max.y,
                                        "accepted plate has pot Y={py} outside [{}, {}] (ny={ny} dy={dy} fy={fy})",
                                        limits.limit_min.y,
                                        limits.limit_max.y
                                    );
                                }
                            }
                        }
                    }
                }
            }
        }

        // The sweep must hit both sides, or the invariant above is vacuous.
        assert!(accepted > 0, "no plate was accepted — sweep too narrow");
        assert!(rejected > 0, "no plate was rejected — sweep too narrow");
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

    /// A single-pot plate: seeds one pot with a single G28 on the wire and no
    /// per-pot hardware traffic (seed_pot is still a placeholder).
    fn single_pot_plate() -> Plate {
        let mut plate = test_plate();
        plate.nb_pot = IntCoord2D { x: 1, y: 1 };
        plate
    }

    #[test]
    fn seed_plate_completes_without_pause_callbacks_when_not_paused() {
        let control = SeedingControl::new();
        let mut robot = DeltaRobot::with_transport(MockTransport::scripted(&["ok\n"]));
        let mut pause_events: Vec<bool> = Vec::new();
        let mut last_done = 0;
        let outcome = robot
            .seed_plate(
                &single_pot_plate(),
                1,
                &control,
                |done, _total| last_done = done,
                |p| pause_events.push(p),
            )
            .unwrap();
        assert_eq!(outcome, SeedOutcome::Completed);
        assert_eq!(last_done, 1);
        // Never paused, so the UI is never told about a pause transition.
        assert!(pause_events.is_empty());
    }

    #[test]
    fn seed_plate_stop_before_first_pot_aborts_without_pause_callback() {
        let control = SeedingControl::new();
        control.request_abort(1); // stop pressed before the pot loop reaches a pause
        let mut robot = DeltaRobot::with_transport(MockTransport::scripted(&["ok\n"]));
        let mut pause_events: Vec<bool> = Vec::new();
        let outcome = robot
            .seed_plate(
                &single_pot_plate(),
                1,
                &control,
                |_done, _total| {},
                |p| pause_events.push(p),
            )
            .unwrap();
        assert_eq!(outcome, SeedOutcome::Aborted);
        assert!(pause_events.is_empty());
    }

    #[test]
    fn seed_plate_reports_pause_then_resume_transition() {
        use std::sync::{Arc, Mutex};
        use std::time::Duration;

        // Pause is requested up front, so the loop parks on the very first pot;
        // a helper thread resumes it once it has parked.
        let control = Arc::new(SeedingControl::new());
        control.request_pause();
        let resumer = {
            let c = control.clone();
            std::thread::spawn(move || {
                std::thread::sleep(Duration::from_millis(50));
                c.resume();
            })
        };

        let events = Arc::new(Mutex::new(Vec::<bool>::new()));
        let ev = events.clone();
        let mut robot = DeltaRobot::with_transport(MockTransport::scripted(&["ok\n"]));
        let outcome = robot
            .seed_plate(
                &single_pot_plate(),
                1,
                &control,
                |_done, _total| {},
                move |p| ev.lock().unwrap().push(p),
            )
            .unwrap();
        resumer.join().unwrap();

        assert_eq!(outcome, SeedOutcome::Completed);
        // The worker confirmed the pause (true), then the resume (false):
        // exactly the transition the UI reflects.
        assert_eq!(*events.lock().unwrap(), vec![true, false]);
    }

    #[test]
    fn ok_matching_is_line_exact() {
        assert!(has_ok_line("ok\n"));
        assert!(has_ok_line("Ok\r\n"));
        assert!(has_ok_line("some report line\nok\n"));
        assert!(has_ok_line("ok")); // firmware without a trailing newline
        assert!(!has_ok_line("G0 X10.0000 FEEDBACK:ok\n")); // echoed command
        assert!(!has_ok_line("FEEDBACK:ok\n"));
        assert!(!has_ok_line("okay\n"));
        assert!(!has_ok_line(""));
    }

    // --- Wire-contract characterization (through the mock transport) ---

    #[test]
    fn connect_performs_isdelta_handshake_then_reads_position() {
        // Handshake answer, then the Position reply resynced on connect (#9).
        let mut robot = DeltaRobot::with_transport(MockTransport::scripted(&[
            "YesDelta\n",
            "X:10.5 Y:-20.0 Z:-150.0 W:0.0 U:0.0\n",
        ]));
        robot.connect("/dev/mock", 115_200).unwrap();
        // The wire contract: identity query, then position query.
        assert_eq!(robot.transport_ref().written_str(), "IsDelta\nPosition\n");
        assert!(robot.is_connected());
        // The tracked position now matches the firmware, not the (0,0,0) default.
        let (x, y, z, _) = robot.get_position();
        assert_eq!((x, y, z), (10.5, -20.0, -150.0));
        // A known position means moves are allowed without a fresh homing.
        assert!(!robot.is_desynchronized());
    }

    #[test]
    fn parse_position_reads_xyz_and_ignores_other_axes() {
        assert_eq!(
            parse_position("X:10.5 Y:-20.0 Z:-150.0 W:1.0 U:2.0\n"),
            Some((10.5, -20.0, -150.0))
        );
        // Cart/other axes and surrounding noise do not matter.
        assert_eq!(parse_position("  X:0 Y:0 Z:0  \n"), Some((0.0, 0.0, 0.0)));
    }

    #[test]
    fn parse_position_rejects_incomplete_or_unrelated_lines() {
        assert_eq!(parse_position("X:10.0 Y:5.0\n"), None); // no Z
        assert_eq!(parse_position("ok\n"), None);
        assert_eq!(parse_position(""), None);
        assert_eq!(parse_position("X:foo Y:1 Z:2\n"), None); // X not numeric
    }

    #[test]
    fn parse_position_takes_the_first_complete_triplet() {
        // A leading report line is skipped in favor of the coordinate line.
        assert_eq!(
            parse_position("Homing done\nX:1.0 Y:2.0 Z:3.0\n"),
            Some((1.0, 2.0, 3.0))
        );
    }

    // --- Timeout behavior, driven by the injected virtual clock (issue #19) ---

    #[test]
    fn wait_for_ok_times_out_deterministically() {
        // The transport never yields an `ok`; the virtual clock advances via
        // the poll sleeps, so the 5s timeout is reached with no real sleeping.
        let clock = MockClock::new();
        let handle = clock.clone();
        let mut robot = DeltaRobot::with_transport_and_clock(MockTransport::scripted(&[]), clock);
        let wall_start = Instant::now();
        assert!(robot.wait_for_ok(5).is_err());
        assert!(handle.virtual_elapsed() >= Duration::from_secs(5));
        // The whole "5 second" wait happened in a fraction of a real second.
        assert!(wall_start.elapsed() < Duration::from_secs(1));
    }

    #[test]
    fn wait_for_ok_timeout_scales_with_the_command_class() {
        // A 2s class stops at ~2s of virtual time, a 10s class at ~10s — the
        // per-command timeout is honored, not a fixed value.
        for secs in [2u64, 10] {
            let clock = MockClock::new();
            let handle = clock.clone();
            let mut robot =
                DeltaRobot::with_transport_and_clock(MockTransport::scripted(&[]), clock);
            assert!(robot.wait_for_ok(secs).is_err());
            assert!(handle.virtual_elapsed() >= Duration::from_secs(secs));
            // One poll interval (10ms) of overshoot at most.
            assert!(
                handle.virtual_elapsed() < Duration::from_secs(secs) + Duration::from_millis(20)
            );
        }
    }

    #[test]
    fn wait_for_ok_returns_before_timeout_when_ok_arrives() {
        // The ok is already waiting, so the first poll returns Ok without the
        // virtual clock ever reaching the timeout.
        let clock = MockClock::new();
        let handle = clock.clone();
        let mut mock = MockTransport::scripted(&[]);
        mock.pending.push_back(b"ok\n".to_vec());
        let mut robot = DeltaRobot::with_transport_and_clock(mock, clock);
        assert!(robot.wait_for_ok(5).is_ok());
        assert!(handle.virtual_elapsed() < Duration::from_secs(5));
    }

    #[test]
    fn handshake_times_out_without_yesdelta() {
        // No YesDelta ever comes back: the 2s handshake timeout fires instantly
        // in virtual time.
        let clock = MockClock::new();
        let handle = clock.clone();
        let mut robot = DeltaRobot::with_transport_and_clock(MockTransport::scripted(&[]), clock);
        let wall_start = Instant::now();
        assert!(robot.handshake().is_err());
        assert!(handle.virtual_elapsed() >= Duration::from_secs(2));
        assert!(wall_start.elapsed() < Duration::from_secs(1));
    }

    #[test]
    fn jog_sends_g91_move_g90_wire_sequence() {
        let mut robot =
            DeltaRobot::with_transport(MockTransport::scripted(&["ok\n", "ok\n", "ok\n"]));
        robot.move_axis(Axis::X, 10.0).unwrap();
        // The wire contract with the firmware, byte for byte.
        assert_eq!(
            robot.transport_ref().written_str(),
            "G91 FEEDBACK:ok\nG0 X10.0000 FEEDBACK:ok\nG90 FEEDBACK:ok\n"
        );
        let (x, y, z, _) = robot.get_position();
        assert!((x - 10.0).abs() < 0.01, "got {x}, want 10.0");
        assert!(y.abs() < 0.01 && z.abs() < 0.01);
    }

    #[test]
    fn out_of_limits_jog_sends_nothing() {
        let mut robot = DeltaRobot::with_transport(MockTransport::scripted(&[]));
        // Default limits are ±200 mm; 250 must be rejected before any I/O.
        assert!(robot.move_axis(Axis::X, 250.0).is_err());
        assert_eq!(robot.transport_ref().written_str(), "");
        let (x, _, _, _) = robot.get_position();
        assert!(x.abs() < 0.01);
    }

    #[test]
    fn stale_input_is_flushed_before_a_command_sequence() {
        // A leftover acknowledgement from a previous command sits in the
        // input buffer; it must not shift this sequence's acknowledgements.
        let mut mock = MockTransport::scripted(&["ok\n", "ok\n", "ok\n"]);
        mock.pending.push_back(b"ok\n".to_vec());
        let mut robot = DeltaRobot::with_transport(mock);
        robot.move_axis(Axis::X, 5.0).unwrap();
        assert_eq!(
            robot.transport_ref().written_str(),
            "G91 FEEDBACK:ok\nG0 X5.0000 FEEDBACK:ok\nG90 FEEDBACK:ok\n"
        );
        let (x, _, _, _) = robot.get_position();
        assert!((x - 5.0).abs() < 0.01, "got {x}, want 5.0");
    }

    #[test]
    fn homing_sends_g28_and_resets_position() {
        let mut robot =
            DeltaRobot::with_transport(MockTransport::scripted(&["ok\n", "ok\n", "ok\n", "ok\n"]));
        robot.move_axis(Axis::Z, -50.0).unwrap();
        robot.home_xyz().unwrap();
        assert!(
            robot
                .transport_ref()
                .written_str()
                .ends_with("G28 FEEDBACK:ok\n")
        );
        let (x, y, z, _) = robot.get_position();
        assert!(x.abs() < 0.01 && y.abs() < 0.01 && z.abs() < 0.01);
    }

    #[test]
    fn failed_mid_sequence_desynchronizes_until_homed() {
        let mut mock = MockTransport::scripted(&["ok\n", "ok\n", "ok\n", "ok\n", "ok\n", "ok\n"]);
        mock.fail_write_at = Some(2); // the G0 write fails, after G91 succeeded
        let mut robot = DeltaRobot::with_transport(mock);

        // The move fails and the state is marked desynchronized; the
        // tracked position must not have changed.
        assert!(robot.move_axis(Axis::X, 10.0).is_err());
        assert!(robot.is_desynchronized());
        let (x, _, _, _) = robot.get_position();
        assert!(x.abs() < 0.01);
        // A best-effort G90 restore was attempted after the failure.
        assert!(
            robot
                .transport_ref()
                .written_str()
                .ends_with("G90 FEEDBACK:ok\n")
        );

        // Further moves are refused without any I/O until homing.
        let writes_before = robot.transport_ref().writes;
        assert!(robot.move_axis(Axis::X, 1.0).is_err());
        assert_eq!(robot.transport_ref().writes, writes_before);

        // A successful homing recovers the state and moves work again.
        robot.home_xyz().unwrap();
        assert!(!robot.is_desynchronized());
        robot.move_axis(Axis::X, 1.0).unwrap();
        let (x, _, _, _) = robot.get_position();
        assert!((x - 1.0).abs() < 0.01, "got {x}, want 1.0");
    }

    #[test]
    fn lost_link_closes_the_port_and_desynchronizes() {
        // A read *error* (as opposed to an empty poll) is a lost link: the
        // command must fail fast, the port must be closed so the UI
        // connection state is truthful, and the position becomes unknown.
        let mut mock = MockTransport::scripted(&[]);
        mock.fail_reads = true;
        let mut robot = DeltaRobot::with_transport(mock);

        assert!(robot.move_axis(Axis::X, 1.0).is_err());
        assert!(!robot.is_connected());
        assert!(robot.is_desynchronized());
    }
}
