//! DeltaX2 Control Library
//!
//! This library provides the core logic for controlling the Delta X 2 robot via serial communication.
//! It includes modules for serial communication and robot-specific G-code generation and state management.

// A panic on the robot worker thread would silently kill robot control while
// the touch UI keeps running, and missing docs erode the cargo-doc API
// reference; both are enforced by the `-D warnings` quality gate.
#![warn(missing_docs)]
#![warn(clippy::unwrap_used, clippy::expect_used)]

pub mod robot;
pub mod serial;

pub use robot::{Axis, Config, DeltaRobot, Plate, SeedOutcome, SeedingControl};
pub use serial::SerialCommunication;
