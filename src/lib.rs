//! DeltaX2 Control Library
//!
//! This library provides the core logic for controlling the Delta X 2 robot via serial communication.
//! It includes modules for serial communication and robot-specific G-code generation and state management.

pub mod serial;
pub mod robot;

pub use robot::{DeltaRobot, Axis, Plate, Config};
pub use serial::SerialCommunication;
