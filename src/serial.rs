//! Serial communication module.
//!
//! This module provides the `SerialCommunication` struct for handling raw serial port
//! interactions, including opening ports, listing available ports, and reading/writing bytes.
//! It abstracts the underlying `serialport` crate to provide a simpler interface for the robot.

use serialport::SerialPort;
use std::time::Duration;
use anyhow::{Result, anyhow};

/// Handles raw serial communication with the robot hardware.
///
/// This struct maintains the state of the serial connection and provides
/// methods to interact with the device.
pub struct SerialCommunication {
    /// The underlying serial port connection, if open.
    /// It is wrapped in an `Option` and a `Box` to support dynamic dispatch
    /// and allow for a "closed" state.
    port: Option<Box<dyn SerialPort>>,
}

impl SerialCommunication {
    /// Creates a new, unconnected `SerialCommunication` instance.
    ///
    /// The internal port is initialized to `None`.
    pub fn new() -> Self {
        Self { port: None }
    }

    /// Opens a serial port with the specified name and baud rate.
    ///
    /// This method configures the port with a short 10ms timeout to ensure
    /// responsive reads and writes without blocking the main event loop for too long.
    ///
    /// # Arguments
    ///
    /// * `port_name` - The system name of the port (e.g., "/dev/ttyUSB0" on Linux or "COM3" on Windows).
    /// * `baud_rate` - The communication speed in bits per second (e.g., 115200).
    ///
    /// # Errors
    ///
    /// Returns an error if the port cannot be found, accessed, or if the hardware
    /// does not support the requested configuration.
    pub fn open(&mut self, port_name: &str, baud_rate: u32) -> Result<()> {
        // Initialize the builder with basic settings
        let port = serialport::new(port_name, baud_rate)
            .timeout(Duration::from_millis(10)) // Set a short timeout for non-blocking feel
            .open()?;
            
        // Successfully opened, store the port boxed for trait-object compatibility
        self.port = Some(port);
        Ok(())
    }

    /// Writes a slice of bytes to the serial port.
    ///
    /// This is used to send G-code commands to the robot.
    ///
    /// # Arguments
    /// 
    /// * `data` - The byte slice to send over the wire.
    ///
    /// # Errors
    ///
    /// Returns `Err` if the serial port is not currently open or if the 
    /// underlying write operation fails.
    pub fn write_data(&mut self, data: &[u8]) -> Result<()> {
        if let Some(ref mut port) = self.port {
            // Write the entire buffer to the device
            port.write_all(data)?;
            
            // Log for debugging purposes (this appears in console)
            println!("Serial write: {:?}", String::from_utf8_lossy(data));
            Ok(())
        } else {
            Err(anyhow!("Serial port not open"))
        }
    }

    /// Reads available bytes from the serial port.
    ///
    /// This method reads up to 1024 bytes at a time. It is intended to be called
    /// in a loop or polling mechanism to consume incoming responses from the robot.
    ///
    /// # Returns
    ///
    /// A `Result` containing a `Vec<u8>` with the bytes actually read.
    ///
    /// # Errors
    ///
    /// Returns an error if the serial port is not open or if the read operation fails
    /// (e.g., device disconnected).
    pub fn read_data(&mut self) -> Result<Vec<u8>> {
        if let Some(ref mut port) = self.port {
            let mut buf = vec![0; 1024];
            // Perform the read; it might read fewer than 1024 bytes
            let n = port.read(&mut buf)?;
            // Trim the buffer to the actual number of bytes read
            buf.truncate(n);
            Ok(buf)
        } else {
            Err(anyhow!("Serial port not open"))
        }
    }

    /// Lists the available serial ports detected on the system.
    ///
    /// This is useful for populating port selection dropdowns in the UI.
    ///
    /// # Returns
    ///
    /// A vector of strings containing the names of all detected serial ports.
    pub fn list_ports() -> Vec<String> {
        serialport::available_ports()
            .unwrap_or_default()
            .into_iter()
            .map(|p| p.port_name)
            .collect()
    }
}
