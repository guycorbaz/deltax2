//! Serial communication module.
//!
//! This module defines the [`Transport`] trait — the byte-level seam between
//! the robot logic and the outside world — and its production
//! implementation, [`SerialCommunication`], a thin wrapper over the
//! `serialport` crate. Tests provide scripted mock transports instead of
//! opening real ports.

use anyhow::{Result, anyhow};
use serialport::SerialPort;
use std::time::Duration;

/// Byte-level transport used by the robot logic.
///
/// [`SerialCommunication`] is the production implementation; tests implement
/// this trait with a scripted mock so that no real serial port is ever
/// opened in a test.
///
/// Contract for [`Transport::read_data`]: `Ok` with an empty vector means
/// "no data available right now" (a normal poll timeout), while `Err` means
/// the link itself failed (device unplugged, port gone).
pub trait Transport {
    /// Opens the transport towards the device.
    ///
    /// # Arguments
    ///
    /// * `port_name` - The system name of the port (e.g., "/dev/ttyUSB0").
    /// * `baud_rate` - The communication speed in bits per second.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport cannot be opened.
    fn open(&mut self, port_name: &str, baud_rate: u32) -> Result<()>;

    /// Writes a slice of bytes to the device.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport is not open or the write fails.
    fn write_data(&mut self, data: &[u8]) -> Result<()>;

    /// Reads the bytes currently available from the device.
    ///
    /// An empty vector means no data is available right now.
    ///
    /// # Errors
    ///
    /// Returns an error only when the link itself failed — a poll timeout is
    /// NOT an error and yields an empty vector instead.
    fn read_data(&mut self) -> Result<Vec<u8>>;

    /// Discards any pending unread input (stale echoes, leftovers from a
    /// previous command).
    fn flush_input(&mut self);

    /// Closes the transport, releasing any OS handle. Safe to call when
    /// already closed.
    fn close(&mut self);

    /// Returns `true` if the transport is currently open.
    fn is_open(&self) -> bool;
}

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

impl Default for SerialCommunication {
    fn default() -> Self {
        Self::new()
    }
}

impl SerialCommunication {
    /// Creates a new, unconnected `SerialCommunication` instance.
    ///
    /// The internal port is initialized to `None`.
    pub fn new() -> Self {
        Self { port: None }
    }

    /// Lists the available serial ports detected on the system.
    ///
    /// This is useful for populating port selection dropdowns in the UI.
    /// Note: built with `default-features = false` (no libudev), enumeration
    /// is degraded on Linux — the port path normally comes from
    /// `config.toml`.
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

impl Transport for SerialCommunication {
    /// Opens a serial port with the specified name and baud rate.
    ///
    /// This method configures the port with a short 10ms timeout to ensure
    /// responsive reads and writes without blocking the calling thread for
    /// too long.
    ///
    /// # Errors
    ///
    /// Returns an error if the port cannot be found, accessed, or if the
    /// hardware does not support the requested configuration.
    fn open(&mut self, port_name: &str, baud_rate: u32) -> Result<()> {
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
    /// # Errors
    ///
    /// Returns `Err` if the serial port is not currently open or if the
    /// underlying write operation fails.
    fn write_data(&mut self, data: &[u8]) -> Result<()> {
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
    /// This method reads up to 1024 bytes at a time. It is intended to be
    /// called in a polling loop to consume incoming responses from the
    /// robot.
    ///
    /// # Returns
    ///
    /// The bytes actually read; an empty vector when the 10ms poll timeout
    /// elapsed with nothing to read (this is NOT an error).
    ///
    /// # Errors
    ///
    /// Returns an error if the serial port is not open or if the link
    /// failed (e.g., device disconnected).
    fn read_data(&mut self) -> Result<Vec<u8>> {
        if let Some(ref mut port) = self.port {
            let mut buf = vec![0; 1024];
            match port.read(&mut buf) {
                Ok(n) => {
                    // Trim the buffer to the actual number of bytes read
                    buf.truncate(n);
                    Ok(buf)
                }
                // A poll timeout is the normal "nothing yet" case, not a
                // link failure — keep it distinguishable from real errors.
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(Vec::new()),
                Err(e) => Err(e.into()),
            }
        } else {
            Err(anyhow!("Serial port not open"))
        }
    }

    /// Discards any bytes sitting in the OS input buffer.
    fn flush_input(&mut self) {
        if let Some(ref mut port) = self.port {
            let _ = port.clear(serialport::ClearBuffer::Input);
        }
    }

    /// Closes the serial port, releasing the OS handle.
    ///
    /// Safe to call when the port is already closed. After this call the
    /// device node is free again (important before re-opening on reconnect,
    /// or after a failed handshake).
    fn close(&mut self) {
        self.port = None;
    }

    /// Returns `true` if a serial port is currently open.
    ///
    /// Note that this only reflects whether [`Transport::open`] succeeded; a
    /// device that was unplugged afterwards still reports `true` until the
    /// next read/write fails.
    fn is_open(&self) -> bool {
        self.port.is_some()
    }
}
