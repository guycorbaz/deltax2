# DeltaX2 Control Application

[![Rust](https://img.shields.io/badge/Rust-edition%202024-orange?logo=rust)](https://www.rust-lang.org/)
[![Slint](https://img.shields.io/badge/UI-Slint-2c4bd0)](https://slint.dev/)
[![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-3%20%7C%204%20%7C%205-c51a4a?logo=raspberrypi&logoColor=white)](https://www.raspberrypi.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

**Project website: [guycorbaz.github.io/deltax2](https://guycorbaz.github.io/deltax2/)**

A modern, high-performance control application for the [**Delta X 2**](https://docs.deltaxrobot.com/products/deltax2/deltax2_basic_kit/) delta-arm seeding robot, built with **Rust** and the **Slint** UI framework. Originally ported from a C++/Qt codebase, this version provides enhanced safety, robust synchronization, and easy configuration via TOML.

## 🚀 Features

- **Blazing Fast**: Leverages Rust's performance and safety for real-time robot control.
- **Modern UI**: A responsive and intuitive interface built with Slint.
- **Delta X 2 Optimized**: Native support for X2 specific protocols, including `IsDelta` verification and `FEEDBACK` synchronization.
- **Safety First**: Configurable software limits for X, Y, and Z axes to prevent hardware damage.
- **Dynamic Configuration**: Easily manage seedling plates and robot parameters via a simple `config.toml` file.
- **G-code Support**: Full support for standard Delta X G-codes (G0, G28, G90, G91, etc.).
- **Raspberry Pi Ready**: Runs on Raspberry Pi 3, 4 and 5 (64-bit) with the official 7'' touch display (800x480), in touch-only kiosk mode, with a specialized deployment guide.

## 🛠 Prerequisites

To build and run this project, you need:

1. **Rust**: Install via [rustup.rs](https://rustup.rs/).
2. **Slint Requirements**: Depending on your OS, you may need certain graphics libraries. See the [Slint Prerequisites](https://slint.dev/docs/slint/src/prerequisites) for details.
3. **Serial Port Access**: Ensure your user has permissions to access serial devices (e.g., `dialout` group on Linux).

## 📦 Installation & Build

1. **Clone the repository**:

   ```bash
   git clone https://github.com/guycorbaz/deltax2.git
   cd deltax2
   ```

2. **Configure the robot**:
   Edit `config.toml` to match your serial port and robot limits.

3. **Build the project**:

   ```bash
   cargo build --release
   ```

4. **Run the application**:

   ```bash
   cargo run
   ```

## ⚙️ Configuration

The application uses `config.toml` for all settings. You can define multiple seedling tray layouts (plates) and set safety boundaries for your specific setup.

Example `config.toml` snippet:

```toml
[serial]
port = "/dev/ttyUSB0"
baud_rate = 115200

[robot]
limit_min = { x = -160.0, y = -160.0, z = -200.0 }
limit_max = { x = 160.0, y = 160.0, z = 0.0 }
```

Detailed configuration documentation can be found within the `config.toml` file itself.

## 📖 Documentation

- **User & Administrator Manual**: See [docs/manual.pdf](docs/manual.pdf) — covers installation, Raspberry Pi deployment, configuration, daily operation, troubleshooting, and the full Delta X 2 G-code specification. LaTeX sources are in `docs/` (build with `latexmk -pdf manual.tex`).
- **Source Code**: Run `cargo doc --open` to view the detailed internal API documentation.

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request or open an issue for bugs and feature requests.

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---
*Developed for the Delta X 2 robot ecosystem.*
