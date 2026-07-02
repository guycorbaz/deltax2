# DeltaX2 Control Application

A modern, high-performance control application for the **Delta X 2** robot, built with **Rust** and the **Slint** UI framework. Originally ported from a C++/Qt codebase, this version provides enhanced safety, robust synchronization, and easy configuration via TOML.

## 🚀 Features

- **Blazing Fast**: Leverages Rust's performance and safety for real-time robot control.
- **Modern UI**: A responsive and intuitive interface built with Slint.
- **Delta X 2 Optimized**: Native support for X2 specific protocols, including `IsDelta` verification and `FEEDBACK` synchronization.
- **Safety First**: Configurable software limits for X, Y, and Z axes to prevent hardware damage.
- **Dynamic Configuration**: Easily manage seedling plates and robot parameters via a simple `config.toml` file.
- **G-code Support**: Full support for standard Delta X G-codes (G0, G28, G90, G91, etc.).
- **Raspberry Pi Ready**: Optimized for 7'' touch screens (800x480) with a specialized deployment guide.

## 🛠 Prerequisites

To build and run this project, you need:

1. **Rust**: Install via [rustup.rs](https://rustup.rs/).
2. **Slint Requirements**: Depending on your OS, you may need certain graphics libraries. See the [Slint Prerequisites](https://slint.dev/docs/slint/src/prerequisites) for details.
3. **Serial Port Access**: Ensure your user has permissions to access serial devices (e.g., `dialout` group on Linux).

## 📦 Installation & Build

1. **Clone the repository**:

   ```bash
   git clone https://github.com/yourusername/deltax2.git
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

- **User & Administrator Manual**: See [documentation/manual.pdf](documentation/manual.pdf) — covers installation, Raspberry Pi deployment, configuration, daily operation, troubleshooting, and the full Delta X 2 G-code specification. LaTeX sources are in `documentation/` (build with `latexmk -pdf manual.tex`).
- **Source Code**: Run `cargo doc --open` to view the detailed internal API documentation.

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request or open an issue for bugs and feature requests.

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details (or add your preferred license).

---
*Developed for the Delta X 2 robot ecosystem.*
