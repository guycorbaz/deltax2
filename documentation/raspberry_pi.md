# Raspberry Pi Deployment Guide

This guide describes how to deploy and run the DeltaX2 application on a Raspberry Pi with a 7'' touch screen (800x480 resolution).

## 1. Operating System

We recommend using **Raspberry Pi OS (64-bit)**. The Lite version is suitable if you plan to run the app in Kiosk mode without a full desktop environment.

## 2. Performance & Rendering

Slint supports multiple rendering backends. On a Raspberry Pi, the **`linuxkms`** backend is often the most performant as it renders directly to the screen without an X11 or Wayland overhead.

### Running with LinuxKMS

1. Ensure your user is in the `video` and `input` groups:

   ```bash
   sudo usermod -aG video,input $USER
   ```

2. Run the application with the following environment variables:

   ```bash
   SLINT_BACKEND=linuxkms ./deltax2
   ```

## 3. Touch Screen Calibration

If the touch input is inverted or misaligned, you may need to configure `libinput` or use a udev rule to set the `LIBINPUT_CALIBRATION_MATRIX`.

## 4. Kiosk Mode

To run the application as a standalone interface on boot:

1. Create a systemd service or use a `.xsession` file if using a display manager.
2. In `ui/appwindow.slint`, you can set `no-frame: true` to remove window decorations.

## 5. Cross-Compilation (Optional)

Building on the Pi can be slow. You can cross-compile from a faster machine using `cross-rs`:

1. Install `cross`: `cargo install cross`
2. Run: `cross build --target aarch64-unknown-linux-gnu --release`
3. Copy the resulting binary and `config.toml` to the Pi.

## 6. Serial Port Permissions

If the application cannot open the serial port, ensure your user has permission:

```bash
sudo usermod -aG dialout $USER
```

Relogin for changes to take effect.

## 7. System Dependencies

If you are building the application on the Pi or running it with the default renderer, you may need the following system libraries:

```bash
sudo apt-get update
sudo apt-get install -y libfreetype6-dev libfontconfig1-dev libssh-dev
```
