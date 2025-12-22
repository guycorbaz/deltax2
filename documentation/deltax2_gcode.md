# Delta X 2 G-code Specifications

This document outlines the G-code commands and communication protocol for the Delta X 2 robot.

## 1. Communication Protocol

- **Baud Rate**: 115200
- **Line Ending**: `\n` (Newline)
- **Handshaking**: The robot typically responds to commands. For critical synchronization, use the `FEEDBACK` parameter.

## 2. Identification & Status

### `IsDelta`

Queries the device to confirm it is a Delta X robot.

- **Response**: `YesDelta`

### `DeltaState`

Queries the current operating state of the robot.

- **Possible Responses**:
  - `Free`: Idle and ready.
  - `Running`: Executing a command.
  - `Wait`: Waiting for input or synchronization.
  - `Almostdone`: Near the end of a movement.
  - `Done`: Movement or command completed.

### `Position`

Returns the current coordinates of the robot.

- **Response Format**: `X:<val> Y:<val> Z:<val> W:<val> U:<val>`

## 3. Movement Commands (G-codes)

### `G00` / `G01`: Linear Movement

Moves the end-effector to the specified coordinates.

- **Syntax**: `G01 X<val> Y<val> Z<val> F<speed>`
- **Example**: `G01 X10.5 Y-20.0 Z-150.0 F2000`

### `G28`: Homing

Moves all axes to their home position (top endstops).

- **Origin (X0 Y0 Z0)**: After homing, the robot is positioned at the top center.
- **Z-Axis**: Moves downward (negative values). Z0 is the top, Z-200 is near the bottom of the workspace.

### `G90`: Absolute Positioning

Interpret subsequent coordinates as absolute positions relative to the origin.

### `G91`: Relative Positioning

Interpret subsequent coordinates as displacements from the current position (useful for jogging).

## 4. Synchronization (FEEDBACK)

### `FEEDBACK:<string>`

You can append `FEEDBACK:<custom_string>` to any G-code command. The robot will echo the `<custom_string>` back to the serial port once the command has been successfully executed.

- **Example**: `G01 X0 Y0 Z-50 FEEDBACK:done`
- **Response**: `done` (sent after the move finishes).

## 5. End Effector Controls (M-codes)

### `M03`: Output On

Activates the end effector (e.g., opens gripper, starts vacuum).

### `M05`: Output Off

Deactivates the end effector.

## 6. Physical Workspace (SP-X2)

- **X Range**: -160 mm to +160 mm
- **Y Range**: -160 mm to +160 mm
- **Z Range**: -200 mm to 0 mm (0 is at the top)
- **Working Diameter**: 320 mm
