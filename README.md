# BotFusion

BotFusion is an experimental mobile robotics stack built around an Arduino low-level controller and a Raspberry Pi/Linux host.

The repository currently contains:

- Arduino firmware prototypes for motors, IMU, safety sensors, and odometry experiments
- Python tooling for serial monitoring, LiDAR acquisition, occupancy-grid mapping, and autonomous control
- protocol and architecture documentation for the next communication layer

This is not yet a packaged product or a single polished application. It is the active workspace for the robot communication, sensing, and autonomy stack.

## Current State

Implemented in the repository today:

- low-level Arduino motor control on TB6612FNG-compatible hardware
- MPU6050-based yaw telemetry and gyro zeroing
- ultrasonic and IR safety sensing
- serial command and telemetry exchange between host and controller
- custom RPLidar serial integration
- occupancy-grid mapping and live visualization
- autonomous control experiments that fuse LiDAR and onboard telemetry
- HTTP streaming utilities for LiDAR scans and runtime robot data

Important status note:

- the current firmware and Python controllers still use a line-based serial protocol such as `CMD,<pwmL>,<pwmR>`, `STOP`, `GYRO_ZERO`, `PING`, and `STAT,...`
- `RSP-v1` in `documentation/` is the target protocol specification for the next robot communication library, not the transport already used by every runtime script

## Repository Layout

- `robotScripts/`
  Arduino and host-side serial experiments.
- `robotScripts/motorsIMU/motorsIMU.ino`
  Low-level controller for PWM motor control, MPU6050 yaw telemetry, ultrasonic and IR sensing, and simple serial commands.
- `robotScripts/sensors/sensors.ino`
  Sensor-driven obstacle behavior with servo scanning, IR safety logic, and telemetry output.
- `robotScripts/odometry/odometry.ino`
  IMU-assisted odometry and online PWM/deadzone calibration experiments.
- `robotScripts/arduinoConnection.py`
  Minimal serial monitor for Arduino telemetry.
- `lidar/`
  Host-side LiDAR drivers, occupancy grids, dashboards, SLAM, and robot controller experiments.
- `lidar/lidarLib.py`
  Custom serial interface for RPLidar devices.
- `lidar/slam.py`
  LiDAR-only occupancy-grid SLAM with live Matplotlib visualization.
- `lidar/robotController.py`
  Main host-side robot controller combining LiDAR, gyro telemetry, occupancy mapping, and simple local obstacle avoidance.
- `lidar/robotControllerHttp.py`
  HTTP-streaming variant of the robot controller.
- `lidar/lidarHttpStreamer.py`
  Headless LiDAR-to-HTTP streaming utility.
- `documentation/`
  Architecture and protocol notes for the communication layer and system structure.

## Documentation

- [RSP-v1 Main Spec](documentation/RSP-v1_Main_Spec.md)
- [RSP-v1 Payload Definitions](documentation/RSP-v1_Payload_Definitions.md)
- [Robot Architecture Overview](documentation/Robot_Architecture_Overview.md)

Note:

- `Robot_Architecture_Overview.md` references `RMSM-v1` as the conceptual robot state-machine model
- a dedicated `RMSM-v1` specification is not yet part of this repository

## Quick Start

### 1. Flash the Arduino firmware

Pick the sketch that matches the experiment you want to run.

Typical starting point:

- `robotScripts/motorsIMU/motorsIMU.ino`

### 2. Connect the hardware

The current codebase assumes a setup close to:

- Arduino Uno
- Raspberry Pi or Linux host
- TB6612FNG-compatible motor driver
- MPU6050
- ultrasonic sensor
- IR sensors
- RPLidar on a serial port
- differential-drive DC motors

### 3. Install Python dependencies

There is no pinned environment file yet. At minimum, the Python tools may require:

```bash
pip install numpy matplotlib pyserial scipy requests pygame
```

Not every script needs every dependency:

- `scipy` is optional but recommended for faster nearest-neighbor search in SLAM and control
- `requests` is only needed for HTTP streaming tools
- `pygame` is only needed for `lidar/botController.py`

### 4. Run a basic serial monitor

```bash
python robotScripts/arduinoConnection.py
```

Adjust the serial port inside the script if your controller is not on `/dev/ttyACM0`.

### 5. Run LiDAR-only mapping

```bash
python lidar/slam.py
```

Default LiDAR port is `/dev/ttyUSB0`. The script saves an occupancy map and trajectory when it exits.

### 6. Run the host robot controller

```bash
python lidar/robotController.py
```

By default it expects:

- Arduino on `/dev/ttyACM0`
- RPLidar on `/dev/ttyUSB0`

Use `--help` on the controller scripts to inspect runtime parameters for ports, map bounds, ICP tuning, GUI settings, and output files.

## Outputs

Depending on the script you run, the repository can produce:

- occupancy-grid images
- trajectory CSV files
- live Matplotlib or Pygame visualizations
- HTTP JSON payloads for scans or robot runtime state

## Near-Term Direction

The current development direction is:

- formalize the robot serial protocol as `RSP-v1`
- turn the existing ad-hoc serial link into a reusable communication library
- align firmware payloads, acknowledgments, and telemetry with the documented protocol
- keep LiDAR mapping and autonomous control connected to the same communication model

## License

This project is distributed under the GNU General Public License v3.0. See [LICENSE](LICENSE).
