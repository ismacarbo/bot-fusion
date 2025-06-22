# BotFusion

A modular robotics platform based on Arduino and Raspberry Pi.

This project starts with basic motor control and IMU integration (MPU6050), and will later expand to include LiDAR-based mapping, autonomous navigation, and multi-sensor fusion.

## Features

- 🔧 Arduino motor control via TB6612 motor driver
- 📡 Serial communication over UART
- 🎯 IMU data (yaw) via MPU6050
- 🧠 Designed for future expansion with LiDAR and SLAM

## Getting Started

1. Flash the Arduino sketch from `/arduino/` to test basic movement.
2. Connect MPU6050 to A4 (SDA) and A5 (SCL).
3. Open serial monitor for yaw data output.

## Roadmap

- [x] Basic 2-wheel motor control
- [x] IMU yaw reading
- [ ] Serial control from Raspberry Pi
- [ ] LiDAR integration
- [ ] SLAM and path planning

## Hardware

- Arduino Uno
- Elegoo Smart Car V4 shield (TB6612)
- MPU6050
- Raspberry Pi (future integration)
- 4 DC motors

## License

MIT
