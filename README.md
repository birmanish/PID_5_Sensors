# PID_5_Sensors
![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)

A hardware-agnostic, highly customizable PID line-following library for autonomous robots utilizing a 5-sensor array.

## Features
- **Kinematic Y-Axis Correction**: Corrects lateral shift calculations by accounting for the robot's angular velocity and physical sensor offsets. Crucial for robots using curved or arc-shaped sensor arrays.
- **Weighted Centroid Calculation**: Computes a precise, continuous floating-point position (from -2.0 to +2.0) based on analog sensor intensity, offering smoother control than binary (black/white) arrays.
- **Adaptive Urgency**: Automatically applies a higher proportional gain (`KpUrgency`) when extreme deviation is detected, preventing the robot from overshooting tight corners.
- **Hardware Agnostic**: Does not lock you into specific microcontrollers, analog pins, or motor drivers. Pass your raw ADC readings to the library, and receive PWM-ready motor speeds back.
- **Path Detection Helpers**: Built-in methods to easily detect intersections (`isCrossing`), lost lines (`isLost`), and center alignment (`isCenterDetected`).

## Theory of Operation: Kinematic Correction
Traditional line followers assume all sensors are arranged in a perfectly straight horizontal line. If you are using a curved sensor array, the outer sensors are physically pushed back (Y-axis offset) compared to the central sensor. 

This library uses the robot's current linear velocity (`vLin`) and angular velocity (`omega`) to mathematically project what the outer sensors *would* see if they were perfectly aligned with the center axis. This look-ahead compensation drastically reduces high-frequency oscillation in tight corners.

## API Reference
- `void setSensorGeometry(float xOffsets[5], float yOffsets[5])`: Defines the physical layout of your sensors relative to the center sensor (in mm).
- `void setThreshold(int index, float threshold, float ground)`: Calibrates a specific sensor.
- `void setWhiteLine(bool white)`: Set to `true` if following a white line on a black surface. 
- `float centroid(float rawValues[5], float vLin, float omega)`: Returns the current deviation from the line center.
