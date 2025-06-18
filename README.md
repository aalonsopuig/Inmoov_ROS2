# Inmoov_ROS2

ROS 2-based control system for a humanoid InMoov robot  
This project integrates basic autonomous behavior, visual perception, speech synthesis, and distributed servo control using ROS 2.

## Project Overview

This repository is part of a Master's Thesis project focused on educational and maker-oriented humanoid robotics. The robot is based on the open-source InMoov platform and is built using low-cost components.

Main features:

- Face detection, tracking, and recognition (offline)
- Local text-to-speech (TTS) using Piper
- Distributed servo control via Arduino Uno and Mega
- Behavior logic based on recognized individuals
- ROS 2 node-based architecture, designed for reproducibility

## Repository Structure

Arduino/        -> Code for the two Arduino-based subsystems
inmoov_ws/      -> ROS 2 workspace containing all custom packages
piper/          -> Local speech synthesis system
docs/           -> Technical documentation (hardware, configuration, usage)

## Quick Start

cd inmoov_ws
colcon build
source install/setup.bash

All nodes are designed to run offline in a local ROS 2 environment.

## Documentation

For detailed documentation, visit the [Wiki](https://github.com/aalonsopuig/Inmoov_ROS2/wiki).

## License

This project is licensed under the [Apache License 2.0](LICENSE).
