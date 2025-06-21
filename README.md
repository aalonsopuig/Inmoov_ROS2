# Inmoov_ROS2

ROS 2-based control system for a humanoid InMoov robot designed by Gael Langevin. <br>
This project integrates basic autonomous behavior, visual perception, speech synthesis, and distributed servo control using ROS 2.

[![Watch the video](https://img.youtube.com/vi/HUr0_2Sw4ho/0.jpg)](https://www.youtube.com/watch?v=HUr0_2Sw4ho)


## Project Overview

This repository is part of a Master's Thesis project focused on educational and maker-oriented humanoid robotics. The robot is based on the open-source InMoov platform and is built using low-cost components.

Main features:

- Face detection, tracking, and recognition (offline)
- Local text-to-speech (TTS) using Piper
- Distributed servo control via Arduino Uno and Mega
- Behavior logic based on recognized individuals
- ROS 2 node-based architecture, designed for reproducibility

## Repository Structure

Arduino/        -> Code for the two Arduino-based subsystems<br>
inmoov_ws/      -> ROS 2 workspace containing all custom packages<br>
piper/          -> Local speech synthesis system<br>
docs/           -> Technical documentation (hardware, configuration, usage)<br>

## Quick Start

cd inmoov_ws<br>
colcon build<br>
source install/setup.bash<br>

All nodes are designed to run offline in a local ROS 2 environment.

## Documentation

For detailed documentation, visit the [Wiki](https://github.com/aalonsopuig/Inmoov_ROS2/wiki).

## License

This project is licensed under the [Apache License 2.0](LICENSE).
