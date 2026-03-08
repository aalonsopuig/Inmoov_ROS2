
# Users guide

This document describes how to start the robot’s functionalities.

## Starting the Robot

1. Turn on the PC.

2. Plug in the robot's power supply, keeping the robot switch off.

3. Connect the robot USB cable to the PC.

4. Turn on the robot.

## Communications: Launching Xicro

Open a terminal and run:

```bash
cd inmoov_ws
source Inmoov_ROS2/inmoov_ws/install/setup.bash
ros2 launch xicro_nodes xicro_nodes.launch.py
```

## Vision

### New known people registration

This script will register new faces taking 5 pictures. It will ask you to write down the name of the person and will create a folder and files with this name:

```bash
cd Inmoov_ROS2/inmoov_ws/src/inmoov_vision/inmoov_vision
python3 register_face.py
```

Then we will generate the embeddings of this files:

```bash
cd Inmoov_ROS2/inmoov_ws
source install/setup.bash
mkdir -p Inmoov_ROS2/inmoov_ws/src/inmoov_vision/inmoov_vision/data
python3 src/inmoov_vision/inmoov_vision/generate_encodings.py
```

### Launching Vision Nodes

Open a new terminal tab and run:

```bash
source Inmoov_ROS2/inmoov_ws/install/setup.bash
ros2 launch inmoov_vision vision_nodes.launch.py
```

To run in debug mode instead, execute:

```bash
ros2 launch inmoov_vision vision_nodes_debug.launch.py
```

## Launching Text-to-Speech Node
Open a new terminal tab and run:

```bash
ros2 run inmoov_voice tts_jaw_node
```

## Launching Face Tracking Behavior Node
Open a new terminal tab and run:

```bash
source install/setup.bash
ros2 run inmoov_behaviors face_tracking_behavior_node
```
To show debug messages (development stage), run:

```bash
ros2 run inmoov_behaviors face_tracking_behavior_node \
  --ros-args \
    --log-level face_tracking_behavior_node:=DEBUG \
    --log-level rcl:=INFO
```

## Launching Face Recognition Behavior Node
Open a new terminal tab and run:

```bash
source install/setup.bash
ros2 run inmoov_behaviors face_recognized_behavior_node
```
To show debug messages (development stage), run:

```bash
ros2 run inmoov_behaviors face_recognized_behavior_node \
  --ros-args \
    --log-level face_recognized_behavior_node:=DEBUG \
    --log-level rcl:=WARN
```

---

[Return to README.md](../README.md)
