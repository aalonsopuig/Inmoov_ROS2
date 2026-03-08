# Installation guide

## Hardware Requirements

- **InMoov humanoid robot (upper body)**  
  Printed parts and assembled with servomotors and sensors as per the original open-source design by [Gaël Langevin](https://inmoov.fr/).

- **Main System:**  
  A computer with at least Intel Core i5-4210U at 1.70 GHz, 16 GB of RAM, 200 GB Hard Drive.

- **Microcontroller subsystems connected via USB to the main system:**  
  - Arduino Uno (controls right arm)  
  - Arduino Mega (controls left arm and head)

- **Camera:**  
  USB webcam mounted on the robot’s head for vision input.

## Software Requirements

- Ubuntu 24.04 LTS or compatible Linux distribution (Lubuntu)  
- ROS 2 Jazzy Jalisco  
- Python 3.12 or later  
- Arduino IDE 2.2.1 or later for uploading firmware to Arduino boards  
- Additional dependencies as listed in the repository’s `package.xml` and `requirements.txt`

---


> **IMPORTANT:** It is asumed that you meet the requirements described above before going ahead with the installation of the project.


## Clone the Project Repository

The project repository is available at:  
[https://github.com/aalonsopuig/Inmoov_ROS2](https://github.com/aalonsopuig/Inmoov_ROS2)

The original development environment placed the repository directly in the user's home directory (`~/`), but users can clone it wherever they prefer, adjusting commands accordingly.

The following commands check out version `v1.1.0`:

```bash
git clone https://github.com/aalonsopuig/Inmoov_ROS2.git
cd Inmoov_ROS2
git checkout v1.0.0
cd inmoov_ws
```


## Install ROS 2 Dependencies
Inside the cloned repository, the ROS 2 workspace is located in inmoov_ws.
Navigate there and run:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Build and Source the Environment

Build the workspace with:

```bash
colcon build
source install/setup.bash
```

## Python Library Requirements for XICRO
XICRO’s code generation functionality depends on specific Python libraries. One of them is `pyserial`, which is often confused with the unrelated `serial` package. If `serial` is installed in your environment, uninstall it before installing `pyserial` to avoid conflicts.

```bash
pip uninstall serial --break-system-packages
pip install pyserial numpy --break-system-packages
echo 'export PATH=$HOME/.local/bin:$PATH' >> ~/.bashrc
source ~/.bashrc
python -c "import serial; print('pyserial version:', serial.__version__)"
python -c "import numpy; print('numpy version:', numpy.__version__)"
```

## Hardware Parameter Customization and XICRO Library Regeneration

The project code and structure are designed to work with specific Arduino boards connected to predefined ports, configured in the YAML files:

- [setup_xicro_subsystem1.yaml](../inmoov_ws/src/Xicro/xicro_pkg/config/setup_xicro_subsystem1.yaml)  
- [setup_xicro_subsystem2.yaml](../inmoov_ws/src/Xicro/xicro_pkg/config/setup_xicro_subsystem2.yaml)

These files are located in the repository at:

`inmoov_ws/src/Xicro/xicro_pkg/config/`

If you wish to use different board models, serial ports, or modify the connected servos/controllers, you must edit these YAML files following the official [XICRO documentation](https://github.com/aalonsopuig/Xicro-enhanced) to adapt them to your hardware and desired ROS 2 topics.

> **IMPORTANT:** The XICRO code generation system works with a single configuration file named `setup_xicro.yaml`. Therefore, before generating code for each subsystem, copy the corresponding configuration file to this standard name.


### Arduino Library Generation (ROS 2 communication firmware)

For each subsystem, follow these steps:

#### Subsystem 1 (right arm)

1. Modify `setup_xicro_subsystem1.yaml` as needed.
2. Run:

```bash
cd Inmoov_ROS2/inmoov_ws
cp src/Xicro/xicro_pkg/config/setup_xicro_subsystem1.yaml src/Xicro/xicro_pkg/config/setup_xicro.yaml
colcon build
source install/setup.bash
ros2 run xicro_pkg generate_library.py -mcu_type arduino
```

#### Subsystem 2 (left arm and head)

1. Modify setup_xicro_subsystem2.yaml as needed.
2. Run:

```bash
cd Inmoov_ROS2/inmoov_ws
cp src/Xicro/xicro_pkg/config/setup_xicro_subsystem2.yaml src/Xicro/xicro_pkg/config/setup_xicro.yaml
colcon build
source install/setup.bash
ros2 run xicro_pkg generate_library.py -mcu_type arduino
```

This will generate .cpp and .h files in ~/Arduino/Xicro_subsys2_ID_2.

### Python Communication Nodes Generation (for ROS 2)
Whenever you change hardware configuration or topics, you must also regenerate the Python communication nodes with the updated YAML file, following the same copying procedure:

#### Subsystem 1 (right arm)

```bash
cd Inmoov_ROS2/inmoov_ws
cp src/Xicro/xicro_pkg/config/setup_xicro_subsystem1.yaml src/Xicro/xicro_pkg/config/setup_xicro.yaml
colcon build
source install/setup.bash
ros2 run xicro_pkg generate_xicro_node.py -mcu_type arduino
```

#### Subsystem 2 (left arm and head)

```bash
cd Inmoov_ROS2/inmoov_ws
cp src/Xicro/xicro_pkg/config/setup_xicro_subsystem2.yaml src/Xicro/xicro_pkg/config/setup_xicro.yaml
colcon build
source install/setup.bash
ros2 run xicro_pkg generate_xicro_node.py -mcu_type arduino
```

## Arduino IDE Installation on PC

This section guides you through installing and launching Arduino IDE 2.2.1 on Linux using the AppImage format, enabling serial port access and resolving necessary dependencies and permissions.

```bash
# Create directory for organized Arduino IDE installation
mkdir -p ~/Applications/arduino-ide
cd ~/Applications/arduino-ide

# Download Arduino IDE 2.2.1 AppImage (portable)
wget https://downloads.arduino.cc/arduino-ide/arduino-ide_2.2.1_Linux_64bit.AppImage

# Make the AppImage executable
chmod +x arduino-ide_2.2.1_Linux_64bit.AppImage

# Install libfuse2 (required for running AppImages on Ubuntu 22.04+)
sudo apt update
sudo apt install libfuse2

# Create a temporary alias to run Arduino IDE with --no-sandbox option
alias arduino='~/Applications/arduino-ide/arduino-ide_2.2.1_Linux_64bit.AppImage --no-sandbox'

# Add alias permanently to ~/.bashrc
echo "alias arduino='~/Applications/arduino-ide/arduino-ide_2.2.1_Linux_64bit.AppImage --no-sandbox'" >> ~/.bashrc

# Apply the changes immediately
source ~/.bashrc

# Add current user to 'dialout' group to access serial ports (e.g. /dev/ttyACM0)
sudo usermod -aG dialout $USER

# Note: You need to log out and log back in (or reboot) for group changes to take effect
Uploading Firmware to Arduino Boards
Load the appropriate firmware sketch depending on the subsystem:
```

For Subsystem 1:

```bash
arduino ~/Arduino/Xicro_subsys1_ID_1/Xicro_subsys1_ID_1.ino
```

For Subsystem 2:

```bash
arduino ~/Arduino/Xicro_subsys2_ID_2/Xicro_subsys2_ID_2.ino
```

Within the Arduino IDE, select the corresponding board (Arduino Uno for subsystem 1 or Arduino Mega for subsystem 2), compile, verify, and upload the firmware.



## Vision Requirements

Make sure your system has the native libraries required by both OpenCV and Dlib, as well as the ROS bridge package `cv_bridge`. For example, on Lubuntu with ROS 2 Jazzy, you can run:

```bash
sudo apt update
sudo apt install \
  python3-opencv \               # Python bindings for OpenCV
  python3-dlib \                 # Dlib module packaged for Ubuntu
  ros-jazzy-cv-bridge            # cv_bridge package for ROS 2 Jazzy
```

It is also recommended to install Colcon extensions:

```bash
sudo apt install python3-colcon-common-extensions
```

Then build the vision package:

```bash
cd Inmoov_ROS2/inmoov_ws
colcon build --packages-select inmoov_vision
source install/setup.bash
```


## Text-to-Speech (TTS) Requirements with Piper

Install the necessary Python packages:

```bash
pip install rclpy std_msgs
pip install sounddevice --break-system-packages
pip install soundfile --break-system-packages
```
Build the TTS package:

```bash
cd Inmoov_ROS2/inmoov_ws
colcon build --packages-select inmoov_voice
source install/setup.bash
```

### Changing Voices or Languages

Change the default voice/language only if you want to use a different voice than the project's default (es_ES-davefx-medium).

#### Downloading a Different Voice/Language

To use a different voice or language with Piper, simply download the two files (ONNX model and its JSON config) for the chosen voice and update the node paths accordingly. For example, to switch to the voice en_US-amy-medium:

Visit [Piper Voice Samples](https://rhasspy.github.io/piper-samples/), select the voice, and click Download. This will lead you to the repository location for that voice.

Download the model files using wget inside the ~/piper directory:

```bash
cd Inmoov_ROS2/piper
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/amy/medium/en_US-amy-medium.onnx
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/amy/medium/en_US-amy-medium.onnx.json
```

#### Update the ROS Node

Edit the file inmoov_ws/src/inmoov_voice/inmoov_voice/tts_jaw_node.py and locate these lines in the constructor:

```bash
self.model_onnx = os.path.join(self.model_dir, 'es_ES-davefx-medium.onnx')
self.model_cfg  = os.path.join(self.model_dir, 'es_ES-davefx-medium.onnx.json')
```

Replace them with the new model filenames. For the example voice:

```bash
self.model_onnx = os.path.join(self.model_dir, 'en_US-amy-medium.onnx')
self.model_cfg  = os.path.join(self.model_dir, 'en_US-amy-medium.onnx.json')
```

#### Rebuild the Package

Finally, rebuild the package and source the environment again:

```bash
cd Inmoov_ROS2/inmoov_ws
colcon build --packages-select inmoov_voice
source install/setup.bash
```

## Behavior Package Requirements

Once the repository is cloned, the behavior nodes will already be available in the `inmoov_behaviors` package. To adjust the robot's speech messages and gestures, you only need to edit two files:

### Greetings

- The phrases the robot says when it detects an unknown person are in:  
  `inmoov_ws/src/inmoov_behaviors/inmoov_behaviors/greetings_unknown.txt`

- The phrases for recognized persons are in:  
  `inmoov_ws/src/inmoov_behaviors/inmoov_behaviors/greetings_known.txt`

Simply add, remove, or modify lines in these files (one phrase per line).

In `greetings_known.txt`, use the placeholder `<nombre>` wherever you want the recognized person's name to appear.

### Movement Sequences

The gestures the robot performs upon recognizing someone are defined in:  
`inmoov_ws/src/inmoov_behaviors/inmoov_behaviors/movements_known.yaml`

Each YAML block specifies a set of servos and their target angles, as well as a delay (in seconds) before moving to the next step.

To modify the behavior, open this file, adjust the joint names, their angles, or the wait times, and save.

### Rebuild After Changes

After modifying any of these files, rebuild and source the workspace:

```bash
cd Inmoov_ROS2/inmoov_ws
colcon build --packages-select inmoov_behaviors
source install/setup.bash

---

[Return to README.md](../README.md)
