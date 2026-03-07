This section describes the overall structure of the Inmoov_ROS2 project repository, workspace, and Arduino user folder.

---

# Repository Root: `Inmoov_ROS2/`

Top-level project directory containing source code, documentation, and workspace.

```bash
Inmoov_ROS2/
├── Arduino/                           # Arduino source code and firmware projects
├── docs/                              # Technical documentation and diagrams
├── inmoov_ws/                         # ROS 2 Jazzy workspace
│   ├── src/
│   │   ├── inmoov_voice/              # TTS package
│   │   ├── inmoov_vision/             # Vision package
│   │   ├── inmoov_behaviors/          # Behavior package
│   │   ├── xicro_nodes/               # Xicro communication nodes
│   │   └── Xicro/                     # Xicro code generation tools
│   └── tools/                         # Additional standalone utilities and scripts
│       └── xicro_servo_test_gui.py    # GUI tool for manual servo testing and calibration
├── piper/                             # Local voice synthesis engine and models
└── README.md
```

# Xicro

```bash
~/inmoov_ws/                                       # Main ROS 2 workspace directory
├── src/                                           # Source folder for ROS 2 packages and repositories
│   ├── Xicro/                                     # Fork or repository of XICRO tool used for code generation
│   │   └── xicro_pkg/                             # Main XICRO Python package containing configuration and scripts
│   │       ├── config/                            # Configuration files for hardware and topics
│   │       │   ├── setup_xicro.yaml               # Default configuration for code generation
│   │       │   ├── setup_xicro_subsystem1.yaml    # Configuration for subsystem 1 (right arm)
│   │       │   ├── setup_xicro_subsystem2.yaml    # Configuration for subsystem 2 (left arm + head)
│   │       └── scripts/                           # Scripts for generating firmware and ROS 2 nodes
│   │           ├── generate_library.py            # Generates Arduino firmware libraries (.cpp and .h)
│   │           ├── generate_xicro_node.py         # Generates ROS 2 Python nodes for communication
│   │           └── ...                            # Additional utility scripts and tools
│   └── xicro_nodes/                               # Custom ROS 2 package containing generated XICRO communication nodes
│       ├── xicro_nodes/                           # Python module directory with ROS 2 nodes
│       │   ├── xicro_node_subsys1_ID_1_arduino.py # Node communicating with Arduino Uno (right arm)
│       │   ├── xicro_node_subsys2_ID_2_arduino.py # Node communicating with Arduino Mega (left arm + head)
│       │   └── init.py                            # Python package marker file
│       ├── launch/                                # Launch files for starting ROS 2 nodes
│       │   └── xicro_nodes.launch.py              # Launches both Arduino communication nodes
│       ├── package.xml                            # ROS 2 package manifest with dependencies and metadata
│       └── setup.py                               # Python package build and installation script
├── tools/                                         # Additional standalone utilities and scripts
│   └── xicro_servo_test_gui.py                    # GUI tool for manual servo testing and calibration
└── ...                                            # Other workspace directories and packages
```
---

# Arduino User Folder: `~/Arduino/`

Contains generated Arduino firmware projects for each subsystem.

```bash
~/Arduino/
├── Xicro_subsys1_ID_1/ # Firmware for right arm subsystem (Arduino Uno)
│   ├── Xicro_subsys1_ID_1.ino
│   ├── Xicro_subsys1_ID_1.cpp
│   └── Xicro_subsys1_ID_1.h
├── Xicro_subsys2_ID_2/ # Firmware for left arm + head subsystem (Arduino Mega)
│   ├── Xicro_subsys2_ID_2.ino
│   ├── Xicro_subsys2_ID_2.cpp
│   └── Xicro_subsys2_ID_2.h
└── ... (other Arduino projects)
```

# inmoov_vision


```bash
inmoov_ws/
└── src/
    └── inmoov_vision/
        ├── inmoov_vision/
        │   ├── __init__.py                                   # Initializes the Python module
        │   ├── face_tracker_node.py                          # ROS 2 node for face tracking
        │   ├── face_recognition_node.py                      # ROS 2 node for known-person recognition
        │   ├── generate_encodings.py                         # Script to generate embeddings (offline)
        │   ├── register_face.py                              # Script to capture images of new people
        │   ├── data/
        │   │   └── encodings.pickle                          # Serialized facial embeddings for recognition
        │   └── models/
        │       ├── shape_predictor_68_face_landmarks.dat     # Dlib model for facial landmarks
        │       └── dlib_face_recognition_resnet_model_v1.dat # Dlib model for embedding extraction
        ├── faces_db/                                         # Image database organized by person
        ├── launch/
        │   └── vision_nodes.launch.py                        # Launch file to start both ROS 2 nodes
        ├── resource/
        │   └── inmoov_vision                                 # ament_python requirement
        ├── setup.py                                          # Main package installer
        ├── setup.cfg                                         # Auxiliary configuration
        └── package.xml                                       # ROS 2 package metadata

```

# inmoov_voice


```bash
Inmoov_ROS2/                          # Repository root
├── inmoov_ws/
│   └── src/
│       └── inmoov_voice/             # Voice synthesis and jaw control package
│           ├── package.xml           # ROS 2 package metadata
│           ├── resource/
│           │   └── inmoov_voice      # marker for ament index
│           ├── setup.py              # Python installation script and entry points
│           └── inmoov_voice/
│               ├── __init__.py       # Python package initializer
│               ├── tts_node.py       # Basic TTS node with Piper
│               └── tts_jaw_node.py   # TTS node with energy analysis and jaw control
└── piper/                            # Local Piper TTS engine
    ├── piper                         # Piper executable
    ├── es_ES-davefx-medium.onnx      # ONNX model for Spanish medium-quality voice
    └── es_ES-davefx-medium.onnx.json # Model configuration file
```

# inmoov_behaviors


```bash
inmoov_ws/
└── src/
    └── inmoov_behaviors/                            # ROS 2 package root
        ├── package.xml                              # Package manifest (dependencies, metadata)
        ├── setup.py                                 # build/install configuration (entry_points, data files)
        ├── resource/
        │   └── inmoov_behaviors                     # ament resource marker
        └── inmoov_behaviors/                        # Python module directory
            ├── __init__.py                          # module marker
            ├── face_tracking_behavior_node.py       # node for pan/tilt/eye tracking
            ├── face_recognized_behavior_node.py     # node for greetings and gesture sequence on recognition
            ├── greetings_unknown.txt                # text file with unknown-face greeting lines
            ├── greetings_known.txt                  # text file with known-face greeting templates
            └── movements_known.yaml                 # YAML sequence of servo movements and delays
```

---

[Return to README.md](../README.md)
