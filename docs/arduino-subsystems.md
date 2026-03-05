# Approach

Servo control is a fundamental aspect of any humanoid robot, especially when using 3D-printed mechanisms and high-torque servos to move large and delicate parts, as is the case with InMoov. Sudden or overly fast movements can cause mechanical overload, current spikes, and even break plastic components, so it is essential that all movements are smooth and well-controlled.

The standard Arduino `Servo.h` library is the most common option for controlling servomotors via PWM, allowing multiple servos to be connected to boards like Arduino Uno and Mega. However, this library does not provide any built-in mechanism for velocity control or for generating acceleration/deceleration ramps—it simply moves the servo immediately to the specified angle.

There are third-party Arduino libraries for more sophisticated velocity control, such as [VarSpeedServo by Jeff Korman](https://github.com/netlabtoolkit/VarSpeedServo), which adds a speed parameter to each servo movement. However, this library is limited to eight simultaneous servos, which is insufficient for InMoov (up to nine servos on the right arm, up to fourteen on the Mega board). Other advanced alternatives, like VarSpeedServoRA4M1 by Kaled Souky, are designed for more powerful microcontrollers than Arduino Uno R3 or Mega, and therefore are not suitable for this hardware.

Given these limitations, a custom velocity control system was developed, fully compatible with the project’s architecture and scalable to the required number of servos.

The software compares the current servo position to the target position and performs small increments or decrements at fixed intervals, interpolating the movement in small steps. This method—known as linear interpolation—ensures smooth, constant-speed motion without acceleration or deceleration ramps.

The main drawback of this approach is that the velocity is constant throughout the movement, with no acceleration or deceleration ramp. In advanced robotics, trapezoidal or S-curve velocity profiles are used, where motion starts slowly, accelerates to cruising speed, and decelerates again before reaching the target. Implementing such ramps could be a future improvement to allow even gentler and more precise movement.

# Implementation

Each Arduino subsystem has a specific program:

- [Xicro_subsys1_ID_1.ino](../Arduino/Xicro_subsys1_ID_1/Xicro_subsys1_ID_1.ino) for Subsystem 1 (Arduino Uno)
- [Xicro_subsys1_ID_2.ino](../Arduino/Xicro_subsys2_ID_2/Xicro_subsys2_ID_2.ino) for Subsystem 2 (Arduino Mega)

The subsystems use the XICRO interface to receive commands from the central computer, allowing each joint to be assigned an individual target angle. Each servo is parameterized with its own safe movement range, a rest position, and a velocity (defined by step size and update interval). Servo movements are interpolated—gradually advancing or retracting until the target is reached—ensuring smooth operation and protecting the mechanics.

If a command with value zero is received, the program interprets this as a request to move the joint to its rest position, without the central system needing to know this value in advance. Received angles are always limited to the safe range configured for each servo before being sent to the motor. The control system is robust against out-of-range values and can be adapted to any servo in the assembly, including both large and small joints. It is flexible and allows easy adjustment of both the global movement speed and the individual parameters of each joint.

## Subsystem 1: Right Arm (Arduino Uno)

| Topic              | Servo Instance       | Arduino Pin | Speed | Rest Angle (º) | Min (º) | Max (º) |
|--------------------|---------------------|-------------|-------|----------------|---------|---------|
| thumb_finger_R     | s_thumb_finger_R    | 2           | 2     | 144            | 112     | 177     |
| index_finger_R     | s_index_finger_R    | 3           | 2     | 135            | 85      | 150     |
| middle_finger_R    | s_middle_finger_R   | 4           | 2     | 135            | 70      | 140     |
| ring_finger_R      | s_ring_finger_R     | 5           | 2     | 135            | 80      | 160     |
| pinky_finger_R     | s_pinky_finger_R    | 6           | 2     | 135            | 90      | 165     |
| bicep_R            | s_bicep_R           | 8           | 1     | 50             | 50      | 110     |
| rotate_R           | s_rotate_R          | 9           | 1     | 90             | 80      | 135     |
| shoulder_R         | s_shoulder_R        | 10          | 1     | 90             | 90      | 120     |
| omoplate_R         | s_omoplate_R        | 11          | 1     | 5              | 0       | 20      |

## Subsystem 2: Left Arm & Head (Arduino Mega)

| Topic              | Servo Instance       | Arduino Pin | Speed | Rest Angle (º) | Min (º) | Max (º) |
|--------------------|---------------------|-------------|-------|----------------|---------|---------|
| thumb_finger_L     | s_thumb_finger_L    | 2           | 2     | 144            | 112     | 177     |
| index_finger_L     | s_index_finger_L    | 3           | 2     | 135            | 85      | 150     |
| middle_finger_L    | s_middle_finger_L   | 4           | 2     | 135            | 70      | 140     |
| ring_finger_L      | s_ring_finger_L     | 5           | 2     | 135            | 80      | 160     |
| pinky_finger_L     | s_pinky_finger_L    | 6           | 2     | 135            | 90      | 165     |
| bicep_L            | s_bicep_L           | 8           | 1     | 50             | 50      | 110     |
| rotate_L           | s_rotate_L          | 9           | 1     | 90             | 80      | 135     |
| shoulder_L         | s_shoulder_L        | 10          | 1     | 90             | 90      | 120     |
| omoplate_L         | s_omoplate_L        | 11          | 1     | 5              | 0       | 20      |
| neck               | s_neck              | 12          | 2     | 110            | 25      | 170     |
| rothead            | s_rothead           | 13          | 1     | 90             | 50      | 130     |
| jaw                | s_jaw               | 26          | 10    | 9              | 9       | 20      |
| eye_x (tilt)       | s_eye_x             | 22          | 2     | 80             | 70      | 107     |
| eye_y (pan)        | s_eye_y             | 24          | 2     | 80             | 60      | 100     |

**Note:** `eye_y` is pan and `eye_x` is tilt.
