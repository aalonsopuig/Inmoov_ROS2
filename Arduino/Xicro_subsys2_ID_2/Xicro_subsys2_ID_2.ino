/*
  File: Xicro_subsys2_ID_2.ino
  Description: This program implements subsystem 2 of the InMoov robot, responsible for controlling the servomotors of the left arm and head using an Arduino Mega and ROS 2 via the XICRO interface. It enables individual control of each joint or axis, applying safety limits, rest position, and adjustable speed for each servo.
  Author: Alejandro Alonso Puig
  Date: May 29, 2025
  License: Apache License, Version 2.0

  Copyright 2025 Alejandro Alonso Puig

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

  General description:
  This program controls the left arm and head servos of the InMoov robot using an Arduino Mega board. It uses the XICRO interface to receive commands from a ROS 2-based central computer, allowing each servo to be assigned a target angle, speed, safety limits, and a rest position.

  Servo movements are interpolated, gradually advancing toward the target to protect the mechanics and avoid abrupt movements. If the received command is zero, the servo automatically moves to its predefined rest position. The system is designed to be safe and flexible, adaptable to various servos, and allows easy recalibration of parameters.

  Usage examples:
    - Sending an angle (e.g., 130) from ROS 2 moves the corresponding servo to that value, limited by its safety range.
    - Sending a 0 to the corresponding topic sends the joint to its defined rest position.
    - State check:
        if (servos[BICEP_L].current == servos[BICEP_L].target) {
          // The left biceps has reached its target position
        }
    - Manual speed and target modification:
        servos[SHOULDER_L].target = 120;
        servos[SHOULDER_L].step = 1;
*/

#include "Xicro_subsys2_ID_2.h"
#include <Servo.h>

#define INTERVAL_MS 60   // Global update interval for all servos (in milliseconds, adjustable)

#define THUMB_L      0
#define INDEX_L      1
#define MIDDLE_L     2
#define RING_L       3
#define PINKY_L      4
#define BICEP_L      5
#define ROTATE_L     6
#define SHOULDER_L   7
#define OMOPLATE_L   8
#define NECK         9
#define ROTHEAD      10
#define JAW          11
#define EYE_X        12
#define EYE_Y        13
#define NUM_SERVOS   14

struct SmoothServo {
  Servo servo;                 // Arduino Servo object (handles PWM)
  int current;                 // Current servo position (degrees)
  int target;                  // Target position to reach (degrees)
  int rest_angle;              // Safe rest angle at startup (degrees)
  int min_angle;               // Minimum allowed angle (mechanical limit, degrees)
  int max_angle;               // Maximum allowed angle (mechanical limit, degrees)
  unsigned long last_update;   // Last update time (milliseconds)
  int step;                    // Increment/decrement per update (speed in degrees)
  uint8_t pin;                 // Arduino pin connected to the servo
  bool first_commanded;        // Indicates if the first valid command has been received
};

// Servo configuration: adjust rest_angle, min, max, step, and pin as needed for your robot
SmoothServo servos[NUM_SERVOS] = {
  //         Servo()  current target rest_angle  min  max  last_update step pin first_commanded
  {Servo(), 0, 0,   144,    112, 177, 0, 2,  2,  false},   // THUMB_L: Left thumb
  {Servo(), 0, 0,   135,     85, 150, 0, 2,  3,  false},   // INDEX_L: Left index finger
  {Servo(), 0, 0,   135,     70, 140, 0, 2,  4,  false},   // MIDDLE_L: Left middle finger
  {Servo(), 0, 0,   135,     80, 160, 0, 2,  5,  false},   // RING_L: Left ring finger
  {Servo(), 0, 0,   135,     90, 165, 0, 2,  6,  false},   // PINKY_L: Left pinky finger
  {Servo(), 0, 0,    50,     50, 110, 0, 1,  8,  false},   // BICEP_L: Left biceps
  {Servo(), 0, 0,    90,     80, 135, 0, 1,  9,  false},   // ROTATE_L: Left arm rotation
  {Servo(), 0, 0,    90,     90, 120, 0, 1, 10,  false},   // SHOULDER_L: Left shoulder
  {Servo(), 0, 0,     5,      0,  20, 0, 1, 11,  false},   // OMOPLATE_L: Left omoplate
  {Servo(), 0, 0,   110,     25, 170, 0, 2, 12,  false},   // NECK: Neck
  {Servo(), 0, 0,    90,     50, 130, 0, 1, 13,  false},   // ROTHEAD: Head rotation
  {Servo(), 0, 0,     9,      9,  20, 0,10, 26,  false},   // JAW: Jaw
  {Servo(), 0, 0,    80,     70, 107, 0, 2, 22,  false},   // EYE_X: Eyes horizontal axis
  {Servo(), 0, 0,    80,     60, 100, 0, 2, 24,  false}    // EYE_Y: Eyes vertical axis
};

Xicro xicro;

// Restricts any value to the safety limits for a servo
int safe_constrain(int value, int min_val, int max_val) {
  return constrain(value, min_val, max_val);
}

void setup() {
  Serial.begin(57600);
  xicro.begin(&Serial);

  // Initialize all servos: set to rest position, attach to pins, and reset control state variables
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].current = servos[i].rest_angle;      // Initialize current position to rest
    servos[i].target  = servos[i].rest_angle;      // Initialize target to rest
    servos[i].servo.attach(servos[i].pin);         // Attach servo to its pin
    servos[i].servo.write(servos[i].rest_angle);   // Move servo to rest position
    servos[i].last_update = millis();              // Store initial time to manage the update interval
    servos[i].first_commanded = false;             // Mark as not yet received ROS2 command
  }
}

void loop() {
  xicro.Spin_node();

  // Read all values received from ROS/XICRO for each servo, in the same order as the servo array
  int incoming_vals[NUM_SERVOS] = {
    xicro.Subscription_thumb_finger_L.message.data,
    xicro.Subscription_index_finger_L.message.data,
    xicro.Subscription_middle_finger_L.message.data,
    xicro.Subscription_ring_finger_L.message.data,
    xicro.Subscription_pinky_finger_L.message.data,
    xicro.Subscription_bicep_L.message.data,
    xicro.Subscription_rotate_L.message.data,
    xicro.Subscription_shoulder_L.message.data,
    xicro.Subscription_omoplate_L.message.data,
    xicro.Subscription_neck.message.data,
    xicro.Subscription_rothead.message.data,
    xicro.Subscription_jaw.message.data,
    xicro.Subscription_eye_x.message.data,
    xicro.Subscription_eye_y.message.data
  };

  for (int i = 0; i < NUM_SERVOS; i++) {
    // Activate ROS/XICRO control as soon as any value arrives (including 0)
    if (!servos[i].first_commanded && incoming_vals[i] != 0) {
      servos[i].first_commanded = true; // First valid command received
    }

    // If already activated, interpret a command of 0 as "go to rest position"
    if (servos[i].first_commanded) {
      if (incoming_vals[i] == 0) {
        servos[i].target = servos[i].rest_angle;
      } else {
        servos[i].target = safe_constrain(incoming_vals[i], servos[i].min_angle, servos[i].max_angle);
      }
    } else {
      // Before any command arrives, stay at rest position
      servos[i].target = servos[i].rest_angle;
    }
  }

  unsigned long now = millis();
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (now - servos[i].last_update > INTERVAL_MS) {
      servos[i].last_update = now; // Update the timestamp for this servo's last movement

      // Smooth interpolation towards target position
      if (servos[i].current < servos[i].target) {
        servos[i].current += servos[i].step;
        if (servos[i].current > servos[i].target) servos[i].current = servos[i].target;
      } else if (servos[i].current > servos[i].target) {
        servos[i].current -= servos[i].step;
        if (servos[i].current < servos[i].target) servos[i].current = servos[i].target;
      }

      // Always restrict before sending to servo
      int bounded = safe_constrain(servos[i].current, servos[i].min_angle, servos[i].max_angle);
      servos[i].servo.write(bounded);
    }
  }
}
