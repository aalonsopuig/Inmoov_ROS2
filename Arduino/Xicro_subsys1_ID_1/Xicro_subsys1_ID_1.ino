/*
  File: Xicro_subsys1_ID_1.ino
  Description: This program implements subsystem 1 of the InMoov robot, responsible for controlling the servomotors of the right arm using an Arduino Uno and ROS 2 via the XICRO interface. It enables individual control of each joint, applying safety limits, rest position, and adjustable speed for each servo.
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
  This program controls the right arm servos of the InMoov robot using an Arduino Uno board. It uses the XICRO interface to receive commands from a ROS 2-based central computer, allowing each servo to be assigned a target angle, speed, safety limits, and a rest position.

  Servo movements are interpolated, gradually advancing toward the target to protect the mechanics and avoid abrupt movements. If the received command is zero, the servo automatically moves to its predefined rest position. The system is designed to be safe and flexible, adaptable to various servos, and allows easy recalibration of parameters.

  Usage examples:
    - Sending an angle (e.g., 130) from ROS 2 moves the corresponding servo to that value, limited by its safety range.
    - Sending a 0 to the corresponding topic sends the joint to its defined rest position.
    - State check:
        if (servos[BICEP_R].current == servos[BICEP_R].target) {
          // The right biceps has reached its target position
        }
    - Manual speed and target modification:
        servos[SHOULDER_R].target = 120;
        servos[SHOULDER_R].step = 1;
*/

#include "Xicro_subsys1_ID_1.h"
#include <Servo.h>

#define INTERVAL_MS 60   // Global update interval for all servos (in milliseconds, adjustable)

#define THUMB_R     0
#define INDEX_R     1
#define MIDDLE_R    2
#define RING_R      3
#define PINKY_R     4
#define BICEP_R     5
#define ROTATE_R    6
#define SHOULDER_R  7
#define OMOPLATE_R  8
#define NUM_SERVOS  9

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
  //        Servo()  current target rest_angle  min  max  last_update step pin first_commanded
  {Servo(), 0, 0,   144,    112, 177, 0,        2,   2,  false},   // THUMB_R: Right thumb
  {Servo(), 0, 0,   135,     85, 150, 0,        2,   3,  false},   // INDEX_R: Right index finger
  {Servo(), 0, 0,   135,     70, 140, 0,        2,   4,  false},   // MIDDLE_R: Right middle finger
  {Servo(), 0, 0,   135,     80, 160, 0,        2,   5,  false},   // RING_R: Right ring finger
  {Servo(), 0, 0,   135,     90, 165, 0,        2,   6,  false},   // PINKY_R: Right pinky finger
  {Servo(), 0, 0,    50,     50, 110, 0,        1,   8,  false},   // BICEP_R: Right biceps
  {Servo(), 0, 0,    90,     80, 135, 0,        1,   9,  false},   // ROTATE_R: Right arm rotation
  {Servo(), 0, 0,    90,     90, 120, 0,        1,  10,  false},   // SHOULDER_R: Right shoulder
  {Servo(), 0, 0,     5,      0,  20, 0,        1,  11,  false}    // OMOPLATE_R: Right omoplate
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
    xicro.Subscription_thumb_finger_R.message.data,
    xicro.Subscription_index_finger_R.message.data,
    xicro.Subscription_middle_finger_R.message.data,
    xicro.Subscription_ring_finger_R.message.data,
    xicro.Subscription_pinky_finger_R.message.data,
    xicro.Subscription_bicep_R.message.data,
    xicro.Subscription_rotate_R.message.data,
    xicro.Subscription_shoulder_R.message.data,
    xicro.Subscription_omoplate_R.message.data
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

