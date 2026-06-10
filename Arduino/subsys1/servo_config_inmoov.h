/*
===============================================================================
File:         servo_config_inmoov.h
Version:      1.1.0
Author:       Alejandro Alonso Puig (https://github.com/aalonsopuig) + GPT
Date:         2026-06-10
License:      Apache 2.0
-------------------------------------------------------------------------------
Description:

Servo configuration file for InMoov Subsystem 1.

This file contains the static configuration parameters for the right arm
subsystem of the InMoov robot. It is intended to be used both by intermediate
hardware validation sketches and by the future XICRO-based firmware version.

The configuration table defines the fixed parameters of each servo:

- Servo name
- PWM output pin
- Full servo angular range
- Mechanically allowed angular range
- Rest position
- PWM pulse limits
- Maximum speed
- Default speed and acceleration percentages
- Optional feedback potentiometer input
- Optional current-sensing input
- Optional overcurrent protection parameters
- Servo inversion
- Fault-detection enable flag

The runtime sketch should keep only dynamic execution state in its SmoothServo
array. Parameters such as pin, limits, rest angle, speed, feedback and current
calibration must be read from this configuration table.

Hardware target:

- Board: Arduino Uno / ATmega328P
- Subsystem: InMoov right arm

Digital pin assignment:

- D2:  THUMB_R
- D3:  INDEX_R
- D4:  MIDDLE_R
- D5:  RING_R
- D6:  PINKY_R
- D8:  BICEP_R
- D9:  ROTATE_R
- D10: SHOULDER_R
- D11: OMOPLATE_R

Analog inputs:

- A0: BICEP_R position feedback potentiometer
- A1: BICEP_R current sensing

Notes:

- AREF is externally connected to 3.3 V in the Subsystem 1 shield.
- The main sketch must call analogReference(EXTERNAL) before using analogRead().
- The configuration table is stored in PROGMEM to reduce SRAM usage.
- D0 and D1 are not used for servos because they are reserved for serial
  communication.
- Avoid integer literals with leading zeroes, such as 070 or 055, because C/C++
  interprets them as octal values.

===============================================================================
*/

#ifndef SERVO_CONFIG_INMOOV_H
#define SERVO_CONFIG_INMOOV_H

#include <Arduino.h>
#include <avr/pgmspace.h>
#include "ServoController.h"

// ============================================================================
// Servo indexes
// ============================================================================
//
// These indexes preserve the order used by the previous XICRO implementation.
// The same order should be kept in the future XICRO subscription array.
// ============================================================================

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

// ============================================================================
// Digital pin assignment - Arduino Uno
// ============================================================================

const uint8_t PIN_THUMB_R     = 2;
const uint8_t PIN_INDEX_R     = 3;
const uint8_t PIN_MIDDLE_R    = 4;
const uint8_t PIN_RING_R      = 5;
const uint8_t PIN_PINKY_R     = 6;
const uint8_t PIN_BICEP_R     = 8;
const uint8_t PIN_ROTATE_R    = 9;
const uint8_t PIN_SHOULDER_R  = 10;
const uint8_t PIN_OMOPLATE_R  = 11;

// ============================================================================
// Analog pin assignment - Arduino Uno
// ============================================================================

const int PIN_BICEP_POSITION_ADC = A0;
const int PIN_BICEP_CURRENT_ADC  = A1;

// ============================================================================
// Servo configuration table
// ============================================================================
//
// Field order:
//
// name
//   Text identifier of the servo. Used mainly for OLED/debug display.
//
// pwm_pin
//   Arduino digital pin connected to the servo PWM signal.
//
// servo_min_deg
//   Minimum logical angle of the servo model.
//   Usually 0 degrees. Used for angle-to-PWM conversion and feedback mapping.
//
// servo_max_deg
//   Maximum logical angle of the servo model.
//   Usually 180 degrees. Used for angle-to-PWM conversion and feedback mapping.
//
// allowed_min_deg
//   Minimum mechanically safe angle for this joint.
//   Commands and startup positions are constrained to this limit.
//
// allowed_max_deg
//   Maximum mechanically safe angle for this joint.
//   Commands and startup positions are constrained to this limit.
//
// rest_deg
//   Safe rest angle for the joint.
//   In the current validation firmware, this is the default target position.
//
// pwm_min_us
//   PWM pulse width, in microseconds, corresponding to servo_min_deg.
//   Used by Servo.attach(pin, min, max) and by manual angle-to-PWM conversion.
//
// pwm_max_us
//   PWM pulse width, in microseconds, corresponding to servo_max_deg.
//   Used by Servo.attach(pin, min, max) and by manual angle-to-PWM conversion.
//
// max_speed_degps
//   Maximum angular speed assumed for this servo, in degrees per second.
//   Runtime speed is calculated as:
//     effective_speed = max_speed_degps * current_speed_pct / 100
//
// default_speed_pct
//   Default runtime speed percentage loaded at startup.
//   Valid practical range: 0 to 100.
//   In the current firmware it is constrained internally to 1..100 when moving.
//
// default_accel_pct
//   Default runtime acceleration percentage loaded at startup.
//   Currently stored and displayed, but not used for trajectory calculation.
//   Kept for compatibility with future versions.
//
// feedback_adc_pin
//   Analog input pin used to read position feedback.
//   Use A0, A1, etc. Use -1 if the servo has no feedback.
//
// fb_adc_at_servo_min_deg
//   ADC value measured when the servo is at servo_min_deg.
//   Used to convert feedback ADC readings into degrees.
//
// fb_adc_at_servo_max_deg
//   ADC value measured when the servo is at servo_max_deg.
//   Used to convert feedback ADC readings into degrees.
//
// current_adc_pin
//   Analog input pin used to read servo current.
//   Use A0, A1, etc. Use -1 if the servo has no current sensing.
//
// current_limit_mA
//   Current limit in milliamps.
//   If fault detection is enabled and current stays above this value for longer
//   than overcurrent_time_ms, the servo enters fault state.
//
// overcurrent_time_ms
//   Time in milliseconds that overcurrent must persist before declaring a fault.
//
// current_adc_offset
//   ADC offset of the current-sensing circuit at zero current.
//   Used in:
//     current_mA = (adc - current_adc_offset) * current_mA_per_count
//
// current_mA_per_count
//   Current conversion factor in milliamps per ADC count.
//   Used together with current_adc_offset to estimate current.
//
// inverted
//   If true, reverses the logical angle-to-PWM relationship.
//   Useful when the servo is mechanically mounted in the opposite direction.
//
// fault_detection_enabled
//   Enables or disables runtime fault detection for this servo.
//   Requires a valid current_adc_pin and current calibration parameters.
//
// ============================================================================

const ServoConfig servoConfigs[NUM_SERVOS] PROGMEM =
{
    {
        "THUMB_R",              // name
        PIN_THUMB_R,            // pwm_pin
        0,                      // servo_min_deg
        180,                    // servo_max_deg
        106,                    // allowed_min_deg
        180,                    // allowed_max_deg
        180,                    // rest_deg
        578,                    // pwm_min_us
        2300,                   // pwm_max_us
        315.8f,                 // max_speed_degps
        50,                     // default_speed_pct
        100,                    // default_accel_pct
        -1,                     // feedback_adc_pin disabled
        0,                      // fb_adc_at_servo_min_deg
        0,                      // fb_adc_at_servo_max_deg
        -1,                     // current_adc_pin disabled
        0,                      // current_limit_mA
        0,                      // overcurrent_time_ms
        0,                      // current_adc_offset
        0.0f,                   // current_mA_per_count
        false,                  // inverted
        false                   // fault_detection_enabled
    },
    {
        "INDEX_R",              // name
        PIN_INDEX_R,            // pwm_pin
        0,                      // servo_min_deg
        180,                    // servo_max_deg
        80,                     // allowed_min_deg
        160,                    // allowed_max_deg
        150,                    // rest_deg
        578,                    // pwm_min_us
        2300,                   // pwm_max_us
        315.8f,                 // max_speed_degps
        50,                     // default_speed_pct
        100,                    // default_accel_pct
        -1,                     // feedback_adc_pin disabled
        0,                      // fb_adc_at_servo_min_deg
        0,                      // fb_adc_at_servo_max_deg
        -1,                     // current_adc_pin disabled
        0,                      // current_limit_mA
        0,                      // overcurrent_time_ms
        0,                      // current_adc_offset
        0.0f,                   // current_mA_per_count
        false,                  // inverted
        false                   // fault_detection_enabled
    },
    {
        "MIDDLE_R",             // name
        PIN_MIDDLE_R,           // pwm_pin
        0,                      // servo_min_deg
        180,                    // servo_max_deg
        65,                     // allowed_min_deg
        166,                    // allowed_max_deg
        150,                    // rest_deg
        578,                    // pwm_min_us
        2300,                   // pwm_max_us
        315.8f,                 // max_speed_degps
        50,                     // default_speed_pct
        100,                    // default_accel_pct
        -1,                     // feedback_adc_pin disabled
        0,                      // fb_adc_at_servo_min_deg
        0,                      // fb_adc_at_servo_max_deg
        -1,                     // current_adc_pin disabled
        0,                      // current_limit_mA
        0,                      // overcurrent_time_ms
        0,                      // current_adc_offset
        0.0f,                   // current_mA_per_count
        false,                  // inverted
        false                   // fault_detection_enabled
    },
    {
        "RING_R",               // name
        PIN_RING_R,             // pwm_pin
        0,                      // servo_min_deg
        180,                    // servo_max_deg
        70,                     // allowed_min_deg
        162,                    // allowed_max_deg
        160,                    // rest_deg
        578,                    // pwm_min_us
        2300,                   // pwm_max_us
        315.8f,                 // max_speed_degps
        50,                     // default_speed_pct
        100,                    // default_accel_pct
        -1,                     // feedback_adc_pin disabled
        0,                      // fb_adc_at_servo_min_deg
        0,                      // fb_adc_at_servo_max_deg
        -1,                     // current_adc_pin disabled
        0,                      // current_limit_mA
        0,                      // overcurrent_time_ms
        0,                      // current_adc_offset
        0.0f,                   // current_mA_per_count
        false,                  // inverted
        false                   // fault_detection_enabled
    },
    {
        "PINKY_R",              // name
        PIN_PINKY_R,            // pwm_pin
        0,                      // servo_min_deg
        180,                    // servo_max_deg
        95,                     // allowed_min_deg
        168,                    // allowed_max_deg
        155,                    // rest_deg
        578,                    // pwm_min_us
        2300,                   // pwm_max_us
        315.8f,                 // max_speed_degps
        50,                     // default_speed_pct
        100,                    // default_accel_pct
        -1,                     // feedback_adc_pin disabled
        0,                      // fb_adc_at_servo_min_deg
        0,                      // fb_adc_at_servo_max_deg
        -1,                     // current_adc_pin disabled
        0,                      // current_limit_mA
        0,                      // overcurrent_time_ms
        0,                      // current_adc_offset
        0.0f,                   // current_mA_per_count
        false,                  // inverted
        false                   // fault_detection_enabled
    },
    {
        "BICEP_R",              // name
        PIN_BICEP_R,            // pwm_pin
        0,                      // servo_min_deg
        180,                    // servo_max_deg
        16,                     // allowed_min_deg
        95,                     // allowed_max_deg
        20,                     // rest_deg
        677,                    // pwm_min_us
        2350,                   // pwm_max_us
        35.0f,                  // max_speed_degps
        40,                     // default_speed_pct
        100,                    // default_accel_pct
        PIN_BICEP_POSITION_ADC, // feedback_adc_pin
        148,                    // fb_adc_at_servo_min_deg
        568,                    // fb_adc_at_servo_max_deg
        PIN_BICEP_CURRENT_ADC,  // current_adc_pin
        1100,                   // current_limit_mA
        500,                    // overcurrent_time_ms
        499,                    // current_adc_offset
        12.2f,                  // current_mA_per_count
        false,                  // inverted
        true                    // fault_detection_enabled
    },
    {
        "ROTATE_R",             // name
        PIN_ROTATE_R,           // pwm_pin
        0,                      // servo_min_deg
        180,                    // servo_max_deg
        60,                     // allowed_min_deg
        145,                    // allowed_max_deg
        105,                    // rest_deg
        677,                    // pwm_min_us
        2350,                   // pwm_max_us
        20.0f,                  // max_speed_degps
        50,                     // default_speed_pct
        100,                    // default_accel_pct
        -1,                     // feedback_adc_pin disabled
        0,                      // fb_adc_at_servo_min_deg
        0,                      // fb_adc_at_servo_max_deg
        -1,                     // current_adc_pin disabled
        0,                      // current_limit_mA
        0,                      // overcurrent_time_ms
        0,                      // current_adc_offset
        0.0f,                   // current_mA_per_count
        false,                  // inverted
        false                   // fault_detection_enabled
    },
    {
        "SHOULDER_R",           // name
        PIN_SHOULDER_R,         // pwm_pin
        0,                      // servo_min_deg
        180,                    // servo_max_deg
        45,                     // allowed_min_deg
        150,                    // allowed_max_deg
        70,                     // rest_deg
        677,                    // pwm_min_us
        2350,                   // pwm_max_us
        20.0f,                  // max_speed_degps
        10,                     // default_speed_pct
        100,                    // default_accel_pct
        -1,                     // feedback_adc_pin disabled
        0,                      // fb_adc_at_servo_min_deg
        0,                      // fb_adc_at_servo_max_deg
        -1,                     // current_adc_pin disabled
        0,                      // current_limit_mA
        0,                      // overcurrent_time_ms
        0,                      // current_adc_offset
        0.0f,                   // current_mA_per_count
        false,                  // inverted
        false                   // fault_detection_enabled
    },
    {
        "OMOPLATE_R",           // name
        PIN_OMOPLATE_R,         // pwm_pin
        0,                      // servo_min_deg
        180,                    // servo_max_deg
        0,                      // allowed_min_deg
        38,                     // allowed_max_deg
        0,                      // rest_deg
        677,                    // pwm_min_us
        2350,                   // pwm_max_us
        35.0f,                  // max_speed_degps
        10,                     // default_speed_pct
        100,                    // default_accel_pct
        -1,                     // feedback_adc_pin disabled
        0,                      // fb_adc_at_servo_min_deg
        0,                      // fb_adc_at_servo_max_deg
        -1,                     // current_adc_pin disabled
        0,                      // current_limit_mA
        0,                      // overcurrent_time_ms
        0,                      // current_adc_offset
        0.0f,                   // current_mA_per_count
        false,                  // inverted
        false                   // fault_detection_enabled
    }
};

#endif
