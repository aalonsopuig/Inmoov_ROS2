/*
===============================================================================
File:         subsys_1.ino
Version:      1.6.0
Author:       Alejandro Alonso Puig (https://github.com/aalonsopuig) + GPT
Date:         2026-06-08
License:      Apache 2.0
-------------------------------------------------------------------------------
Description:

Hardware validation firmware for InMoov Subsystem 1.

This sketch validates the Arduino Uno shield used in Subsystem 1 of the InMoov
robot. It activates servo power group 1, keeps servo power group 2 disabled, and
moves all group 1 servos to their configured rest positions.

This version does not use ROS 2 or XICRO.

Main functions:

- Activate servo power group 1.
- Keep servo power group 2 OFF.
- Wait after enabling servo power before reading feedback.
- Attach and hold all group 1 servos.
- For servos with feedback, start from measured physical position.
- Move group 1 servos to their configured rest positions.
- Use runtime speed values loaded from servoConfigs[].
- Implement basic overcurrent fault detection.
- Use a pushbutton connected to D7 only to select the servo shown on screen.
- Show selected servo information on an OLED display using U8x8lib.
- Avoid full-screen clearing to reduce OLED flicker.
- Use external 3.3 V ADC reference through AREF.

Important:

- This version uses Servo.h instead of ServoController to fit in Arduino Uno RAM.
- Static servo parameters are read from servoConfigs[] in PROGMEM.
- Runtime state is kept in SmoothServo.
- Group 1: ON  -> THUMB_R, INDEX_R, MIDDLE_R, RING_R, PINKY_R, BICEP_R, ROTATE_R
- Group 2: OFF -> SHOULDER_R, OMOPLATE_R
- Group 2 servos are not driven because their power group is OFF.

Hardware:

- Arduino Uno

- D12: Servo group 1 power control
       Bicep + fingers + rotate
       LOW  -> servos OFF
       HIGH -> servos ON

- D13: Servo group 2 power control
       Shoulder + omoplate
       LOW  -> servos OFF
       HIGH -> servos ON

- D7:  Next-servo pushbutton
       Button connects D7 to GND when pressed
       Internal pull-up is enabled

- A0:  BICEP_R position feedback input, if configured in .h
- A1:  BICEP_R current sense input, if configured in .h

- AREF connected externally to 3.3 V
- OLED SSD1315 / SSD1306-compatible display via I2C

===============================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include <Servo.h>
#include <U8x8lib.h>

#include "servo_config_inmoov.h"

// ============================================================================
// Shield pins
// ============================================================================

#define SERVO_GROUP_1_PIN  12
#define SERVO_GROUP_2_PIN  13
#define NEXT_BUTTON_PIN     7

// ============================================================================
// OLED configuration
// ============================================================================

U8X8_SSD1306_128X64_NONAME_HW_I2C oled(U8X8_PIN_NONE);

#define OLED_COLS 16
#define OLED_ROWS 8

char lastOledRows[OLED_ROWS][OLED_COLS + 1];

// ============================================================================
// ADC and timing configuration
// ============================================================================

#define ADC_SAMPLES              4
#define SERVO_UPDATE_MS         40
#define DISPLAY_UPDATE_MS      250
#define DEBOUNCE_MS             40
#define POWER_SETTLE_MS       2000
#define CURRENT_STARTUP_GRACE 1000

// ============================================================================
// Dynamic servo state
// ============================================================================

struct SmoothServo
{
    Servo servo;

    int current;
    int target;

    int current_speed_pct;
    int current_accel_pct;

    bool fault_active;
    bool first_commanded;
    bool attached;

    unsigned long last_update;
    unsigned long overcurrent_start;
    unsigned long ignore_current_until;
};

SmoothServo servos[NUM_SERVOS];

// ============================================================================
// Global state
// ============================================================================

uint8_t currentServoIndex = 0;
ServoConfig activeCfg;

bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long lastDisplayUpdate = 0;

// ============================================================================
// Utility functions
// ============================================================================

int rounded_int(float value)
{
    if (value >= 0.0f)
    {
        return (int)(value + 0.5f);
    }
    else
    {
        return (int)(value - 0.5f);
    }
}

int safe_constrain(int value, int min_val, int max_val)
{
    return constrain(value, min_val, max_val);
}

float safe_constrain_float(float value, float min_val, float max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

void readServoConfig(uint8_t index, ServoConfig &cfg)
{
    memcpy_P(&cfg, &servoConfigs[index], sizeof(ServoConfig));
}

float readAveragedADC(uint8_t pin, uint8_t samples)
{
    long sum = 0;

    for (uint8_t i = 0; i < samples; i++)
    {
        sum += analogRead(pin);
    }

    return (float)sum / (float)samples;
}

bool hasFeedbackADC(const ServoConfig &cfg)
{
    return cfg.feedback_adc_pin >= 0;
}

bool hasFeedbackCalibration(const ServoConfig &cfg)
{
    return hasFeedbackADC(cfg) &&
           cfg.fb_adc_at_servo_max_deg != cfg.fb_adc_at_servo_min_deg &&
           cfg.servo_max_deg > cfg.servo_min_deg;
}

bool hasCurrentADC(const ServoConfig &cfg)
{
    return cfg.current_adc_pin >= 0;
}

bool hasCurrentCalibration(const ServoConfig &cfg)
{
    return hasCurrentADC(cfg) &&
           cfg.current_mA_per_count > 0.0f;
}

float feedbackAngleFromADC(const ServoConfig &cfg, int adcValue)
{
    if (!hasFeedbackCalibration(cfg))
    {
        return cfg.rest_deg;
    }

    float ratio =
        (float)(adcValue - cfg.fb_adc_at_servo_min_deg) /
        (float)(cfg.fb_adc_at_servo_max_deg - cfg.fb_adc_at_servo_min_deg);

    float angle =
        cfg.servo_min_deg +
        ratio * (cfg.servo_max_deg - cfg.servo_min_deg);

    return safe_constrain_float(angle, cfg.servo_min_deg, cfg.servo_max_deg);
}

float currentMilliAmpsFromADC(const ServoConfig &cfg, int adcValue)
{
    if (!hasCurrentCalibration(cfg))
    {
        return 0.0f;
    }

    float current_mA =
        (adcValue - (float)cfg.current_adc_offset) *
        cfg.current_mA_per_count;

    if (current_mA < 0.0f)
    {
        current_mA = 0.0f;
    }

    return current_mA;
}

int angleToPwmUs(const ServoConfig &cfg, int angleDeg)
{
    int boundedAngle = safe_constrain(
        angleDeg,
        rounded_int(cfg.servo_min_deg),
        rounded_int(cfg.servo_max_deg)
    );

    int physicalAngle = boundedAngle;

    if (cfg.inverted)
    {
        physicalAngle =
            rounded_int(cfg.servo_max_deg) -
            (boundedAngle - rounded_int(cfg.servo_min_deg));
    }

    float spanDeg = cfg.servo_max_deg - cfg.servo_min_deg;

    if (spanDeg <= 0.0f)
    {
        return cfg.pwm_min_us;
    }

    float ratio =
        ((float)physicalAngle - cfg.servo_min_deg) / spanDeg;

    float pwm =
        cfg.pwm_min_us +
        ratio * (cfg.pwm_max_us - cfg.pwm_min_us);

    if (pwm < cfg.pwm_min_us) pwm = cfg.pwm_min_us;
    if (pwm > cfg.pwm_max_us) pwm = cfg.pwm_max_us;

    return rounded_int(pwm);
}

int computeStepDeg(const ServoConfig &cfg, uint8_t servoIndex)
{
    float speed_pct = servos[servoIndex].current_speed_pct;

    if (speed_pct < 1.0f)
    {
        speed_pct = 1.0f;
    }

    if (speed_pct > 100.0f)
    {
        speed_pct = 100.0f;
    }

    float deg_per_second =
        cfg.max_speed_degps * speed_pct / 100.0f;

    float deg_per_update =
        deg_per_second * ((float)SERVO_UPDATE_MS / 1000.0f);

    int step = rounded_int(deg_per_update);

    if (step < 1)
    {
        step = 1;
    }

    return step;
}

int readInitialServoAngle(const ServoConfig &cfg)
{
    int initialDeg;

    if (hasFeedbackADC(cfg) && hasFeedbackCalibration(cfg))
    {
        int fbAdc = rounded_int(readAveragedADC(cfg.feedback_adc_pin, ADC_SAMPLES));
        initialDeg = rounded_int(feedbackAngleFromADC(cfg, fbAdc));
    }
    else
    {
        initialDeg = rounded_int(cfg.rest_deg);
    }

    initialDeg = safe_constrain(
        initialDeg,
        rounded_int(cfg.allowed_min_deg),
        rounded_int(cfg.allowed_max_deg)
    );

    return initialDeg;
}

// ============================================================================
// Power handling
// ============================================================================

bool servoBelongsToGroup1(uint8_t servoIndex)
{
    return servoIndex == THUMB_R  ||
           servoIndex == INDEX_R  ||
           servoIndex == MIDDLE_R ||
           servoIndex == RING_R   ||
           servoIndex == PINKY_R  ||
           servoIndex == BICEP_R  ||
           servoIndex == ROTATE_R;
}

bool servoBelongsToGroup2(uint8_t servoIndex)
{
    return servoIndex == SHOULDER_R ||
           servoIndex == OMOPLATE_R;
}

void setServoPowerGroup1(bool enabled)
{
    digitalWrite(SERVO_GROUP_1_PIN, enabled ? HIGH : LOW);
}

void setServoPowerGroup2(bool enabled)
{
    digitalWrite(SERVO_GROUP_2_PIN, enabled ? HIGH : LOW);
}

void applyPowerState()
{
    setServoPowerGroup1(true);
    setServoPowerGroup2(false);
}

void forceServoPowerOff()
{
    setServoPowerGroup1(false);
    setServoPowerGroup2(false);
}

bool isSelectedServoPowered(uint8_t servoIndex)
{
    if (servoBelongsToGroup1(servoIndex))
    {
        return digitalRead(SERVO_GROUP_1_PIN) == HIGH;
    }

    if (servoBelongsToGroup2(servoIndex))
    {
        return digitalRead(SERVO_GROUP_2_PIN) == HIGH;
    }

    return false;
}

// ============================================================================
// Fault handling
// ============================================================================

void checkServoFault(uint8_t index, const ServoConfig &cfg)
{
    if (!servos[index].attached)
    {
        return;
    }

    if (servos[index].fault_active)
    {
        return;
    }

    if (millis() < servos[index].ignore_current_until)
    {
        servos[index].overcurrent_start = 0;
        return;
    }

    if (!cfg.fault_detection_enabled)
    {
        servos[index].overcurrent_start = 0;
        return;
    }

    if (!hasCurrentADC(cfg) || !hasCurrentCalibration(cfg))
    {
        servos[index].overcurrent_start = 0;
        return;
    }

    int currentAdc = rounded_int(readAveragedADC(cfg.current_adc_pin, ADC_SAMPLES));
    int currentMilliAmps = rounded_int(currentMilliAmpsFromADC(cfg, currentAdc));

    if (currentMilliAmps > rounded_int(cfg.current_limit_mA))
    {
        if (servos[index].overcurrent_start == 0)
        {
            servos[index].overcurrent_start = millis();
        }

        if (millis() - servos[index].overcurrent_start >= cfg.overcurrent_time_ms)
        {
            servos[index].fault_active = true;
            servos[index].servo.detach();
            servos[index].attached = false;
            servos[index].overcurrent_start = 0;
        }
    }
    else
    {
        servos[index].overcurrent_start = 0;
    }
}

// ============================================================================
// Servo state handling
// ============================================================================

void initServoState(uint8_t index)
{
    ServoConfig cfg;
    readServoConfig(index, cfg);

    servos[index].current = rounded_int(cfg.rest_deg);
    servos[index].target = rounded_int(cfg.rest_deg);

    servos[index].current_speed_pct = cfg.default_speed_pct;
    servos[index].current_accel_pct = cfg.default_accel_pct;

    servos[index].fault_active = false;
    servos[index].first_commanded = false;
    servos[index].attached = false;

    servos[index].last_update = millis();
    servos[index].overcurrent_start = 0;
    servos[index].ignore_current_until = 0;
}

void initAllServoStates()
{
    for (uint8_t i = 0; i < NUM_SERVOS; i++)
    {
        initServoState(i);
    }
}

void loadActiveServo()
{
    readServoConfig(currentServoIndex, activeCfg);
}

void attachGroup1ServosFromFeedback()
{
    unsigned long now = millis();

    for (uint8_t i = 0; i < NUM_SERVOS; i++)
    {
        if (!servoBelongsToGroup1(i))
        {
            continue;
        }

        ServoConfig cfg;
        readServoConfig(i, cfg);

        int initialDeg = readInitialServoAngle(cfg);

        servos[i].current = initialDeg;

        servos[i].target = safe_constrain(
            rounded_int(cfg.rest_deg),
            rounded_int(cfg.allowed_min_deg),
            rounded_int(cfg.allowed_max_deg)
        );

        int pwmUs = angleToPwmUs(cfg, servos[i].current);

        /*
          Preload the pulse width before attach().

          This avoids the Servo library starting with its default pulse value
          before the sketch writes the measured current position.
        */
        servos[i].servo.writeMicroseconds(pwmUs);
        servos[i].servo.attach(cfg.pwm_pin, cfg.pwm_min_us, cfg.pwm_max_us);
        servos[i].servo.writeMicroseconds(pwmUs);

        servos[i].attached = true;
        servos[i].fault_active = false;
        servos[i].overcurrent_start = 0;
        servos[i].ignore_current_until = now + CURRENT_STARTUP_GRACE;
        servos[i].last_update = now;
    }
}

void updateGroup1Servos()
{
    unsigned long now = millis();

    for (uint8_t i = 0; i < NUM_SERVOS; i++)
    {
        if (!servoBelongsToGroup1(i))
        {
            continue;
        }

        if (!servos[i].attached)
        {
            continue;
        }

        if (now - servos[i].last_update < SERVO_UPDATE_MS)
        {
            continue;
        }

        servos[i].last_update = now;

        ServoConfig cfg;
        readServoConfig(i, cfg);

        checkServoFault(i, cfg);

        if (servos[i].fault_active)
        {
            continue;
        }

        servos[i].target = safe_constrain(
            rounded_int(cfg.rest_deg),
            rounded_int(cfg.allowed_min_deg),
            rounded_int(cfg.allowed_max_deg)
        );

        int step = computeStepDeg(cfg, i);

        if (servos[i].current < servos[i].target)
        {
            servos[i].current += step;

            if (servos[i].current > servos[i].target)
            {
                servos[i].current = servos[i].target;
            }
        }
        else if (servos[i].current > servos[i].target)
        {
            servos[i].current -= step;

            if (servos[i].current < servos[i].target)
            {
                servos[i].current = servos[i].target;
            }
        }

        servos[i].current = safe_constrain(
            servos[i].current,
            rounded_int(cfg.allowed_min_deg),
            rounded_int(cfg.allowed_max_deg)
        );

        int pwmUs = angleToPwmUs(cfg, servos[i].current);
        servos[i].servo.writeMicroseconds(pwmUs);
    }
}

// ============================================================================
// Button handling
// ============================================================================

bool nextButtonPressedEvent()
{
    bool reading = digitalRead(NEXT_BUTTON_PIN);

    if (reading != lastButtonReading)
    {
        lastDebounceTime = millis();
        lastButtonReading = reading;
    }

    if ((millis() - lastDebounceTime) > DEBOUNCE_MS)
    {
        if (reading != stableButtonState)
        {
            stableButtonState = reading;

            if (stableButtonState == LOW)
            {
                return true;
            }
        }
    }

    return false;
}

void selectNextServo()
{
    currentServoIndex++;

    if (currentServoIndex >= NUM_SERVOS)
    {
        currentServoIndex = 0;
    }

    loadActiveServo();
}

// ============================================================================
// OLED handling
// ============================================================================

void resetOledCache()
{
    for (uint8_t r = 0; r < OLED_ROWS; r++)
    {
        for (uint8_t c = 0; c < OLED_COLS; c++)
        {
            lastOledRows[r][c] = '\0';
        }

        lastOledRows[r][OLED_COLS] = '\0';
    }
}

void printRow(uint8_t row, const char *text)
{
    char buffer[OLED_COLS + 1];

    uint8_t i = 0;

    while (i < OLED_COLS && text[i] != '\0')
    {
        buffer[i] = text[i];
        i++;
    }

    while (i < OLED_COLS)
    {
        buffer[i] = ' ';
        i++;
    }

    buffer[OLED_COLS] = '\0';

    if (strncmp(buffer, lastOledRows[row], OLED_COLS) != 0)
    {
        oled.setCursor(0, row);
        oled.print(buffer);
        strncpy(lastOledRows[row], buffer, OLED_COLS + 1);
    }
}

void oledPrintSplash()
{
    oled.clearDisplay();
    resetOledCache();

    printRow(0, "SUBSYSTEM 1");
    printRow(2, "G1 power ON");
    printRow(3, "Wait feedback");
    printRow(4, "settling...");
    printRow(6, "G2 power OFF");
}

void updateDisplay()
{
    int feedbackDeg = 0;
    int currentMilliAmps = 0;

    bool hasFb = hasFeedbackADC(activeCfg);
    bool hasFbCal = hasFeedbackCalibration(activeCfg);

    bool hasCur = hasCurrentADC(activeCfg);
    bool hasCurCal = hasCurrentCalibration(activeCfg);

    bool selectedServoPowered = isSelectedServoPowered(currentServoIndex);

    if (hasFb && hasFbCal)
    {
        int feedbackAdc =
            rounded_int(readAveragedADC(activeCfg.feedback_adc_pin, ADC_SAMPLES));

        feedbackDeg =
            rounded_int(feedbackAngleFromADC(activeCfg, feedbackAdc));
    }

    if (hasCur && hasCurCal)
    {
        int currentAdc =
            rounded_int(readAveragedADC(activeCfg.current_adc_pin, ADC_SAMPLES));

        currentMilliAmps =
            rounded_int(currentMilliAmpsFromADC(activeCfg, currentAdc));
    }

    char row[OLED_COLS + 1];

    snprintf(row, sizeof(row), "S:%u/%u %.8s",
             currentServoIndex,
             NUM_SERVOS - 1,
             activeCfg.name);
    printRow(0, row);

    snprintf(row, sizeof(row), "PWR:%s OUT:%s",
             selectedServoPowered ? "ON" : "OFF",
             servos[currentServoIndex].attached ? "ON" : "OFF");
    printRow(1, row);

    snprintf(row, sizeof(row), "Lim:%d-%d",
             rounded_int(activeCfg.allowed_min_deg),
             rounded_int(activeCfg.allowed_max_deg));
    printRow(2, row);

    snprintf(row, sizeof(row), "R:%d Set:%d",
             rounded_int(activeCfg.rest_deg),
             servos[currentServoIndex].current);
    printRow(3, row);

    snprintf(row, sizeof(row), "Sp:%d Ac:%d",
             servos[currentServoIndex].current_speed_pct,
             servos[currentServoIndex].current_accel_pct);
    printRow(4, row);

    if (hasFb && hasFbCal && hasCur && hasCurCal)
    {
        snprintf(row, sizeof(row), "FB:%d I:%dmA",
                 feedbackDeg,
                 currentMilliAmps);
    }
    else if (hasFb && hasFbCal)
    {
        snprintf(row, sizeof(row), "FB:%d I:-",
                 feedbackDeg);
    }
    else if (hasCur && hasCurCal)
    {
        snprintf(row, sizeof(row), "FB:- I:%dmA",
                 currentMilliAmps);
    }
    else
    {
        snprintf(row, sizeof(row), "FB:- I:-");
    }

    printRow(5, row);

    snprintf(row, sizeof(row), "FD:%s ST:%s",
             activeCfg.fault_detection_enabled ? "ON" : "OFF",
             servos[currentServoIndex].fault_active ? "FAULT" : "OK");
    printRow(6, row);

    if (servoBelongsToGroup1(currentServoIndex))
    {
        snprintf(row, sizeof(row), "Mode:G1 REST");
    }
    else
    {
        snprintf(row, sizeof(row), "Mode:G2 OFF");
    }

    printRow(7, row);
}

// ============================================================================
// Setup
// ============================================================================

void setup()
{
    pinMode(SERVO_GROUP_1_PIN, OUTPUT);
    pinMode(SERVO_GROUP_2_PIN, OUTPUT);

    forceServoPowerOff();

    pinMode(NEXT_BUTTON_PIN, INPUT_PULLUP);

    lastButtonReading = digitalRead(NEXT_BUTTON_PIN);
    stableButtonState = lastButtonReading;
    lastDebounceTime = millis();

    analogReference(EXTERNAL);

    Wire.begin();

    oled.begin();
    oled.setPowerSave(0);
    oled.setFont(u8x8_font_chroma48medium8_r);

    resetOledCache();

    initAllServoStates();

    currentServoIndex = THUMB_R;
    loadActiveServo();

    /*
      Safe startup sequence:

      1. Keep all servo PWM outputs detached.
      2. Enable group 1 power.
      3. Wait for servo electronics and feedback circuitry to settle.
      4. Read feedback.
      5. Attach servo output using the measured current position.
      6. Move gradually toward rest.
    */

    applyPowerState();
    oledPrintSplash();

    delay(POWER_SETTLE_MS);

    attachGroup1ServosFromFeedback();

    resetOledCache();
    oled.clearDisplay();

    updateDisplay();
    lastDisplayUpdate = millis();
}

// ============================================================================
// Main loop
// ============================================================================

void loop()
{
    applyPowerState();

    updateGroup1Servos();

    if (nextButtonPressedEvent())
    {
        selectNextServo();
        updateDisplay();
    }

    if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_MS)
    {
        lastDisplayUpdate = millis();
        updateDisplay();
    }
}
