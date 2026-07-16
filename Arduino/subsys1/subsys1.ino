/*
===============================================================================
File:         subsys_1.ino
Version:      1.27.0
Author:       Alejandro Alonso Puig (https://github.com/aalonsopuig) + GPT
Date:         2026-06-30
License:      Apache 2.0
-------------------------------------------------------------------------------
Description:

InMoov Subsystem 1 autonomous startup firmware.

Behavior:

- On power-up:
  - D12 and D13 OFF.
  - Initial 2 second stabilization pause.
  - Automatic staged startup to rest position.
  - No button press is required for startup.

- After startup:
  - The button only cycles through servo information screens.
  - No button press commands motion.
  - Servo information is shown cyclically for all group 1 and group 2 servos.

Startup sequence:

1. D12/D13 OFF.
2. D12 ON, with no PWM active.
3. Wait for stabilization.
4. Read BICEP_R feedback.
5. Convert ADC to degrees.
6. Limit BICEP_R measured position to allowed range.
7. D12 OFF.
8. Attach BICEP_R with PWM corresponding to measured safe position.
9. Attach remaining group 1 servos: fingers and rotate.
10. Send rest PWM to all group 1 servos except BICEP_R.
11. D12 ON.
12. Attach group 2 servos.
13. Send rest PWM to all group 2 servos.
14. D13 ON.
15. Move BICEP_R smoothly to rest.

===============================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include <Servo.h>
#include <U8x8lib.h>

#include "servo_config_inmoov.h"

// ============================================================================
// Version
// ============================================================================

#define FW_VERSION "1.27.0"

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
// Timing and ADC
// ============================================================================

#define ADC_SAMPLES             8
#define SERVO_UPDATE_MS        40
#define DISPLAY_UPDATE_MS     250
#define DEBOUNCE_MS            40
#define INITIAL_STABILIZE_MS 2000
#define POWER_SETTLE_MS      2000
#define POWER_OFF_PAUSE_MS    200

// ============================================================================
// Auxiliary servos: all non-BICEP_R servos in subsystem 1
// ============================================================================

#define NUM_AUX_SERVOS 8

const uint8_t auxServoIndex[NUM_AUX_SERVOS] =
{
    THUMB_R,
    INDEX_R,
    MIDDLE_R,
    RING_R,
    PINKY_R,
    ROTATE_R,
    SHOULDER_R,
    OMOPLATE_R
};

#define NUM_DISPLAY_SERVOS 9

const uint8_t displayServoIndex[NUM_DISPLAY_SERVOS] =
{
    THUMB_R,
    INDEX_R,
    MIDDLE_R,
    RING_R,
    PINKY_R,
    BICEP_R,
    ROTATE_R,
    SHOULDER_R,
    OMOPLATE_R
};

struct AuxServoState
{
    Servo servo;
    int current_deg;
    int target_deg;
    int speed_pct;
    bool attached;
    unsigned long last_update;
};

AuxServoState aux[NUM_AUX_SERVOS];

// ============================================================================
// Biceps state
// ============================================================================

enum SystemMode
{
    MODE_STARTING = 0,
    MODE_RUNNING,
    MODE_FAULT
};

struct BicepState
{
    Servo servo;

    int current_deg;
    int target_deg;

    int feedback_adc;
    int feedback_deg_raw;
    int feedback_deg_safe;

    int current_mA;
    int pwm_us;

    int speed_pct;
    int accel_pct;

    bool attached;
    bool fault_active;

    unsigned long last_update;
    unsigned long overcurrent_start;
};

BicepState bicep;
ServoConfig bicepCfg;

SystemMode mode = MODE_STARTING;

// ============================================================================
// Button and display state
// ============================================================================

bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long lastDisplayUpdate = 0;

int selectedDisplayServo = -1;   // -1 = instruction screen

// ============================================================================
// Generic utility functions
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

int safe_constrain_int(int value, int min_val, int max_val)
{
    if (value < min_val)
    {
        return min_val;
    }

    if (value > max_val)
    {
        return max_val;
    }

    return value;
}

void readServoConfig(uint8_t index, ServoConfig &cfg)
{
    memcpy_P(&cfg, &servoConfigs[index], sizeof(ServoConfig));
}

void readBicepConfig()
{
    readServoConfig(BICEP_R, bicepCfg);
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

// ============================================================================
// Servo names and groups
// ============================================================================

const char* servoName(uint8_t servoIndex)
{
    switch (servoIndex)
    {
        case THUMB_R:
            return "THUMB_R";
        case INDEX_R:
            return "INDEX_R";
        case MIDDLE_R:
            return "MIDDLE_R";
        case RING_R:
            return "RING_R";
        case PINKY_R:
            return "PINKY_R";
        case BICEP_R:
            return "BICEP_R";
        case ROTATE_R:
            return "ROTATE_R";
        case SHOULDER_R:
            return "SHOULDER_R";
        case OMOPLATE_R:
            return "OMOPLATE_R";
        default:
            return "UNKNOWN";
    }
}

int servoGroup(uint8_t servoIndex)
{
    if (servoIndex == THUMB_R  ||
        servoIndex == INDEX_R  ||
        servoIndex == MIDDLE_R ||
        servoIndex == RING_R   ||
        servoIndex == PINKY_R  ||
        servoIndex == BICEP_R  ||
        servoIndex == ROTATE_R)
    {
        return 1;
    }

    if (servoIndex == SHOULDER_R ||
        servoIndex == OMOPLATE_R)
    {
        return 2;
    }

    return 0;
}

bool auxServoIsGroup1(uint8_t servoIndex)
{
    return servoIndex == THUMB_R  ||
           servoIndex == INDEX_R  ||
           servoIndex == MIDDLE_R ||
           servoIndex == RING_R   ||
           servoIndex == PINKY_R  ||
           servoIndex == ROTATE_R;
}

bool auxServoIsGroup2(uint8_t servoIndex)
{
    return servoIndex == SHOULDER_R ||
           servoIndex == OMOPLATE_R;
}

int auxArrayIndexFromServoIndex(uint8_t servoIndex)
{
    for (uint8_t i = 0; i < NUM_AUX_SERVOS; i++)
    {
        if (auxServoIndex[i] == servoIndex)
        {
            return i;
        }
    }

    return -1;
}

// ============================================================================
// Servo configuration helpers
// ============================================================================

int cfgAllowedMin(const ServoConfig &cfg)
{
    return rounded_int(cfg.allowed_min_deg);
}

int cfgAllowedMax(const ServoConfig &cfg)
{
    return rounded_int(cfg.allowed_max_deg);
}

int cfgServoMin(const ServoConfig &cfg)
{
    return rounded_int(cfg.servo_min_deg);
}

int cfgServoMax(const ServoConfig &cfg)
{
    return rounded_int(cfg.servo_max_deg);
}

int cfgRestDeg(const ServoConfig &cfg)
{
    return safe_constrain_int(
        rounded_int(cfg.rest_deg),
        cfgAllowedMin(cfg),
        cfgAllowedMax(cfg)
    );
}

int cfgMinDeg(const ServoConfig &cfg)
{
    return cfgAllowedMin(cfg);
}

int cfgMaxDeg(const ServoConfig &cfg)
{
    return cfgAllowedMax(cfg);
}

int cfgConstrainAllowedAngle(const ServoConfig &cfg, int angleDeg)
{
    return safe_constrain_int(
        angleDeg,
        cfgAllowedMin(cfg),
        cfgAllowedMax(cfg)
    );
}

int cfgConstrainServoAngle(const ServoConfig &cfg, int angleDeg)
{
    return safe_constrain_int(
        angleDeg,
        cfgServoMin(cfg),
        cfgServoMax(cfg)
    );
}

int cfgAngleToPwmUs(const ServoConfig &cfg, int angleDeg)
{
    int logicalAngle = cfgConstrainServoAngle(cfg, angleDeg);
    int physicalAngle = logicalAngle;

    if (cfg.inverted)
    {
        physicalAngle =
            cfgServoMax(cfg) -
            (logicalAngle - cfgServoMin(cfg));
    }

    physicalAngle = cfgConstrainServoAngle(cfg, physicalAngle);

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

    if (pwm < cfg.pwm_min_us)
    {
        pwm = cfg.pwm_min_us;
    }

    if (pwm > cfg.pwm_max_us)
    {
        pwm = cfg.pwm_max_us;
    }

    return rounded_int(pwm);
}

int cfgComputeStepDeg(const ServoConfig &cfg, int speedPct)
{
    float speed_pct = speedPct;

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

// ============================================================================
// BICEP_R feedback and current helpers
// ============================================================================

bool bicepHasFeedbackADC()
{
    return bicepCfg.feedback_adc_pin >= 0;
}

bool bicepHasFeedbackCalibration()
{
    return bicepHasFeedbackADC() &&
           bicepCfg.fb_adc_at_servo_max_deg != bicepCfg.fb_adc_at_servo_min_deg &&
           bicepCfg.servo_max_deg > bicepCfg.servo_min_deg;
}

bool bicepHasCurrentADC()
{
    return bicepCfg.current_adc_pin >= 0;
}

bool bicepHasCurrentCalibration()
{
    return bicepHasCurrentADC() &&
           bicepCfg.current_mA_per_count > 0.0f;
}

int bicepFeedbackDegRawFromADC(int adcValue)
{
    if (!bicepHasFeedbackCalibration())
    {
        return cfgRestDeg(bicepCfg);
    }

    float ratio =
        (float)(adcValue - bicepCfg.fb_adc_at_servo_min_deg) /
        (float)(bicepCfg.fb_adc_at_servo_max_deg - bicepCfg.fb_adc_at_servo_min_deg);

    float angle =
        bicepCfg.servo_min_deg +
        ratio * (bicepCfg.servo_max_deg - bicepCfg.servo_min_deg);

    return rounded_int(angle);
}

float bicepCurrentMilliAmpsFromADC(int adcValue)
{
    if (!bicepHasCurrentCalibration())
    {
        return 0.0f;
    }

    float current_mA =
        (adcValue - (float)bicepCfg.current_adc_offset) *
        bicepCfg.current_mA_per_count;

    if (current_mA < 0.0f)
    {
        current_mA = 0.0f;
    }

    return current_mA;
}

int bicepRestDeg()
{
    return cfgRestDeg(bicepCfg);
}

int bicepConstrainAllowedAngle(int angleDeg)
{
    return cfgConstrainAllowedAngle(bicepCfg, angleDeg);
}

int bicepReadCurrentMilliAmps()
{
    if (!bicepHasCurrentADC() || !bicepHasCurrentCalibration())
    {
        return 0;
    }

    int currentAdc =
        rounded_int(readAveragedADC(bicepCfg.current_adc_pin, ADC_SAMPLES));

    return rounded_int(bicepCurrentMilliAmpsFromADC(currentAdc));
}

void updateBicepSensors()
{
    if (digitalRead(SERVO_GROUP_1_PIN) == HIGH)
    {
        if (bicepHasFeedbackADC())
        {
            bicep.feedback_adc =
                rounded_int(readAveragedADC(bicepCfg.feedback_adc_pin, ADC_SAMPLES));

            bicep.feedback_deg_raw =
                bicepFeedbackDegRawFromADC(bicep.feedback_adc);

            bicep.feedback_deg_safe =
                bicepConstrainAllowedAngle(bicep.feedback_deg_raw);
        }
        else
        {
            bicep.feedback_adc = -1;
            bicep.feedback_deg_raw = bicepRestDeg();
            bicep.feedback_deg_safe = bicepRestDeg();
        }

        if (bicep.attached)
        {
            bicep.current_mA = bicepReadCurrentMilliAmps();
        }
        else
        {
            bicep.current_mA = 0;
        }
    }
    else
    {
        bicep.feedback_adc = -1;
        bicep.feedback_deg_raw = bicep.current_deg;
        bicep.feedback_deg_safe = bicep.current_deg;
        bicep.current_mA = 0;
    }
}

// ============================================================================
// Power handling
// ============================================================================

void setGroup1Power(bool enabled)
{
    digitalWrite(SERVO_GROUP_1_PIN, enabled ? HIGH : LOW);
}

void setGroup2Power(bool enabled)
{
    digitalWrite(SERVO_GROUP_2_PIN, enabled ? HIGH : LOW);
}

void allPowerOff()
{
    setGroup1Power(false);
    setGroup2Power(false);
}

// ============================================================================
// Auxiliary servo handling
// ============================================================================

void initAuxServos()
{
    for (uint8_t i = 0; i < NUM_AUX_SERVOS; i++)
    {
        ServoConfig cfg;
        readServoConfig(auxServoIndex[i], cfg);

        aux[i].current_deg = cfgRestDeg(cfg);
        aux[i].target_deg = cfgRestDeg(cfg);
        aux[i].speed_pct = cfg.default_speed_pct;
        aux[i].attached = false;
        aux[i].last_update = millis();
    }
}

void attachAuxServosAtRestForGroup(uint8_t groupNumber)
{
    for (uint8_t i = 0; i < NUM_AUX_SERVOS; i++)
    {
        uint8_t servoIndex = auxServoIndex[i];

        if (groupNumber == 1 && !auxServoIsGroup1(servoIndex))
        {
            continue;
        }

        if (groupNumber == 2 && !auxServoIsGroup2(servoIndex))
        {
            continue;
        }

        ServoConfig cfg;
        readServoConfig(servoIndex, cfg);

        int restDeg = cfgRestDeg(cfg);
        int pwmUs = cfgAngleToPwmUs(cfg, restDeg);

        aux[i].current_deg = restDeg;
        aux[i].target_deg = restDeg;

        if (!aux[i].attached)
        {
            aux[i].servo.writeMicroseconds(pwmUs);
            aux[i].servo.attach(cfg.pwm_pin);
            aux[i].attached = true;
        }

        aux[i].servo.writeMicroseconds(pwmUs);
        aux[i].last_update = millis();
    }
}

void detachAuxServos()
{
    for (uint8_t i = 0; i < NUM_AUX_SERVOS; i++)
    {
        if (aux[i].attached)
        {
            aux[i].servo.detach();
            aux[i].attached = false;
        }
    }
}

void setAuxTargetsRest()
{
    for (uint8_t i = 0; i < NUM_AUX_SERVOS; i++)
    {
        ServoConfig cfg;
        readServoConfig(auxServoIndex[i], cfg);
        aux[i].target_deg = cfgRestDeg(cfg);
    }
}

void updateAuxServos()
{
    unsigned long now = millis();

    for (uint8_t i = 0; i < NUM_AUX_SERVOS; i++)
    {
        if (!aux[i].attached)
        {
            continue;
        }

        if (now - aux[i].last_update < SERVO_UPDATE_MS)
        {
            continue;
        }

        aux[i].last_update = now;

        ServoConfig cfg;
        readServoConfig(auxServoIndex[i], cfg);

        aux[i].target_deg =
            cfgConstrainAllowedAngle(cfg, aux[i].target_deg);

        int step = cfgComputeStepDeg(cfg, aux[i].speed_pct);

        if (aux[i].current_deg < aux[i].target_deg)
        {
            aux[i].current_deg += step;

            if (aux[i].current_deg > aux[i].target_deg)
            {
                aux[i].current_deg = aux[i].target_deg;
            }
        }
        else if (aux[i].current_deg > aux[i].target_deg)
        {
            aux[i].current_deg -= step;

            if (aux[i].current_deg < aux[i].target_deg)
            {
                aux[i].current_deg = aux[i].target_deg;
            }
        }

        aux[i].current_deg =
            cfgConstrainAllowedAngle(cfg, aux[i].current_deg);

        int pwmUs = cfgAngleToPwmUs(cfg, aux[i].current_deg);
        aux[i].servo.writeMicroseconds(pwmUs);
    }
}

// ============================================================================
// BICEP_R PWM and motion
// ============================================================================

void updateBicepPwmCache()
{
    bicep.pwm_us = cfgAngleToPwmUs(bicepCfg, bicep.current_deg);
}

void detachBicepPwm()
{
    if (bicep.attached)
    {
        bicep.servo.detach();
    }

    bicep.attached = false;
}

void attachBicepAtMeasuredSafePosition()
{
    bicep.current_deg = bicep.feedback_deg_safe;
    bicep.target_deg = bicep.current_deg;

    updateBicepPwmCache();

    bicep.servo.writeMicroseconds(bicep.pwm_us);
    bicep.servo.attach(bicepCfg.pwm_pin);
    bicep.servo.writeMicroseconds(bicep.pwm_us);

    bicep.attached = true;
    bicep.fault_active = false;
    bicep.overcurrent_start = 0;
    bicep.last_update = millis();
}

void setBicepTargetRest()
{
    bicep.target_deg = bicepRestDeg();
}

void checkBicepFault()
{
    if (!bicep.attached)
    {
        bicep.overcurrent_start = 0;
        return;
    }

    if (bicep.fault_active)
    {
        return;
    }

    if (!bicepCfg.fault_detection_enabled)
    {
        bicep.overcurrent_start = 0;
        return;
    }

    if (!bicepHasCurrentADC() || !bicepHasCurrentCalibration())
    {
        bicep.overcurrent_start = 0;
        return;
    }

    updateBicepSensors();

    if (bicep.current_mA > rounded_int(bicepCfg.current_limit_mA))
    {
        if (bicep.overcurrent_start == 0)
        {
            bicep.overcurrent_start = millis();
        }

        if (millis() - bicep.overcurrent_start >= bicepCfg.overcurrent_time_ms)
        {
            bicep.fault_active = true;
            detachBicepPwm();
            mode = MODE_FAULT;
        }
    }
    else
    {
        bicep.overcurrent_start = 0;
    }
}

void updateBicepMotion()
{
    if (!bicep.attached)
    {
        updateBicepSensors();
        return;
    }

    checkBicepFault();

    if (bicep.fault_active || mode == MODE_FAULT)
    {
        return;
    }

    unsigned long now = millis();

    if (now - bicep.last_update < SERVO_UPDATE_MS)
    {
        return;
    }

    bicep.last_update = now;

    bicep.target_deg =
        bicepConstrainAllowedAngle(bicep.target_deg);

    int step = cfgComputeStepDeg(bicepCfg, bicep.speed_pct);

    if (bicep.current_deg < bicep.target_deg)
    {
        bicep.current_deg += step;

        if (bicep.current_deg > bicep.target_deg)
        {
            bicep.current_deg = bicep.target_deg;
        }
    }
    else if (bicep.current_deg > bicep.target_deg)
    {
        bicep.current_deg -= step;

        if (bicep.current_deg < bicep.target_deg)
        {
            bicep.current_deg = bicep.target_deg;
        }
    }

    bicep.current_deg =
        bicepConstrainAllowedAngle(bicep.current_deg);

    updateBicepPwmCache();
    bicep.servo.writeMicroseconds(bicep.pwm_us);
}

// ============================================================================
// Autonomous startup
// ============================================================================

void performStagedStartupToRest()
{
    detachBicepPwm();
    detachAuxServos();

    allPowerOff();

    /*
      Phase 1:
      Power group 1 with no PWM active, only to read BICEP_R feedback.
      No Servo object is attached at this point.
    */
    setGroup1Power(true);

    oled.clearDisplay();
    resetOledCache();
    printRow(0, "SUBSYS1 1.27");
    printRow(1, "AUTO START");
    printRow(2, "D12 ON");
    printRow(3, "NO PWM");
    printRow(4, "READ BICEP FB");
    printRow(6, "WAIT 2 sec");

    delay(POWER_SETTLE_MS);

    updateBicepSensors();

    /*
      Phase 2:
      Switch D12 OFF before preparing PWM lines.
    */
    setGroup1Power(false);
    delay(POWER_OFF_PAUSE_MS);

    /*
      Phase 3:
      Prepare BICEP_R PWM from measured safe feedback position while D12 is OFF.
    */
    attachBicepAtMeasuredSafePosition();

    /*
      Phase 4:
      Prepare rest PWM for remaining group 1 servos while D12 is still OFF.
    */
    attachAuxServosAtRestForGroup(1);

    /*
      Phase 5:
      Re-enable group 1. BICEP_R starts from measured safe position.
      Then command BICEP_R to rest.
    */
    setGroup1Power(true);
    setBicepTargetRest();

    /*
      Phase 6:
      Prepare rest PWM for group 2 while D13 is OFF.
    */
    attachAuxServosAtRestForGroup(2);

    /*
      Phase 7:
      Enable group 2.
    */
    setGroup2Power(true);

    mode = MODE_RUNNING;
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

void showIntroScreen()
{
    resetOledCache();
    oled.clearDisplay();

    printRow(0, "SUBSYS1 1.27");
    printRow(1, "InMoov Paul");
    printRow(3, "Startup done");
    printRow(5, "Press button");
    printRow(6, "to view servo");
    printRow(7, "information");
}

void updateServoInfoScreen()
{
    if (selectedDisplayServo < 0)
    {
        return;
    }

    uint8_t servoIndex = displayServoIndex[selectedDisplayServo];

    ServoConfig cfg;
    readServoConfig(servoIndex, cfg);

    char row[OLED_COLS + 1];

    snprintf(row, sizeof(row), "SUBSYS1 1.27");
    printRow(0, row);

    snprintf(row, sizeof(row), "%d/%d %s",
             selectedDisplayServo + 1,
             NUM_DISPLAY_SERVOS,
             servoName(servoIndex));
    printRow(1, row);

    snprintf(row, sizeof(row), "Group:%d Pin:%d",
             servoGroup(servoIndex),
             cfg.pwm_pin);
    printRow(2, row);

    snprintf(row, sizeof(row), "Min:%d Max:%d",
             cfgAllowedMin(cfg),
             cfgAllowedMax(cfg));
    printRow(3, row);

    snprintf(row, sizeof(row), "Rest:%d Sp:%d",
             cfgRestDeg(cfg),
             cfg.default_speed_pct);
    printRow(4, row);

    if (servoIndex == BICEP_R)
    {
        updateBicepSensors();
        updateBicepPwmCache();

        snprintf(row, sizeof(row), "Set:%d T:%d",
                 bicep.current_deg,
                 bicep.target_deg);
        printRow(5, row);

        snprintf(row, sizeof(row), "ADC:%d D:%d",
                 bicep.feedback_adc,
                 bicep.feedback_deg_raw);
        printRow(6, row);

        snprintf(row, sizeof(row), "I:%dmA %s",
                 bicep.current_mA,
                 bicep.fault_active ? "FLT" : "OK");
        printRow(7, row);
    }
    else
    {
        int auxIdx = auxArrayIndexFromServoIndex(servoIndex);
        int currentDeg = 0;
        int targetDeg = 0;
        bool attached = false;

        if (auxIdx >= 0)
        {
            currentDeg = aux[auxIdx].current_deg;
            targetDeg = aux[auxIdx].target_deg;
            attached = aux[auxIdx].attached;
        }

        int pwmUs = cfgAngleToPwmUs(cfg, currentDeg);

        snprintf(row, sizeof(row), "Set:%d T:%d",
                 currentDeg,
                 targetDeg);
        printRow(5, row);

        snprintf(row, sizeof(row), "PWM:%d", pwmUs);
        printRow(6, row);

        snprintf(row, sizeof(row), "%s %s",
                 attached ? "PWM:ON" : "PWM:OFF",
                 mode == MODE_FAULT ? "FAULT" : "OK");
        printRow(7, row);
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

void handleButtonPress()
{
    selectedDisplayServo++;

    if (selectedDisplayServo >= NUM_DISPLAY_SERVOS)
    {
        selectedDisplayServo = 0;
    }

    resetOledCache();
    oled.clearDisplay();
    updateServoInfoScreen();
}

// ============================================================================
// Setup
// ============================================================================

void setup()
{
    pinMode(SERVO_GROUP_1_PIN, OUTPUT);
    pinMode(SERVO_GROUP_2_PIN, OUTPUT);

    allPowerOff();

    pinMode(NEXT_BUTTON_PIN, INPUT_PULLUP);

    lastButtonReading = digitalRead(NEXT_BUTTON_PIN);
    stableButtonState = lastButtonReading;
    lastDebounceTime = millis();

    /*
      AREF is externally connected to 3.3 V.
      This must be configured before analogRead().
    */
    analogReference(EXTERNAL);

    readBicepConfig();

    bicep.current_deg = bicepRestDeg();
    bicep.target_deg = bicepRestDeg();

    bicep.feedback_adc = -1;
    bicep.feedback_deg_raw = bicep.current_deg;
    bicep.feedback_deg_safe = bicep.current_deg;

    bicep.current_mA = 0;
    bicep.speed_pct = bicepCfg.default_speed_pct;
    bicep.accel_pct = bicepCfg.default_accel_pct;
    bicep.attached = false;
    bicep.fault_active = false;
    bicep.last_update = millis();
    bicep.overcurrent_start = 0;
    bicep.pwm_us = 0;

    initAuxServos();

    Wire.begin();

    oled.begin();
    oled.setPowerSave(0);
    oled.setFont(u8x8_font_chroma48medium8_r);

    resetOledCache();
    oled.clearDisplay();

    printRow(0, "SUBSYS1 1.27");
    printRow(1, "Booting...");
    printRow(3, "Please wait");
    printRow(5, "Stabilizing");
    printRow(6, "system...");

    delay(INITIAL_STABILIZE_MS);

    performStagedStartupToRest();

    showIntroScreen();

    lastDisplayUpdate = millis();
}

// ============================================================================
// Main loop
// ============================================================================

void loop()
{
    updateAuxServos();
    updateBicepMotion();

    if (nextButtonPressedEvent())
    {
        handleButtonPress();
    }

    if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_MS)
    {
        lastDisplayUpdate = millis();

        if (selectedDisplayServo >= 0)
        {
            updateServoInfoScreen();
        }
    }
}
