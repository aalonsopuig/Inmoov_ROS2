/*
===============================================================================
File:         subsys_1.ino
Version:      1.15.0
Author:       Alejandro Alonso Puig (https://github.com/aalonsopuig) + GPT
Date:         2026-06-10
License:      Apache 2.0
-------------------------------------------------------------------------------
Description:

Safe full Subsystem 1 validation firmware for InMoov.

Controlled servos:

Group 1, D12:
- THUMB_R
- INDEX_R
- MIDDLE_R
- RING_R
- PINKY_R
- BICEP_R
- ROTATE_R

Group 2, D13:
- SHOULDER_R
- OMOPLATE_R

Required sequence:

- Startup:
  D12 OFF, D13 OFF, no PWM.

- Button press 1:
  Generate PWM at rest for all non-feedback servos while D12 and D13 are still OFF:
  fingers, ROTATE_R, SHOULDER_R and OMOPLATE_R.
  Enable D12 and D13.
  Wait 2 seconds.
  Read BICEP_R feedback.
  Use measured BICEP_R position as current position.
  Attach BICEP_R PWM from that measured position.
  Move BICEP_R to rest.
  Result: all servos go to rest.

- Button press 2:
  Move all servos to maximum allowed position.

- Button press 3:
  Move all servos to minimum allowed position.

- Button press 4:
  Move all servos to rest.
  When all reach rest, detach PWM and switch D12 and D13 OFF.

Safety:

- BICEP_R is never attached before feedback has been read.
- BICEP_R always starts from measured feedback position.
- BICEP_R current fault detection is active whenever BICEP_R PWM is active.
- All commanded angles are constrained to the configured allowed limits.
- Non-feedback servos receive a valid rest PWM before their power groups are enabled.

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
// Timing and ADC
// ============================================================================

#define ADC_SAMPLES         8
#define SERVO_UPDATE_MS    40
#define DISPLAY_UPDATE_MS 250
#define DEBOUNCE_MS        40
#define POWER_SETTLE_MS  2000

// ============================================================================
// Auxiliary servos: all non-feedback servos in subsystem 1
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

enum TestMode
{
    MODE_POWER_OFF = 0,
    MODE_TO_REST,
    MODE_TO_MAX,
    MODE_TO_MIN,
    MODE_REST_THEN_OFF,
    MODE_FAULT
};

struct BicepState
{
    Servo servo;

    int current_deg;
    int target_deg;
    int feedback_deg;
    int current_mA;

    int speed_pct;
    int accel_pct;

    bool attached;
    bool fault_active;

    unsigned long last_update;
    unsigned long overcurrent_start;
};

BicepState bicep;
ServoConfig bicepCfg;

TestMode mode = MODE_POWER_OFF;

// ============================================================================
// Button and display state
// ============================================================================

bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long lastDisplayUpdate = 0;

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
// Generic servo configuration helpers
// ============================================================================

int cfgAllowedMin(const ServoConfig &cfg)
{
    return rounded_int(cfg.allowed_min_deg);
}

int cfgAllowedMax(const ServoConfig &cfg)
{
    return rounded_int(cfg.allowed_max_deg);
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

int cfgConstrainAngle(const ServoConfig &cfg, int angleDeg)
{
    return safe_constrain_int(
        angleDeg,
        cfgAllowedMin(cfg),
        cfgAllowedMax(cfg)
    );
}

int cfgAngleToPwmUs(const ServoConfig &cfg, int angleDeg)
{
    int boundedAngle = cfgConstrainAngle(cfg, angleDeg);
    int physicalAngle = boundedAngle;

    if (cfg.inverted)
    {
        physicalAngle =
            rounded_int(cfg.servo_max_deg) -
            (boundedAngle - rounded_int(cfg.servo_min_deg));
    }

    physicalAngle = safe_constrain_int(
        physicalAngle,
        rounded_int(cfg.servo_min_deg),
        rounded_int(cfg.servo_max_deg)
    );

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

float bicepFeedbackAngleFromADC(int adcValue)
{
    if (!bicepHasFeedbackCalibration())
    {
        return bicepCfg.rest_deg;
    }

    float ratio =
        (float)(adcValue - bicepCfg.fb_adc_at_servo_min_deg) /
        (float)(bicepCfg.fb_adc_at_servo_max_deg - bicepCfg.fb_adc_at_servo_min_deg);

    float angle =
        bicepCfg.servo_min_deg +
        ratio * (bicepCfg.servo_max_deg - bicepCfg.servo_min_deg);

    return safe_constrain_float(angle, bicepCfg.servo_min_deg, bicepCfg.servo_max_deg);
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

int bicepMinDeg()
{
    return cfgMinDeg(bicepCfg);
}

int bicepMaxDeg()
{
    return cfgMaxDeg(bicepCfg);
}

int bicepConstrainAngle(int angleDeg)
{
    return cfgConstrainAngle(bicepCfg, angleDeg);
}

int bicepReadFeedbackDeg()
{
    int measuredDeg;

    if (bicepHasFeedbackADC() && bicepHasFeedbackCalibration())
    {
        int fbAdc = rounded_int(readAveragedADC(bicepCfg.feedback_adc_pin, ADC_SAMPLES));
        measuredDeg = rounded_int(bicepFeedbackAngleFromADC(fbAdc));
    }
    else
    {
        measuredDeg = bicepRestDeg();
    }

    return bicepConstrainAngle(measuredDeg);
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
        bicep.feedback_deg = bicepReadFeedbackDeg();
        bicep.current_mA = bicepReadCurrentMilliAmps();
    }
    else
    {
        bicep.feedback_deg = bicep.current_deg;
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

bool auxServoBelongsToGroup1(uint8_t servoIndex)
{
    return servoIndex == THUMB_R  ||
           servoIndex == INDEX_R  ||
           servoIndex == MIDDLE_R ||
           servoIndex == RING_R   ||
           servoIndex == PINKY_R  ||
           servoIndex == ROTATE_R;
}

bool auxServoBelongsToGroup2(uint8_t servoIndex)
{
    return servoIndex == SHOULDER_R ||
           servoIndex == OMOPLATE_R;
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

void attachAuxServosAtRestBeforePower()
{
    for (uint8_t i = 0; i < NUM_AUX_SERVOS; i++)
    {
        ServoConfig cfg;
        readServoConfig(auxServoIndex[i], cfg);

        int restDeg = cfgRestDeg(cfg);
        int pwmUs = cfgAngleToPwmUs(cfg, restDeg);

        aux[i].current_deg = restDeg;
        aux[i].target_deg = restDeg;

        if (!aux[i].attached)
        {
            aux[i].servo.writeMicroseconds(pwmUs);
            aux[i].servo.attach(cfg.pwm_pin, cfg.pwm_min_us, cfg.pwm_max_us);
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

void setAuxTargetsMin()
{
    for (uint8_t i = 0; i < NUM_AUX_SERVOS; i++)
    {
        ServoConfig cfg;
        readServoConfig(auxServoIndex[i], cfg);
        aux[i].target_deg = cfgMinDeg(cfg);
    }
}

void setAuxTargetsMax()
{
    for (uint8_t i = 0; i < NUM_AUX_SERVOS; i++)
    {
        ServoConfig cfg;
        readServoConfig(auxServoIndex[i], cfg);
        aux[i].target_deg = cfgMaxDeg(cfg);
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

        aux[i].target_deg = cfgConstrainAngle(cfg, aux[i].target_deg);

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

        aux[i].current_deg = cfgConstrainAngle(cfg, aux[i].current_deg);

        int pwmUs = cfgAngleToPwmUs(cfg, aux[i].current_deg);
        aux[i].servo.writeMicroseconds(pwmUs);
    }
}

bool auxServosAtTarget()
{
    for (uint8_t i = 0; i < NUM_AUX_SERVOS; i++)
    {
        if (aux[i].current_deg != aux[i].target_deg)
        {
            return false;
        }
    }

    return true;
}

// ============================================================================
// BICEP_R PWM and motion
// ============================================================================

void detachBicepPwm()
{
    if (bicep.attached)
    {
        bicep.servo.detach();
    }

    bicep.attached = false;
}

void attachBicepFromMeasuredPosition()
{
    updateBicepSensors();

    bicep.current_deg = bicepConstrainAngle(bicep.feedback_deg);
    bicep.target_deg = bicep.current_deg;

    int pwmUs = cfgAngleToPwmUs(bicepCfg, bicep.current_deg);

    /*
      Critical BICEP_R safety rule:
      PWM is attached only after feedback has been read.
      The first commanded pulse corresponds to the measured current position.
    */
    bicep.servo.writeMicroseconds(pwmUs);
    bicep.servo.attach(
        bicepCfg.pwm_pin,
        bicepCfg.pwm_min_us,
        bicepCfg.pwm_max_us
    );
    bicep.servo.writeMicroseconds(pwmUs);

    bicep.attached = true;
    bicep.fault_active = false;
    bicep.overcurrent_start = 0;
    bicep.last_update = millis();
}

void setBicepTargetRest()
{
    bicep.target_deg = bicepRestDeg();
}

void setBicepTargetMin()
{
    bicep.target_deg = bicepMinDeg();
}

void setBicepTargetMax()
{
    bicep.target_deg = bicepMaxDeg();
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

    bicep.target_deg = bicepConstrainAngle(bicep.target_deg);

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

    bicep.current_deg = bicepConstrainAngle(bicep.current_deg);

    int pwmUs = cfgAngleToPwmUs(bicepCfg, bicep.current_deg);
    bicep.servo.writeMicroseconds(pwmUs);
}

bool bicepAtTarget()
{
    return bicep.current_deg == bicep.target_deg;
}

// ============================================================================
// Shutdown
// ============================================================================

void shutdownToPowerOff()
{
    detachBicepPwm();
    detachAuxServos();

    allPowerOff();

    bicep.current_deg = bicepRestDeg();
    bicep.target_deg = bicepRestDeg();
    bicep.feedback_deg = bicep.current_deg;
    bicep.current_mA = 0;
    bicep.fault_active = false;
    bicep.overcurrent_start = 0;

    initAuxServos();

    mode = MODE_POWER_OFF;
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

const char* modeText()
{
    switch (mode)
    {
        case MODE_POWER_OFF:
            return "PWR OFF";
        case MODE_TO_REST:
            return "TO REST";
        case MODE_TO_MAX:
            return "TO MAX";
        case MODE_TO_MIN:
            return "TO MIN";
        case MODE_REST_THEN_OFF:
            return "REST+OFF";
        case MODE_FAULT:
            return "FAULT";
        default:
            return "?";
    }
}

void updateDisplay()
{
    updateBicepSensors();

    char row[OLED_COLS + 1];

    snprintf(row, sizeof(row), "SUBSYS1 SAFE");
    printRow(0, row);

    snprintf(row, sizeof(row), "%s", modeText());
    printRow(1, row);

    snprintf(row, sizeof(row), "G1:%s G2:%s",
             digitalRead(SERVO_GROUP_1_PIN) ? "ON" : "OFF",
             digitalRead(SERVO_GROUP_2_PIN) ? "ON" : "OFF");
    printRow(2, row);

    snprintf(row, sizeof(row), "FB:%d Set:%d",
             bicep.feedback_deg,
             bicep.current_deg);
    printRow(3, row);

    snprintf(row, sizeof(row), "T:%d R:%d",
             bicep.target_deg,
             bicepRestDeg());
    printRow(4, row);

    snprintf(row, sizeof(row), "Lim:%d-%d",
             bicepMinDeg(),
             bicepMaxDeg());
    printRow(5, row);

    snprintf(row, sizeof(row), "I:%dmA L:%d",
             bicep.current_mA,
             rounded_int(bicepCfg.current_limit_mA));
    printRow(6, row);

    snprintf(row, sizeof(row), "FD:%s ST:%s",
             bicepCfg.fault_detection_enabled ? "ON" : "OFF",
             bicep.fault_active ? "FLT" : "OK");
    printRow(7, row);
}

void splashPowerSettling()
{
    oled.clearDisplay();
    resetOledCache();

    printRow(0, "SUBSYS1 SAFE");
    printRow(2, "Aux PWM rest");
    printRow(3, "D12+D13 ON");
    printRow(4, "Read bicep FB");
    printRow(6, "Wait 2 sec");
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
    if (mode == MODE_POWER_OFF)
    {
        detachBicepPwm();
        detachAuxServos();

        /*
          Required safe startup:

          1. D12 and D13 still OFF.
          2. All non-feedback servos receive rest PWM.
          3. D12 and D13 ON.
          4. Wait 2 s.
          5. Read BICEP_R feedback.
          6. Attach BICEP_R from measured position.
          7. Move BICEP_R to rest.
        */
        attachAuxServosAtRestBeforePower();

        setGroup1Power(true);
        setGroup2Power(true);

        mode = MODE_TO_REST;

        splashPowerSettling();
        delay(POWER_SETTLE_MS);

        updateBicepSensors();

        bicep.current_deg = bicepConstrainAngle(bicep.feedback_deg);
        bicep.target_deg = bicep.current_deg;
        bicep.fault_active = false;
        bicep.overcurrent_start = 0;

        attachBicepFromMeasuredPosition();

        setAuxTargetsRest();
        setBicepTargetRest();

        resetOledCache();
        oled.clearDisplay();
        updateDisplay();

        return;
    }

    if (mode == MODE_TO_REST)
    {
        setAuxTargetsMax();
        setBicepTargetMax();

        mode = MODE_TO_MAX;

        updateDisplay();
        return;
    }

    if (mode == MODE_TO_MAX)
    {
        setAuxTargetsMin();
        setBicepTargetMin();

        mode = MODE_TO_MIN;

        updateDisplay();
        return;
    }

    if (mode == MODE_TO_MIN)
    {
        setAuxTargetsRest();
        setBicepTargetRest();

        mode = MODE_REST_THEN_OFF;

        updateDisplay();
        return;
    }

    if (mode == MODE_REST_THEN_OFF)
    {
        shutdownToPowerOff();
        updateDisplay();
        return;
    }

    if (mode == MODE_FAULT)
    {
        shutdownToPowerOff();
        updateDisplay();
        return;
    }
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

    analogReference(EXTERNAL);

    readBicepConfig();

    bicep.current_deg = bicepRestDeg();
    bicep.target_deg = bicepRestDeg();
    bicep.feedback_deg = bicep.current_deg;
    bicep.current_mA = 0;
    bicep.speed_pct = bicepCfg.default_speed_pct;
    bicep.accel_pct = bicepCfg.default_accel_pct;
    bicep.attached = false;
    bicep.fault_active = false;
    bicep.last_update = millis();
    bicep.overcurrent_start = 0;

    initAuxServos();

    Wire.begin();

    oled.begin();
    oled.setPowerSave(0);
    oled.setFont(u8x8_font_chroma48medium8_r);

    resetOledCache();
    oled.clearDisplay();

    mode = MODE_POWER_OFF;

    updateDisplay();
    lastDisplayUpdate = millis();
}

// ============================================================================
// Main loop
// ============================================================================

void loop()
{
    updateAuxServos();
    updateBicepMotion();

    if (mode == MODE_REST_THEN_OFF)
    {
        if (auxServosAtTarget() && bicepAtTarget())
        {
            shutdownToPowerOff();
        }
    }

    if (nextButtonPressedEvent())
    {
        handleButtonPress();
    }

    if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_MS)
    {
        lastDisplayUpdate = millis();
        updateDisplay();
    }
}
