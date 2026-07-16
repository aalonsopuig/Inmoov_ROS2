/*
===============================================================================
File:         Xicro_subsys1_ID_1.ino
Version:      2.0.1
Author:       Alejandro Alonso Puig (https://github.com/aalonsopuig) + GPT
Date:         2026-07-16
License:      Apache 2.0
-------------------------------------------------------------------------------
Description:

Firmware for InMoov robot, subsystem 1, Arduino Uno + XICRO + ROS 2.

This version integrates:

- XICRO command reception from ROS 2.
- Safe staged startup to rest position.
- Servo limits from servo_config_inmoov.h.
- Smooth interpolated motion.
- BICEP_R feedback-based startup.
- BICEP_R current protection.
- OLED information screen.
- Button-driven servo information display.

RAM-saving changes in v2.0.1:

- Removed OLED row cache:
    char lastOledRows[8][17]
  This saves 136 bytes of SRAM.

- OLED rows are now always rewritten directly.
  This is less elegant, but much safer for Arduino Uno SRAM.

- Runtime variables use smaller integer types where practical.

Important XICRO behavior:

- Before a servo receives its first non-zero ROS/XICRO command, it remains at rest.
- First non-zero command activates ROS/XICRO control for that servo.
- After activation, command 0 means "go to rest".
- Every command is constrained to allowed_min_deg / allowed_max_deg.

Startup sequence:

1. D12/D13 OFF.
2. Initial 2 s stabilization.
3. D12 ON with no PWM active.
4. Wait 2 s.
5. Read BICEP_R feedback.
6. Convert ADC to degrees.
7. Limit measured BICEP_R angle to allowed range.
8. D12 OFF.
9. Attach BICEP_R with PWM corresponding to measured safe position.
10. Attach remaining group 1 servos at rest.
11. D12 ON.
12. Command BICEP_R to rest.
13. Attach group 2 servos at rest.
14. D13 ON.
15. Enter normal XICRO-controlled mode.

===============================================================================
*/

#include "Xicro_subsys1_ID_1.h"      // XICRO-generated interface for this Arduino node.

#include <Arduino.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include <Servo.h>
#include <U8x8lib.h>

#include "servo_config_inmoov.h"     // ServoConfig table stored in PROGMEM.

// ============================================================================
// Firmware version
// ============================================================================

#define FW_VERSION "2.0.1"

// ============================================================================
// Shield pins
// ============================================================================

#define SERVO_GROUP_1_PIN  12        // Power control for group 1: fingers, bicep, rotate.
#define SERVO_GROUP_2_PIN  13        // Power control for group 2: shoulder, omoplate.
#define NEXT_BUTTON_PIN     7        // Button to cycle OLED information screens.

// ============================================================================
// XICRO
// ============================================================================

#define XICRO_BAUDRATE 57600         // Same baudrate as the previous XICRO firmware.

Xicro xicro;                         // Global XICRO interface object.

// ============================================================================
// OLED
// ============================================================================

U8X8_SSD1306_128X64_NONAME_HW_I2C oled(U8X8_PIN_NONE);

#define OLED_COLS 16                 // 16 text columns in U8x8 mode.
#define OLED_ROWS 8                  // 8 text rows in U8x8 mode.

// ============================================================================
// Timing
// ============================================================================

#define ADC_SAMPLES             8    // Number of ADC samples for simple averaging.
#define SERVO_UPDATE_MS        40    // Servo interpolation period.
#define DISPLAY_UPDATE_MS    2000    // OLED refresh period when showing servo data.
#define DEBOUNCE_MS            40    // Button debounce time.
#define INITIAL_STABILIZE_MS 2000    // Initial general stabilization delay.
#define POWER_SETTLE_MS      2000    // Delay after enabling D12 only for bicep feedback read.
#define POWER_OFF_PAUSE_MS    200    // Small pause after switching D12 off before attaching PWM.

// ============================================================================
// System mode
// ============================================================================

enum SystemMode
{
    MODE_STARTING = 0,               // Startup sequence is being executed.
    MODE_RUNNING,                    // Normal mode: XICRO commands are processed.
    MODE_FAULT                       // BICEP_R current fault: bicep PWM detached.
};

SystemMode mode = MODE_STARTING;

// ============================================================================
// Runtime servo state
// ============================================================================

struct RuntimeServo
{
    Servo servo;                     // Arduino Servo object. This consumes SRAM, but is needed.

    int16_t current_deg;             // Current commanded position in degrees.
    int16_t target_deg;              // Target commanded position in degrees.
    int16_t pwm_us;                  // Last PWM value sent with writeMicroseconds().

    uint8_t speed_pct;               // Speed percentage from configuration.
    bool attached;                   // True when Servo.attach() has been called.
    bool first_commanded;            // True after first non-zero ROS command.
    unsigned long last_update;       // Last interpolation update time.
};

RuntimeServo joints[NUM_SERVOS];     // One runtime state per configured servo.

// Cached bicep configuration, because BICEP_R is used often.
ServoConfig bicepCfg;

// BICEP_R sensor and safety state.
int16_t bicep_feedback_adc = -1;
int16_t bicep_feedback_deg_raw = 0;
int16_t bicep_feedback_deg_safe = 0;
int16_t bicep_current_mA = 0;
bool bicep_fault_active = false;
unsigned long bicep_overcurrent_start = 0;

// ============================================================================
// Button and display state
// ============================================================================

bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long lastDisplayUpdate = 0;

int8_t selectedDisplayServo = -1;    // -1 means intro screen. 0..8 means servo index screen.

// ============================================================================
// Generic helpers
// ============================================================================

int16_t rounded_int(float value)
{
    if (value >= 0.0f)
    {
        return (int16_t)(value + 0.5f);
    }
    else
    {
        return (int16_t)(value - 0.5f);
    }
}

int16_t safe_constrain_int(int16_t value, int16_t min_val, int16_t max_val)
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

float readAveragedADC(uint8_t pin, uint8_t samples)
{
    long sum = 0;

    for (uint8_t i = 0; i < samples; i++)
    {
        sum += analogRead(pin);
    }

    return (float)sum / (float)samples;
}

void readServoConfig(uint8_t index, ServoConfig &cfg)
{
    // servoConfigs[] is stored in PROGMEM in servo_config_inmoov.h.
    // We copy one ServoConfig at a time to SRAM only when needed.
    memcpy_P(&cfg, &servoConfigs[index], sizeof(ServoConfig));
}

void readBicepConfig()
{
    readServoConfig(BICEP_R, bicepCfg);
}

// ============================================================================
// OLED helpers
// ============================================================================

void printRow(uint8_t row, const char *text)
{
    /*
      RAM-saving OLED writer.

      Previous version kept a full 8x17 row cache in SRAM to avoid rewriting
      unchanged rows. That was convenient, but expensive on Arduino Uno.

      This version always rewrites the row. It uses only a local 17-byte buffer,
      which exists on the stack only while this function runs.
    */
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

    oled.setCursor(0, row);
    oled.print(buffer);
}

void clearScreen()
{
    oled.clearDisplay();
}

void showIntroScreen()
{
    clearScreen();

    printRow(0, "SUBSYS1 2.0.1");
    printRow(1, "XICRO READY");
    printRow(3, "Press button");
    printRow(4, "to view servo");
    printRow(5, "information");

    if (bicep_fault_active)
    {
        printRow(7, "Status: FAULT");
    }
    else
    {
        printRow(7, "Status: OK");
    }
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

uint8_t servoGroup(uint8_t servoIndex)
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

bool servoIsGroup1NonBicep(uint8_t servoIndex)
{
    return servoIndex == THUMB_R  ||
           servoIndex == INDEX_R  ||
           servoIndex == MIDDLE_R ||
           servoIndex == RING_R   ||
           servoIndex == PINKY_R  ||
           servoIndex == ROTATE_R;
}

bool servoIsGroup2(uint8_t servoIndex)
{
    return servoIndex == SHOULDER_R ||
           servoIndex == OMOPLATE_R;
}

// ============================================================================
// Servo configuration helpers
// ============================================================================

int16_t cfgAllowedMin(const ServoConfig &cfg)
{
    return rounded_int(cfg.allowed_min_deg);
}

int16_t cfgAllowedMax(const ServoConfig &cfg)
{
    return rounded_int(cfg.allowed_max_deg);
}

int16_t cfgServoMin(const ServoConfig &cfg)
{
    return rounded_int(cfg.servo_min_deg);
}

int16_t cfgServoMax(const ServoConfig &cfg)
{
    return rounded_int(cfg.servo_max_deg);
}

int16_t cfgRestDeg(const ServoConfig &cfg)
{
    // Rest angle must always be inside the mechanical safe range.
    return safe_constrain_int(
        rounded_int(cfg.rest_deg),
        cfgAllowedMin(cfg),
        cfgAllowedMax(cfg)
    );
}

int16_t cfgConstrainAllowedAngle(const ServoConfig &cfg, int16_t angleDeg)
{
    // Mechanical command limit.
    return safe_constrain_int(
        angleDeg,
        cfgAllowedMin(cfg),
        cfgAllowedMax(cfg)
    );
}

int16_t cfgConstrainServoAngle(const ServoConfig &cfg, int16_t angleDeg)
{
    // Servo calibration range, not necessarily the same as the allowed range.
    return safe_constrain_int(
        angleDeg,
        cfgServoMin(cfg),
        cfgServoMax(cfg)
    );
}

int16_t cfgAngleToPwmUs(const ServoConfig &cfg, int16_t angleDeg)
{
    /*
      Converts logical angle to PWM.

      Important:
      - This function does NOT apply allowed_min_deg / allowed_max_deg.
      - It only uses servo_min_deg / servo_max_deg and pwm_min_us / pwm_max_us.
      - Mechanical limits must be applied before setting current_deg or target_deg.
    */
    int16_t logicalAngle = cfgConstrainServoAngle(cfg, angleDeg);
    int16_t physicalAngle = logicalAngle;

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

int16_t cfgComputeStepDeg(const ServoConfig &cfg, uint8_t speedPct)
{
    /*
      Converts max speed + percentage + update period into integer degree step.

      A minimum step of 1 degree is enforced so that movement never stalls due
      to rounding on slow servos.
    */
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

    int16_t step = rounded_int(deg_per_update);

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

int16_t bicepFeedbackDegRawFromADC(int16_t adcValue)
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

float bicepCurrentMilliAmpsFromADC(int16_t adcValue)
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

int16_t bicepReadCurrentMilliAmps()
{
    if (!bicepHasCurrentADC() || !bicepHasCurrentCalibration())
    {
        return 0;
    }

    int16_t currentAdc =
        rounded_int(readAveragedADC(bicepCfg.current_adc_pin, ADC_SAMPLES));

    return rounded_int(bicepCurrentMilliAmpsFromADC(currentAdc));
}

void updateBicepSensors()
{
    /*
      Feedback is meaningful only when group 1 is powered.
      Current is checked only when BICEP_R is attached and driven.
    */
    if (digitalRead(SERVO_GROUP_1_PIN) == HIGH)
    {
        if (bicepHasFeedbackADC())
        {
            bicep_feedback_adc =
                rounded_int(readAveragedADC(bicepCfg.feedback_adc_pin, ADC_SAMPLES));

            bicep_feedback_deg_raw =
                bicepFeedbackDegRawFromADC(bicep_feedback_adc);

            bicep_feedback_deg_safe =
                cfgConstrainAllowedAngle(bicepCfg, bicep_feedback_deg_raw);
        }
        else
        {
            bicep_feedback_adc = -1;
            bicep_feedback_deg_raw = cfgRestDeg(bicepCfg);
            bicep_feedback_deg_safe = cfgRestDeg(bicepCfg);
        }

        if (joints[BICEP_R].attached)
        {
            bicep_current_mA = bicepReadCurrentMilliAmps();
        }
        else
        {
            bicep_current_mA = 0;
        }
    }
    else
    {
        bicep_feedback_adc = -1;
        bicep_feedback_deg_raw = joints[BICEP_R].current_deg;
        bicep_feedback_deg_safe = joints[BICEP_R].current_deg;
        bicep_current_mA = 0;
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
// Runtime servo initialization and attach/detach
// ============================================================================

void initRuntimeServos()
{
    for (uint8_t i = 0; i < NUM_SERVOS; i++)
    {
        ServoConfig cfg;
        readServoConfig(i, cfg);

        joints[i].current_deg = cfgRestDeg(cfg);
        joints[i].target_deg = cfgRestDeg(cfg);
        joints[i].pwm_us = cfgAngleToPwmUs(cfg, joints[i].current_deg);
        joints[i].speed_pct = cfg.default_speed_pct;
        joints[i].attached = false;
        joints[i].first_commanded = false;
        joints[i].last_update = millis();
    }
}

void attachServoAtCurrent(uint8_t servoIndex)
{
    /*
      Attach servo using the already stored current_deg.

      Pattern used deliberately:
        writeMicroseconds()
        attach()
        writeMicroseconds()

      The first write sets the Servo library internal value before attaching.
      The second write reinforces the intended PWM immediately after attach.
    */
    ServoConfig cfg;
    readServoConfig(servoIndex, cfg);

    joints[servoIndex].pwm_us =
        cfgAngleToPwmUs(cfg, joints[servoIndex].current_deg);

    if (!joints[servoIndex].attached)
    {
        joints[servoIndex].servo.writeMicroseconds(joints[servoIndex].pwm_us);
        joints[servoIndex].servo.attach(cfg.pwm_pin);
        joints[servoIndex].attached = true;
    }

    joints[servoIndex].servo.writeMicroseconds(joints[servoIndex].pwm_us);
    joints[servoIndex].last_update = millis();
}

void attachServoAtRest(uint8_t servoIndex)
{
    /*
      Attach a non-feedback servo directly at its rest position.

      This is used while its power group is still OFF, so that the PWM line is
      already valid before the servo receives power.
    */
    ServoConfig cfg;
    readServoConfig(servoIndex, cfg);

    joints[servoIndex].current_deg = cfgRestDeg(cfg);
    joints[servoIndex].target_deg = cfgRestDeg(cfg);
    joints[servoIndex].pwm_us = cfgAngleToPwmUs(cfg, joints[servoIndex].current_deg);

    if (!joints[servoIndex].attached)
    {
        joints[servoIndex].servo.writeMicroseconds(joints[servoIndex].pwm_us);
        joints[servoIndex].servo.attach(cfg.pwm_pin);
        joints[servoIndex].attached = true;
    }

    joints[servoIndex].servo.writeMicroseconds(joints[servoIndex].pwm_us);
    joints[servoIndex].last_update = millis();
}

void detachServo(uint8_t servoIndex)
{
    if (joints[servoIndex].attached)
    {
        joints[servoIndex].servo.detach();
        joints[servoIndex].attached = false;
    }
}

void detachAllServos()
{
    for (uint8_t i = 0; i < NUM_SERVOS; i++)
    {
        detachServo(i);
    }
}

// ============================================================================
// Startup sequence
// ============================================================================

void performStagedStartupToRest()
{
    mode = MODE_STARTING;

    detachAllServos();
    allPowerOff();

    /*
      Phase 1:
      Enable D12 with no PWM active.
      Purpose: power BICEP_R feedback electronics and read its actual position
      in an electrically quiet situation, with no PWM running in the cable flat.
    */
    setGroup1Power(true);

    clearScreen();
    printRow(0, "SUBSYS1 2.0.1");
    printRow(1, "AUTO START");
    printRow(2, "D12 ON");
    printRow(3, "NO PWM");
    printRow(4, "READ BICEP FB");
    printRow(6, "WAIT 2 sec");

    delay(POWER_SETTLE_MS);

    updateBicepSensors();

    /*
      Phase 2:
      Turn group 1 power OFF again before attaching PWM lines.
    */
    setGroup1Power(false);
    delay(POWER_OFF_PAUSE_MS);

    /*
      Phase 3:
      Prepare BICEP_R PWM from feedback-derived safe position.
      D12 is OFF, so the servo should not move while the PWM line is prepared.
    */
    joints[BICEP_R].current_deg = bicep_feedback_deg_safe;
    joints[BICEP_R].target_deg = joints[BICEP_R].current_deg;
    attachServoAtCurrent(BICEP_R);

    /*
      Phase 4:
      Prepare group 1 remaining servos at rest while D12 is still OFF.
    */
    for (uint8_t i = 0; i < NUM_SERVOS; i++)
    {
        if (servoIsGroup1NonBicep(i))
        {
            attachServoAtRest(i);
        }
    }

    /*
      Phase 5:
      Enable group 1.
      BICEP_R starts from measured safe position; then it is commanded to rest.
    */
    setGroup1Power(true);

    ServoConfig bcfg;
    readServoConfig(BICEP_R, bcfg);
    joints[BICEP_R].target_deg = cfgRestDeg(bcfg);

    /*
      Phase 6:
      Prepare group 2 servos at rest while D13 is still OFF.
    */
    for (uint8_t i = 0; i < NUM_SERVOS; i++)
    {
        if (servoIsGroup2(i))
        {
            attachServoAtRest(i);
        }
    }

    /*
      Phase 7:
      Enable group 2.
    */
    setGroup2Power(true);

    mode = MODE_RUNNING;
}

// ============================================================================
// XICRO command handling
// ============================================================================

int16_t getIncomingXicroValue(uint8_t servoIndex)
{
    /*
      The field names come from the XICRO-generated header.
      They match the previous subsystem 1 firmware.
    */
    switch (servoIndex)
    {
        case THUMB_R:
            return xicro.Subscription_thumb_finger_R.message.data;

        case INDEX_R:
            return xicro.Subscription_index_finger_R.message.data;

        case MIDDLE_R:
            return xicro.Subscription_middle_finger_R.message.data;

        case RING_R:
            return xicro.Subscription_ring_finger_R.message.data;

        case PINKY_R:
            return xicro.Subscription_pinky_finger_R.message.data;

        case BICEP_R:
            return xicro.Subscription_bicep_R.message.data;

        case ROTATE_R:
            return xicro.Subscription_rotate_R.message.data;

        case SHOULDER_R:
            return xicro.Subscription_shoulder_R.message.data;

        case OMOPLATE_R:
            return xicro.Subscription_omoplate_R.message.data;

        default:
            return 0;
    }
}

void updateTargetsFromXicro()
{
    if (mode != MODE_RUNNING)
    {
        return;
    }

    for (uint8_t i = 0; i < NUM_SERVOS; i++)
    {
        ServoConfig cfg;
        readServoConfig(i, cfg);

        int16_t incoming = getIncomingXicroValue(i);

        /*
          Activation policy inherited from previous firmware:

          - Before activation, a servo ignores zeros and remains at rest.
          - First non-zero command activates that servo.
          - After activation, zero means "go to rest".
        */
        if (!joints[i].first_commanded && incoming != 0)
        {
            joints[i].first_commanded = true;
        }

        if (joints[i].first_commanded)
        {
            if (incoming == 0)
            {
                joints[i].target_deg = cfgRestDeg(cfg);
            }
            else
            {
                joints[i].target_deg =
                    cfgConstrainAllowedAngle(cfg, incoming);
            }
        }
        else
        {
            joints[i].target_deg = cfgRestDeg(cfg);
        }
    }
}

// ============================================================================
// Motion and safety
// ============================================================================

void checkBicepFault()
{
    if (!joints[BICEP_R].attached)
    {
        bicep_overcurrent_start = 0;
        return;
    }

    if (bicep_fault_active)
    {
        return;
    }

    if (!bicepCfg.fault_detection_enabled)
    {
        bicep_overcurrent_start = 0;
        return;
    }

    if (!bicepHasCurrentADC() || !bicepHasCurrentCalibration())
    {
        bicep_overcurrent_start = 0;
        return;
    }

    updateBicepSensors();

    if (bicep_current_mA > rounded_int(bicepCfg.current_limit_mA))
    {
        if (bicep_overcurrent_start == 0)
        {
            bicep_overcurrent_start = millis();
        }

        if (millis() - bicep_overcurrent_start >= bicepCfg.overcurrent_time_ms)
        {
            /*
              Fault strategy:
              - Detach only BICEP_R PWM.
              - Leave system in FAULT mode.
              - Other servos keep their last state.
              - This avoids forcing more motion during a possible mechanical issue.
            */
            bicep_fault_active = true;
            detachServo(BICEP_R);
            mode = MODE_FAULT;
        }
    }
    else
    {
        bicep_overcurrent_start = 0;
    }
}

void updateServoMotion(uint8_t servoIndex)
{
    if (!joints[servoIndex].attached)
    {
        return;
    }

    if (servoIndex == BICEP_R)
    {
        checkBicepFault();

        if (bicep_fault_active || mode == MODE_FAULT)
        {
            return;
        }
    }

    ServoConfig cfg;
    readServoConfig(servoIndex, cfg);

    unsigned long now = millis();

    if (now - joints[servoIndex].last_update < SERVO_UPDATE_MS)
    {
        return;
    }

    joints[servoIndex].last_update = now;

    /*
      Reapply safety constraint on target at every update.
      This protects against any invalid ROS/XICRO command.
    */
    joints[servoIndex].target_deg =
        cfgConstrainAllowedAngle(cfg, joints[servoIndex].target_deg);

    int16_t step = cfgComputeStepDeg(cfg, joints[servoIndex].speed_pct);

    if (joints[servoIndex].current_deg < joints[servoIndex].target_deg)
    {
        joints[servoIndex].current_deg += step;

        if (joints[servoIndex].current_deg > joints[servoIndex].target_deg)
        {
            joints[servoIndex].current_deg = joints[servoIndex].target_deg;
        }
    }
    else if (joints[servoIndex].current_deg > joints[servoIndex].target_deg)
    {
        joints[servoIndex].current_deg -= step;

        if (joints[servoIndex].current_deg < joints[servoIndex].target_deg)
        {
            joints[servoIndex].current_deg = joints[servoIndex].target_deg;
        }
    }

    /*
      Reapply safety constraint to current_deg before generating PWM.
    */
    joints[servoIndex].current_deg =
        cfgConstrainAllowedAngle(cfg, joints[servoIndex].current_deg);

    joints[servoIndex].pwm_us =
        cfgAngleToPwmUs(cfg, joints[servoIndex].current_deg);

    joints[servoIndex].servo.writeMicroseconds(joints[servoIndex].pwm_us);
}

void updateAllServoMotion()
{
    for (uint8_t i = 0; i < NUM_SERVOS; i++)
    {
        updateServoMotion(i);
    }
}

// ============================================================================
// Display
// ============================================================================

void updateServoInfoScreen()
{
    if (selectedDisplayServo < 0)
    {
        return;
    }

    uint8_t servoIndex = (uint8_t)selectedDisplayServo;

    ServoConfig cfg;
    readServoConfig(servoIndex, cfg);

    char row[OLED_COLS + 1];

    clearScreen();

    snprintf(row, sizeof(row), "SUBSYS1 2.0.1");
    printRow(0, row);

    snprintf(row, sizeof(row), "%d/%d %s",
             selectedDisplayServo + 1,
             NUM_SERVOS,
             servoName(servoIndex));
    printRow(1, row);

    snprintf(row, sizeof(row), "G:%d Pin:%d",
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

    snprintf(row, sizeof(row), "Set:%d T:%d",
             joints[servoIndex].current_deg,
             joints[servoIndex].target_deg);
    printRow(5, row);

    snprintf(row, sizeof(row), "PWM:%d %s",
             joints[servoIndex].pwm_us,
             joints[servoIndex].attached ? "ON" : "OFF");
    printRow(6, row);

    if (servoIndex == BICEP_R)
    {
        updateBicepSensors();

        snprintf(row, sizeof(row), "I:%dmA %s",
                 bicep_current_mA,
                 bicep_fault_active ? "FAULT" : "OK");
    }
    else
    {
        snprintf(row, sizeof(row), "%s",
                 mode == MODE_FAULT ? "SYSTEM FAULT" : "SYSTEM OK");
    }

    printRow(7, row);
}

// ============================================================================
// Button
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

    if (selectedDisplayServo >= NUM_SERVOS)
    {
        selectedDisplayServo = 0;
    }

    updateServoInfoScreen();
}

// ============================================================================
// Setup
// ============================================================================

void setup()
{
    /*
      Power lines must be defined and switched OFF as early as possible.
    */
    pinMode(SERVO_GROUP_1_PIN, OUTPUT);
    pinMode(SERVO_GROUP_2_PIN, OUTPUT);
    allPowerOff();

    /*
      Button input uses internal pull-up.
      Pressed = LOW.
    */
    pinMode(NEXT_BUTTON_PIN, INPUT_PULLUP);
    lastButtonReading = digitalRead(NEXT_BUTTON_PIN);
    stableButtonState = lastButtonReading;
    lastDebounceTime = millis();

    /*
      AREF is externally connected to 3.3 V.
      This must be configured before analogRead().
    */
    analogReference(EXTERNAL);

    /*
      XICRO serial link.
    */
    Serial.begin(XICRO_BAUDRATE);
    xicro.begin(&Serial);

    /*
      Read repeated bicep configuration once.
      Initialize runtime states from servo_config_inmoov.h.
    */
    readBicepConfig();
    initRuntimeServos();

    /*
      OLED initialization.
    */
    Wire.begin();

    oled.begin();
    oled.setPowerSave(0);
    oled.setFont(u8x8_font_chroma48medium8_r);

    clearScreen();
    printRow(0, "SUBSYS1 2.0.1");
    printRow(1, "XICRO BOOT");
    printRow(3, "Stabilizing");
    printRow(4, "system...");
    printRow(6, "Please wait");

    /*
      General stabilization delay before staged servo startup.
    */
    delay(INITIAL_STABILIZE_MS);

    /*
      Safe autonomous startup to rest.
    */
    performStagedStartupToRest();

    /*
      Initial information screen.
    */
    selectedDisplayServo = -1;
    showIntroScreen();

    lastDisplayUpdate = millis();
}

// ============================================================================
// Main loop
// ============================================================================

void loop()
{
    /*
      Keep XICRO alive and receive ROS 2 topic values.
    */
    xicro.Spin_node();

    /*
      Convert XICRO values into servo targets.
      Limits are applied before targets are stored.
    */
    updateTargetsFromXicro();

    /*
      Smoothly move every attached servo towards its target.
      BICEP_R current fault is checked inside its update path.
    */
    updateAllServoMotion();

    /*
      Button only changes the displayed servo information.
      It never commands motion.
    */
    if (nextButtonPressedEvent())
    {
        handleButtonPress();
    }

    /*
      Periodic refresh only when a servo screen is selected.
      The intro screen remains static to reduce OLED activity.
    */
    if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_MS)
    {
        lastDisplayUpdate = millis();

        if (selectedDisplayServo >= 0)
        {
            updateServoInfoScreen();
        }
    }
}
