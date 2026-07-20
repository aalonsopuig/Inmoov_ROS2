/*
===============================================================================
File:         Xicro_subsys1_ID_1.ino
Version:      2.1.0
Author:       Alejandro Alonso Puig (https://github.com/aalonsopuig) + GPT
Date:         2026-07-20
License:      Apache 2.0
-------------------------------------------------------------------------------
Description:

Firmware for InMoov robot "Paul" - Subsystem 1.

This Arduino Uno firmware controls the right arm and related servos of the
InMoov robot through a ROS 2 interface generated with XICRO. The Arduino receives
servo target positions from ROS 2 topics, applies mechanical limits defined in
servo_config_inmoov.h, and moves the servos smoothly using incremental position
updates.

The firmware controls two external servo power groups through the custom shield:

- Servo group 1, controlled by D12:
    BICEP_R, THUMB_R, INDEX_R, MIDDLE_R, RING_R, PINKY_R and ROTATE_R.

- Servo group 2, controlled by D13:
    SHOULDER_R and OMOPLATE_R.

Both power groups are kept disabled immediately after reset and are enabled only
after a controlled startup sequence. This reduces the risk of uncontrolled servo
movement during Arduino boot, USB serial reset, XICRO initialization or PWM
attachment.

The BICEP_R servo has additional protection and sensing:

- Position feedback through an analog potentiometer.
- Current sensing through an analog current sensor.
- Feedback-based startup, so the firmware attaches BICEP_R at its measured
  mechanical position before commanding it smoothly to its rest position.
- Overcurrent protection, which detaches BICEP_R and enters FAULT mode if the
  measured current remains above the configured limit for the configured time.

The OLED display is used as a low-memory diagnostic interface. Because the
Arduino Uno has limited SRAM, the display is not refreshed periodically. It is
updated only in these cases:

- During boot.
- When startup has completed.
- When the diagnostic button is pressed.
- When a BICEP_R current fault is detected.

The button on D7 cycles through two diagnostic screens:

- Screen 1/2:
    Last commanded target positions for all servos except BICEP_R.

- Screen 2/2:
    BICEP_R commanded target position, measured feedback position and current.

Angle values on the OLED are shown as plain integer degrees without any degree
symbol. This avoids font compatibility problems and keeps the fixed-width text
layout aligned on small character displays.

The values shown as commanded positions are internal target values after applying
the configured safety limits. They are not physical position measurements, except
for the BICEP_R feedback value, which is derived from its potentiometer.

The firmware does not use Serial.print() because the hardware serial port is
reserved for XICRO communication with the ROS 2 host.

===============================================================================
*/

#include "Xicro_subsys1_ID_1.h"

#include <Arduino.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include <Servo.h>
#include <U8x8lib.h>

#include "servo_config_inmoov.h"

// ============================================================================
// Firmware version
// ============================================================================

#define FW_VERSION "2.1.0"

// ============================================================================
// Shield pins
// ============================================================================

#define SERVO_GROUP_1_PIN  12        // Servo power group 1 control. LOW = off, HIGH = on.
#define SERVO_GROUP_2_PIN  13        // Servo power group 2 control. LOW = off, HIGH = on.
#define NEXT_BUTTON_PIN     7        // Diagnostic button. Connected to GND. Uses INPUT_PULLUP.

// ============================================================================
// XICRO serial configuration
// ============================================================================

#define XICRO_BAUDRATE 57620         // Must match the ROS-side XICRO node baudrate.

Xicro xicro;                         // XICRO communication object generated from YAML.

// ============================================================================
// OLED display
// ============================================================================

/*
  SSD1315 OLED modules are usually SSD1306-compatible at command level.
  U8x8 is used instead of a framebuffer-based graphics library to reduce SRAM
  consumption on Arduino Uno.
*/
U8X8_SSD1306_128X64_NONAME_HW_I2C oled(U8X8_PIN_NONE);

// ============================================================================
// Timing parameters
// ============================================================================

#define ADC_SAMPLES             8    // Number of ADC samples used for simple averaging.
#define SERVO_UPDATE_MS        40    // Servo interpolation update period.
#define DEBOUNCE_MS            40    // Button debounce time.
#define INITIAL_STABILIZE_MS 2000    // Initial delay after XICRO start, while keeping XICRO alive.
#define POWER_SETTLE_MS      2000    // Time for group 1 power to settle before reading BICEP_R feedback.
#define POWER_OFF_PAUSE_MS    200    // Pause after powering off before attaching PWM.

// ============================================================================
// System mode
// ============================================================================

enum SystemMode
{
    MODE_STARTING = 0,               // Startup sequence in progress.
    MODE_RUNNING,                    // Normal XICRO-controlled operation.
    MODE_FAULT                       // Fault condition. BICEP_R is detached.
};

SystemMode mode = MODE_STARTING;

// ============================================================================
// Runtime servo state
// ============================================================================

struct RuntimeServo
{
    Servo servo;                     // Arduino Servo object.

    int16_t current_deg;             // Internal smoothed commanded position.
    int16_t target_deg;              // Target angle after safety limiting.
    int16_t pwm_us;                  // Last PWM pulse width written to servo.

    uint8_t speed_pct;               // Speed percentage from configuration.
    bool attached;                   // True after Servo.attach().
    bool first_commanded;            // False until first non-zero XICRO command.
};

RuntimeServo joints[NUM_SERVOS];     // Runtime state for all servos in subsystem 1.

unsigned long lastServoUpdate = 0;   // One shared update timer for all servos.

// ============================================================================
// Cached BICEP_R configuration and sensor state
// ============================================================================

ServoConfig bicepCfg;                // Cached configuration for BICEP_R.

int16_t bicep_feedback_adc = -1;     // Raw averaged ADC value from BICEP_R feedback.
int16_t bicep_feedback_deg_raw = 0;  // Feedback-derived position before allowed-range limiting.
int16_t bicep_feedback_deg_safe = 0; // Feedback-derived position constrained to allowed range.
int16_t bicep_current_mA = 0;        // Current estimate in mA.

int16_t bicep_fault_current_mA = 0;  // Latched current at the moment of fault.
int16_t bicep_fault_feedback_deg = 0;// Latched feedback position at the moment of fault.

bool bicep_fault_active = false;     // True after BICEP_R overcurrent fault.
bool faultScreenShown = false;       // Prevents repeated OLED redraws after fault.
unsigned long bicep_overcurrent_start = 0;

// ============================================================================
// Button and OLED diagnostic state
// ============================================================================

bool lastButtonReading = HIGH;       // Last raw button reading.
bool stableButtonState = HIGH;       // Debounced button state.
unsigned long lastDebounceTime = 0;  // Last time the raw button reading changed.

/*
  diagnosticScreen indicates which diagnostic screen will be shown on the next
  button press.

  0 -> next press shows screen 1/2.
  1 -> next press shows screen 2/2.
*/
uint8_t diagnosticScreen = 0;

// ============================================================================
// Generic helpers
// ============================================================================

int16_t rounded_int(float value)
{
    /*
      Small helper for explicit float-to-int rounding.

      Using a helper keeps the conversions consistent throughout the firmware.
    */
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
    /*
      Integer constrain helper.

      This avoids relying on macro expansion and keeps the code explicit.
    */
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
    /*
      Reads an analog input several times and returns the average value.

      The function uses a long accumulator because up to 1023 * samples is added.
      With the current ADC_SAMPLES value this is far below the long range, but
      the type avoids accidental overflow if samples is increased later.
    */
    long sum = 0;

    for (uint8_t i = 0; i < samples; i++)
    {
        sum += analogRead(pin);
    }

    return (float)sum / (float)samples;
}

void readServoConfig(uint8_t index, ServoConfig &cfg)
{
    /*
      servoConfigs[] is stored in PROGMEM in servo_config_inmoov.h.

      Copying only one ServoConfig at a time avoids keeping the full
      configuration table in SRAM. This is important on Arduino Uno.
    */
    memcpy_P(&cfg, &servoConfigs[index], sizeof(ServoConfig));
}

void readBicepConfig()
{
    /*
      BICEP_R configuration is used frequently for feedback and current safety.
      Keeping only this single configuration cached is a reasonable SRAM tradeoff.
    */
    readServoConfig(BICEP_R, bicepCfg);
}

void waitWithXicro(unsigned long duration_ms)
{
    /*
      Wait while keeping XICRO alive.

      The Arduino Uno normally resets when the host opens the serial port.
      During startup waits, XICRO is still serviced so that the ROS-side bridge
      does not appear dead while the firmware is completing its safe startup.
    */
    unsigned long start = millis();

    while (millis() - start < duration_ms)
    {
        xicro.Spin_node();
    }
}

// ============================================================================
// OLED helpers
// ============================================================================

void oledPrintAngle3(int16_t value)
{
    /*
      Prints an angle using a fixed width of 3 numeric characters.

      Examples:
        0   -> "  0"
        7   -> "  7"
        45  -> " 45"
        170 -> "170"

      No degree symbol or suffix is printed. This keeps the OLED layout stable
      with the selected U8x8 font and avoids unsupported-character problems.
    */
    if (value < 0)
    {
        oled.print(F("---"));
        return;
    }

    if (value < 10)
    {
        oled.print(F("  "));
    }
    else if (value < 100)
    {
        oled.print(F(" "));
    }

    oled.print(value);
}

void oledPrintStatusLine()
{
    /*
      Common first line used by diagnostic screens.
    */
    oled.setCursor(0, 0);
    oled.print(F("SUB1 2.1.0 "));

    if (mode == MODE_FAULT || bicep_fault_active)
    {
        oled.print(F("FLT"));
    }
    else
    {
        oled.print(F("OK "));
    }
}

void oledShowBoot()
{
    /*
      Boot screen.

      Displayed once during setup, before the safe servo startup sequence.
    */
    oled.clearDisplay();

    oled.setCursor(0, 0);
    oled.print(F("SUBSYS1"));

    oled.setCursor(0, 1);
    oled.print(F("FW 2.1.0"));

    oled.setCursor(0, 2);
    oled.print(F("XICRO ROS2"));

    oled.setCursor(0, 3);
    oled.print(F("OLED DIAG"));

    oled.setCursor(0, 5);
    oled.print(F("Starting..."));
}

void oledShowReady()
{
    /*
      Ready screen.

      Displayed once after safe startup. The OLED is not refreshed again unless
      the button is pressed or a fault occurs.
    */
    oled.clearDisplay();

    oled.setCursor(0, 0);
    oled.print(F("SUB1 2.1.0 OK"));

    oled.setCursor(0, 1);
    oled.print(F("XICRO READY"));

    oled.setCursor(0, 2);
    oled.print(F("SERVOS READY"));

    oled.setCursor(0, 4);
    oled.print(F("BTN: DIAG"));

    oled.setCursor(0, 6);
    oled.print(F("NO AUTO REFRESH"));
}

void oledShowCommandScreen()
{
    /*
      Diagnostic screen 1/2.

      Shows the last commanded target angles for all servos except BICEP_R.
      These are target_deg values after safety limiting.

      The screen is useful to confirm that XICRO/ROS 2 commands are reaching the
      Arduino and being assigned to the expected servo channels.
    */
    oled.clearDisplay();

    oledPrintStatusLine();

    oled.setCursor(0, 1);
    oled.print(F("CMD 1/2"));

    oled.setCursor(0, 2);
    oled.print(F("TH:"));
    oledPrintAngle3(joints[THUMB_R].target_deg);
    oled.print(F(" IN:"));
    oledPrintAngle3(joints[INDEX_R].target_deg);

    oled.setCursor(0, 3);
    oled.print(F("MI:"));
    oledPrintAngle3(joints[MIDDLE_R].target_deg);
    oled.print(F(" RI:"));
    oledPrintAngle3(joints[RING_R].target_deg);

    oled.setCursor(0, 4);
    oled.print(F("PI:"));
    oledPrintAngle3(joints[PINKY_R].target_deg);
    oled.print(F(" RO:"));
    oledPrintAngle3(joints[ROTATE_R].target_deg);

    oled.setCursor(0, 5);
    oled.print(F("SH:"));
    oledPrintAngle3(joints[SHOULDER_R].target_deg);
    oled.print(F(" OM:"));
    oledPrintAngle3(joints[OMOPLATE_R].target_deg);

    oled.setCursor(0, 7);
    oled.print(F("BTN: NEXT"));
}

void oledShowBicepScreen()
{
    /*
      Diagnostic screen 2/2.

      Shows the most useful BICEP_R runtime data:

      CMD:
        Target angle currently commanded by ROS 2/XICRO after safety limiting.

      FB:
        Physical position estimate derived from the BICEP_R feedback
        potentiometer.

      I:
        Estimated BICEP_R current in mA.

      No periodic refresh is performed. Values are sampled and displayed only
      when the button is pressed.
    */
    updateBicepSensors();

    oled.clearDisplay();

    oledPrintStatusLine();

    oled.setCursor(0, 1);
    oled.print(F("BICEP 2/2"));

    oled.setCursor(0, 2);
    oled.print(F("CMD:"));
    oledPrintAngle3(joints[BICEP_R].target_deg);

    oled.setCursor(0, 3);
    oled.print(F("FB: "));
    oledPrintAngle3(bicep_feedback_deg_safe);

    oled.setCursor(0, 4);
    oled.print(F("I:"));
    oled.print(bicep_current_mA);
    oled.print(F("mA"));

    oled.setCursor(0, 6);

    if (bicep_fault_active)
    {
        oled.print(F("BICEP FAULT"));
    }
    else
    {
        oled.print(F("BICEP OK"));
    }

    oled.setCursor(0, 7);
    oled.print(F("BTN: NEXT"));
}

void oledShowFault()
{
    /*
      Automatic fault screen.

      Important:
      This function intentionally does not call updateBicepSensors().

      At the moment of overcurrent detection, the current and feedback are
      latched before detaching BICEP_R. If sensors were read again after
      detaching, current could be overwritten with zero and the screen would
      lose the most relevant diagnostic value.
    */
    oled.clearDisplay();

    oled.setCursor(0, 0);
    oled.print(F("SUB1 2.1.0"));

    oled.setCursor(0, 1);
    oled.print(F("FAULT"));

    oled.setCursor(0, 2);
    oled.print(F("BICEP CURRENT"));

    oled.setCursor(0, 3);
    oled.print(F("I:"));
    oled.print(bicep_fault_current_mA);
    oled.print(F("mA"));

    oled.setCursor(0, 4);
    oled.print(F("FB:"));
    oledPrintAngle3(bicep_fault_feedback_deg);

    oled.setCursor(0, 5);
    oled.print(F("PWM DETACHED"));

    oled.setCursor(0, 7);
    oled.print(F("RESET REQUIRED"));
}

void oledShowDiagnosticScreen()
{
    /*
      Displays the diagnostic screen selected by diagnosticScreen.
    */
    if (diagnosticScreen == 0)
    {
        oledShowCommandScreen();
    }
    else
    {
        oledShowBicepScreen();
    }
}

// ============================================================================
// Button handling
// ============================================================================

bool nextButtonPressedEvent()
{
    /*
      Debounced falling-edge detector for the diagnostic button.

      The button is wired to GND and uses INPUT_PULLUP:
        HIGH = not pressed
        LOW  = pressed
    */
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
    /*
      First button press shows diagnostic screen 1/2.
      Second button press shows diagnostic screen 2/2.
      Subsequent presses alternate between both screens.

      The screen is drawn first, then the selector is advanced. This makes the
      first press after the READY screen display CMD 1/2.
    */
    oledShowDiagnosticScreen();

    if (diagnosticScreen == 0)
    {
        diagnosticScreen = 1;
    }
    else
    {
        diagnosticScreen = 0;
    }
}

// ============================================================================
// Servo group helpers
// ============================================================================

bool servoIsGroup1NonBicep(uint8_t servoIndex)
{
    /*
      Group 1 servos except BICEP_R.

      BICEP_R is excluded because it has a special feedback-based startup
      sequence.
    */
    return servoIndex == THUMB_R  ||
           servoIndex == INDEX_R  ||
           servoIndex == MIDDLE_R ||
           servoIndex == RING_R   ||
           servoIndex == PINKY_R  ||
           servoIndex == ROTATE_R;
}

bool servoIsGroup2(uint8_t servoIndex)
{
    /*
      Group 2 servos controlled by D13.
    */
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
    /*
      Returns the configured rest angle constrained to the mechanically allowed
      range. This prevents an invalid rest_deg from driving a servo outside its
      safe movement interval.
    */
    return safe_constrain_int(
        rounded_int(cfg.rest_deg),
        cfgAllowedMin(cfg),
        cfgAllowedMax(cfg)
    );
}

int16_t cfgConstrainAllowedAngle(const ServoConfig &cfg, int16_t angleDeg)
{
    /*
      Constrains a commanded target angle to the mechanically allowed range.
    */
    return safe_constrain_int(
        angleDeg,
        cfgAllowedMin(cfg),
        cfgAllowedMax(cfg)
    );
}

int16_t cfgConstrainServoAngle(const ServoConfig &cfg, int16_t angleDeg)
{
    /*
      Constrains an angle to the full calibrated servo range.

      This is used only during angle-to-PWM conversion. It is different from the
      mechanically allowed range.
    */
    return safe_constrain_int(
        angleDeg,
        cfgServoMin(cfg),
        cfgServoMax(cfg)
    );
}

int16_t cfgAngleToPwmUs(const ServoConfig &cfg, int16_t angleDeg)
{
    /*
      Converts a logical servo angle to PWM pulse width.

      Important:
      This function does not apply allowed_min_deg / allowed_max_deg.
      Mechanical limits are applied before assigning target_deg and current_deg.

      This function only maps:
        servo_min_deg..servo_max_deg
      to:
        pwm_min_us..pwm_max_us

      If cfg.inverted is true, the logical angle is mirrored within the calibrated
      servo range before PWM conversion.
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
      Computes the angular increment applied every SERVO_UPDATE_MS.

      The maximum configured speed is scaled by speedPct. A minimum step of
      1 degree/update avoids movement stalls caused by rounding to zero.
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
    /*
      Converts BICEP_R feedback ADC reading into an estimated servo angle using
      the calibration stored in servo_config_inmoov.h.
    */
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
    /*
      Converts BICEP_R current-sensor ADC reading to current in mA.

      Negative values are clamped to zero because they are not meaningful for
      this protection logic.
    */
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
    /*
      Reads and converts the BICEP_R current sensor.

      Returns zero if current sensing is not configured.
    */
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
      Updates BICEP_R feedback and current state.

      Feedback is read when group 1 power is active, because the potentiometer
      circuit is expected to be powered with that group.

      Current is only meaningful while BICEP_R is attached and driven. If the
      servo is detached, current is reported as zero during normal operation.
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
    /*
      Disables both external servo power groups.
    */
    setGroup1Power(false);
    setGroup2Power(false);
}

// ============================================================================
// Servo attach/detach and initialization
// ============================================================================

void initRuntimeServos()
{
    /*
      Initializes runtime state from the PROGMEM servo configuration table.

      Servos are not attached here. PWM attachment is performed later by the
      safe startup sequence.
    */
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
    }

    lastServoUpdate = millis();
}

void attachServoAtCurrent(uint8_t servoIndex)
{
    /*
      Attaches a servo at its current runtime angle.

      The PWM pulse is written before and after attach to reduce the chance of
      an unintended jump during attachment.
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
}

void attachServoAtRest(uint8_t servoIndex)
{
    /*
      Forces runtime state to the configured rest position, then attaches the
      servo at that position.
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
}

void detachServo(uint8_t servoIndex)
{
    /*
      Detaches one servo if it was previously attached.
    */
    if (joints[servoIndex].attached)
    {
        joints[servoIndex].servo.detach();
        joints[servoIndex].attached = false;
    }
}

void detachAllServos()
{
    /*
      Detaches all servos. Used at startup before the staged power-up sequence.
    */
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
    /*
      Safe startup sequence.

      The most delicate servo is BICEP_R because it can carry larger mechanical
      loads. For that reason, its actual feedback position is read before PWM is
      attached. The servo is then attached at the measured position and only
      afterwards commanded smoothly to rest.
    */
    mode = MODE_STARTING;

    detachAllServos();
    allPowerOff();

    /*
      Phase 1:
      Enable group 1 with no PWM active. This powers the BICEP_R feedback sensor
      and allows reading its current mechanical position without driving the
      servo.
    */
    setGroup1Power(true);
    waitWithXicro(POWER_SETTLE_MS);

    updateBicepSensors();

    /*
      Phase 2:
      Turn group 1 off before attaching PWM signals.
    */
    setGroup1Power(false);
    waitWithXicro(POWER_OFF_PAUSE_MS);

    /*
      Phase 3:
      Attach BICEP_R at the feedback-derived safe angle.
    */
    joints[BICEP_R].current_deg = bicep_feedback_deg_safe;
    joints[BICEP_R].target_deg = joints[BICEP_R].current_deg;
    attachServoAtCurrent(BICEP_R);

    /*
      Phase 4:
      Attach the remaining group 1 servos at their rest positions while group 1
      power is still off.
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
      Enable group 1. BICEP_R is now driven from its measured position toward
      its configured rest angle by the normal smooth-motion logic.
    */
    setGroup1Power(true);

    ServoConfig bcfg;
    readServoConfig(BICEP_R, bcfg);
    joints[BICEP_R].target_deg = cfgRestDeg(bcfg);

    /*
      Phase 6:
      Attach group 2 servos at rest while group 2 power is off.
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
      Enable group 2 and enter normal operation.
    */
    setGroup2Power(true);

    lastServoUpdate = millis();
    mode = MODE_RUNNING;
}

// ============================================================================
// XICRO command handling
// ============================================================================

int16_t getIncomingXicroValue(uint8_t servoIndex)
{
    /*
      Maps each runtime servo index to the corresponding XICRO subscription.

      The field names are generated by XICRO and must match the generated
      Xicro_subsys1_ID_1.h header.
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
    /*
      Reads XICRO subscription values and updates each servo target.

      Activation policy inherited from the original tested subsystem firmware:

      - Before the first non-zero command, each servo remains at rest.
      - The first non-zero command activates ROS/XICRO control for that servo.
      - After activation, command 0 means "return to rest".
    */
    if (mode != MODE_RUNNING)
    {
        return;
    }

    for (uint8_t i = 0; i < NUM_SERVOS; i++)
    {
        ServoConfig cfg;
        readServoConfig(i, cfg);

        int16_t incoming = getIncomingXicroValue(i);

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
    /*
      Checks BICEP_R overcurrent protection.

      If current remains above the configured limit for the configured time, the
      firmware latches diagnostic values, detaches BICEP_R and enters FAULT mode.
    */
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
              Latch diagnostic data before detaching the servo.

              Once the servo is detached, normal sensor update logic would set
              current to zero. The fault screen must show the actual current
              that caused the protection to trigger.
            */
            bicep_fault_current_mA = bicep_current_mA;
            bicep_fault_feedback_deg = bicep_feedback_deg_safe;

            bicep_fault_active = true;
            detachServo(BICEP_R);
            mode = MODE_FAULT;

            if (!faultScreenShown)
            {
                oledShowFault();
                faultScreenShown = true;
            }
        }
    }
    else
    {
        bicep_overcurrent_start = 0;
    }
}

void updateServoMotion(uint8_t servoIndex)
{
    /*
      Performs one smooth-motion update for the selected servo.

      The servo moves from current_deg toward target_deg by a bounded step.
      The resulting angle is converted to PWM and written using writeMicroseconds.
    */
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

    joints[servoIndex].current_deg =
        cfgConstrainAllowedAngle(cfg, joints[servoIndex].current_deg);

    joints[servoIndex].pwm_us =
        cfgAngleToPwmUs(cfg, joints[servoIndex].current_deg);

    joints[servoIndex].servo.writeMicroseconds(joints[servoIndex].pwm_us);
}

void updateAllServoMotion()
{
    /*
      Updates all servos at a fixed cadence.

      One shared timer is used for all servos to save SRAM compared with storing
      one timestamp per servo.
    */
    unsigned long now = millis();

    if (now - lastServoUpdate < SERVO_UPDATE_MS)
    {
        return;
    }

    lastServoUpdate = now;

    for (uint8_t i = 0; i < NUM_SERVOS; i++)
    {
        updateServoMotion(i);
    }
}

// ============================================================================
// Setup
// ============================================================================

void setup()
{
    /*
      Power control pins are initialized immediately and both servo groups are
      disabled before anything else.
    */
    pinMode(SERVO_GROUP_1_PIN, OUTPUT);
    pinMode(SERVO_GROUP_2_PIN, OUTPUT);
    allPowerOff();

    /*
      Configure diagnostic button with internal pull-up.
    */
    pinMode(NEXT_BUTTON_PIN, INPUT_PULLUP);
    lastButtonReading = digitalRead(NEXT_BUTTON_PIN);
    stableButtonState = lastButtonReading;
    lastDebounceTime = millis();

    /*
      AREF is externally tied to 3.3 V.
      This must be selected before any analogRead().
    */
    analogReference(EXTERNAL);

    /*
      Start XICRO communication.
      Serial must not be used for debug prints because it is dedicated to XICRO.
    */
    Serial.begin(XICRO_BAUDRATE);
    xicro.begin(&Serial);

    /*
      Initialize OLED once.
      No periodic refresh is performed in loop().
    */
    Wire.begin();
    oled.begin();
    oled.setPowerSave(0);
    oled.setFont(u8x8_font_chroma48medium8_r);
    oledShowBoot();

    /*
      Initialize servo configuration and runtime state.
    */
    readBicepConfig();
    initRuntimeServos();

    /*
      Initial stabilization while keeping XICRO alive.
    */
    waitWithXicro(INITIAL_STABILIZE_MS);

    /*
      Controlled servo startup.
    */
    performStagedStartupToRest();

    /*
      Final base screen.
      After this point, OLED is updated only on button press or fault.
    */
    oledShowReady();
}

// ============================================================================
// Main loop
// ============================================================================

void loop()
{
    /*
      Service XICRO communication.
    */
    xicro.Spin_node();

    /*
      Update servo targets from ROS 2/XICRO subscription values.
    */
    updateTargetsFromXicro();

    /*
      Move servos smoothly and check BICEP_R protection.
    */
    updateAllServoMotion();

    /*
      Update OLED only if the diagnostic button generates a debounced press
      event.
    */
    if (nextButtonPressedEvent())
    {
        handleButtonPress();
    }
}
