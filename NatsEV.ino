/*
 ╔══════════════════════════════════════════════════════════════╗
 ║  S-CURVE ROUTE — Pico 2W WEI EV Version2                     ║
 ║                                                              ║
 ║  Ported from Tektite RotEv S-curve code.                     ║
 ║  Smooth sinusoidal lane changes, no arc segments.            ║
 ║                                                              ║
 ║  Path: LANE_CHANGE_OUT → MID_STRAIGHT → LANE_CHANGE_BACK     ║
 ║                                                              ║
 ║  Lane change out:  y(x) = (d/2)(1 - cos(π·x/Lc))             ║
 ║  Mid straight:     y = d                                     ║
 ║  Lane change back: y = d - (d/2)(1 - cos(π·(x-Lc-mid)/Lc))   ║
 ║                                                              ║
 ║  Hardware:                                                   ║
 ║    MCU   : Pico 2W (arduino-pico)                            ║
 ║    IMU   : BNO085 on I2C1 (SDA=GP26, SCL=GP27)               ║
 ║    Motor : TB6612  L(A): PWM=17 AO1=19 AO2=18                ║
 ║                    R(B): PWM=22 BO1=20 BO2=21 (dir swapped)  ║
 ║    Enc   : L: A=GP7  B=GP8    R: A=GP9  B=GP10               ║
 ║    Buttons: GO=GP15  STOP=GP14 (active LOW)                  ║
 ║                                                              ║
 ║  Removed: aiming phase, gyro calibration, RGB LEDs           ║
 ║  Added: BNO085 game rotation vector, deferred serial print   ║
 ║                                                              ║
 ║  Libraries: Adafruit BNO08x                                  ║
 ╚══════════════════════════════════════════════════════════════╝
 */

#include <math.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// Forward declaration — Arduino IDE generates function prototypes
// at the top of the file, so this enum must appear before any
// function that uses State as a parameter type.
enum State {
  IDLE,
  LANE_CHANGE_OUT,
  MID_STRAIGHT,
  LANE_CHANGE_BACK,
  DONE
};

enum LedMode {
  LED_INIT,     // fast blink — setup/BNO connecting
  LED_READY,    // heartbeat pulse — waiting for GO
  LED_RUNNING,  // solid on — run in progress
  LED_DONE,     // slow blink — finished normal
  LED_ERROR     // very fast blink — abnormal stop
};


// ═══════════════════════════════════════════════════════════════
//  PINS - Hardware Set Up
// ═══════════════════════════════════════════════════════════════

#define IMU_SDA 26
#define IMU_SCL 27
#define PWMA 17
#define AO1 19
#define AO2 18
#define PWMB 22
#define BO1 20
#define BO2 21
#define L_ENC_A 7
#define L_ENC_B 8
#define R_ENC_A 9
#define R_ENC_B 10
#define BTN_GO 15
#define BTN_STOP 14

// ═══════════════════════════════════════════════════════════════
//  COMPETITION PARAMETERS  (edit these for each run)
// ═══════════════════════════════════════════════════════════════

float TARGET_DISTANCE_M = 9.68f;  // total forward distance (m)
float TARGET_TIME_S = 10.50f;      // target run time (s)
float STOP_OFFSET_M = 0.0f;        // shorten end to compensate if any coast
float CAN_GAP = 0.10f;             // gap between cans (m)
float MID_STRAIGHT_M = 0.50f;      // straight section at offset (m)

// ═══════════════════════════════════════════════════════════════
//  VEHICLE CONSTANTS
// ═══════════════════════════════════════════════════════════════
// 1-5/8" wheel diameter
const float WHEEL_DIAM_M = 0.041275f * (6.964f / 7.0f) * (9.98f / 10.0f);  //was 0.041275 per wheel size 1 5/8inch.  adjusted based on straight run.
const float WHEEL_RADIUS_M = WHEEL_DIAM_M * 0.5f;
const float GEAR_RATIO = 1.0f;
const float TRACK_WIDTH_M = 0.049f;  // measured for Wei EV, this parameter is actually not used in this code, as only using BNO for heading

// Encoder: rising edge on A only, 3 CPR × 51.45 gear ratio
const float TICKS_PER_REV = 154.35f;
const float DIST_PER_TICK = (PI * WHEEL_DIAM_M) / TICKS_PER_REV;

// ═══════════════════════════════════════════════════════════════
//  HEADING PID
// ═══════════════════════════════════════════════════════════════
const float KP_HEADING = 0.027f;
const float KD_HEADING = 0.0013f;
const float KI_HEADING = 0.005f;
const float HEADING_I_MAX = 0.05f;
float heading_integral = 0.0f;
const float KFF_CURV = 0.003f;  // curvature feedforward gain
const float KP_CROSS = 20.0f;   //factor for heading correction based on lateral offset from target y

// ═══════════════════════════════════════════════════════════════
//  SPEED PID
// ═══════════════════════════════════════════════════════════════
const float KP_SPEED = 0.50f;
const float KI_SPEED = 0.80f;
const float SPEED_I_MAX = 0.15f;
float speed_integral = 0.0f;

// ═══════════════════════════════════════════════════════════════
//  RAMP DISTANCE AND SPEED FOR PATH PLANNING
// ═══════════════════════════════════════════════════════════════
const float RAMP_DIST_MIN = 0.40f;
const float RAMP_UP_MIN_SPEED = 0.20f;      // m/s
const float RAMP_DOWN_MIN_SPEED = 0.12f;    // m/s
const float RAMP_FRAC = 0.25f;              //m, use this part of the lane curve out as ramp up distance.  the smaller, the shorter ramp up/down
float RAMP_DIST_M;                          // ramp up and down distance, in meters
float CRUISE_SPEED = 0.65f;                 // only a starting piont, it will get updated by computeSpeeds()
const float V_MAX_CRUISE = 1.40f;           //cap of max cruise speed, m/s
const float TRANSITION_OVERHEAD_S = -0.5f;  //The vehicle's ramp is faster than the theoretical rampTime() predicts because the speed PID overshoots

// ═══════════════════════════════════════════════════════════════
//  MOTOR / ENCODER SIGNS  (verify with sensor test)
// ═══════════════════════════════════════════════════════════════

const int ENC_L_SIGN = 1;   // +1 if left ticks go + for forward
const int ENC_R_SIGN = 1;   // +1 if right ticks go + for forward
const int MOTOR_L_DIR = 1;  // motor direction (handled in motorWrite)
const int MOTOR_R_DIR = 1;


// ═══════════════════════════════════════════════════════════════
//  MOTOR BIAS
// ═══════════════════════════════════════════════════════════════
const float MOTOR_BIAS_L = 1.00f;
float MOTOR_BIAS_R = 1.00f;  // updated by computeSpeeds(), as our right motor is a bit weaker

float computeMotorBiasR(float cruise_speed) {
  float bias = 1.04f - 0.03f * cruise_speed;
  return constrain(bias, 1.00f, 1.05f);
}

// ═══════════════════════════════════════════════════════════════
//  SPEED → PWR RELATIONSHIP
//  May 2026 Test Results for WEI EV version2
// ═══════════════════════════════════════════════════════════════

const float PWR_BIAS = 0.111f;   // ← updated after velocity test on 5/6
const float PWR_SLOPE = 0.592f;  // ← updated after velocity test on 5/6
const float PWR_MIN = 0.15f;     //15% to get the EV to move
const float PWR_MAX = 0.90f;     //90% get to 1.35m/s speed

// ═══════════════════════════════════════════════════════════════
//  TIMING
// ═══════════════════════════════════════════════════════════════

const uint32_t LOOP_US = 2000;  // 5 ms control loop


// ═══════════════════════════════════════════════════════════════
//  LED STATUS
// ═══════════════════════════════════════════════════════════════
LedMode ledMode = LED_INIT;
uint32_t led_timer_ms = 0;
bool led_on = false;

void ledSetMode(LedMode m) {
  ledMode = m;
  led_timer_ms = millis();
  led_on = false;
}

void ledUpdate() {
  uint32_t elapsed = millis() - led_timer_ms;

  switch (ledMode) {
    case LED_INIT:
      // Fast blink: 80ms on/off
      if (elapsed >= 80) {
        led_timer_ms = millis();
        led_on = !led_on;
      }
      break;

    case LED_READY:
      // Heartbeat: brief flash every 2 seconds
      if (!led_on && elapsed >= 2000) {
        led_on = true;
        led_timer_ms = millis();
      } else if (led_on && elapsed >= 300) {
        led_on = false;
        led_timer_ms = millis();
      }
      break;

    case LED_RUNNING:
      // Solid on
      led_on = true;
      break;

    case LED_DONE:
      // Slow blink: 500ms on/off
      //if (elapsed >= 500) { led_timer_ms = millis(); led_on = !led_on; }
      if (!led_on && elapsed >= 2000) {
        led_on = true;
        led_timer_ms = millis();
      } else if (led_on && elapsed >= 300) {
        led_on = false;
        led_timer_ms = millis();
      }
      break;

    case LED_ERROR:
      // Very fast blink: 80ms on/off
      if (elapsed >= 80) {
        led_timer_ms = millis();
        led_on = !led_on;
      }
      break;
  }

  digitalWrite(LED_BUILTIN, led_on);
}


// ═══════════════════════════════════════════════════════════════
//  BNO085 - initialization and polling data
// ═══════════════════════════════════════════════════════════════

Adafruit_BNO08x bno(-1);
sh2_SensorValue_t _evt;
float yawDeg = 0.0f;      // raw heading from BNO085
float yawRateDPS = 0.0f;  // yaw rate from calibrated gyro

bool initIMU() {
  Wire1.setSDA(IMU_SDA);
  Wire1.setSCL(IMU_SCL);
  Wire1.begin();
  if (!bno.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire1)) return false;
  if (!bno.enableReport(SH2_GAME_ROTATION_VECTOR, 5000)) return false;
  if (!bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 5000)) return false;
  return true;
}

float quatToYaw(float qi, float qj, float qk, float qr) {
  float siny = 2.0f * (qr * qk + qi * qj);
  float cosy = 1.0f - 2.0f * (qj * qj + qk * qk);
  return atan2(siny, cosy) * 180.0f / PI;
}

void pollIMU() {
  while (bno.getSensorEvent(&_evt)) {
    switch (_evt.sensorId) {
      case SH2_GAME_ROTATION_VECTOR:
        yawDeg = quatToYaw(_evt.un.gameRotationVector.i,
                           _evt.un.gameRotationVector.j,
                           _evt.un.gameRotationVector.k,
                           _evt.un.gameRotationVector.real);
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        yawRateDPS = _evt.un.gyroscope.z * (180.0f / PI);
        break;
    }
  }
}

// ═══════════════════════════════════════════════════════════════
//  ENCODERS
// ═══════════════════════════════════════════════════════════════

volatile long leftTicks = 0;
volatile long rightTicks = 0;

void leftISR() {
  digitalRead(L_ENC_B) ? leftTicks++ : leftTicks--;
}
void rightISR() {
  digitalRead(R_ENC_B) ? rightTicks++ : rightTicks--;
}

// ═══════════════════════════════════════════════════════════════
//  MOTORS
// ═══════════════════════════════════════════════════════════════

void motorLeft(float pwr) {
  pwr = constrain(pwr, -1.0f, 1.0f);
  int duty = (int)(fabsf(pwr) * 255);
  if (pwr > 0.001f) {
    digitalWrite(AO1, HIGH);
    digitalWrite(AO2, LOW);
  } else if (pwr < -0.001f) {
    digitalWrite(AO1, LOW);
    digitalWrite(AO2, HIGH);
  } else {
    digitalWrite(AO1, HIGH);
    digitalWrite(AO2, HIGH);
    duty = 0;
  }
  analogWrite(PWMA, duty);
}

void motorRight(float pwr) {
  pwr = constrain(pwr, -1.0f, 1.0f);
  int duty = (int)(fabsf(pwr) * 255);
  if (pwr > 0.001f) {
    digitalWrite(BO1, LOW);
    digitalWrite(BO2, HIGH);
  }  // swapped
  else if (pwr < -0.001f) {
    digitalWrite(BO1, HIGH);
    digitalWrite(BO2, LOW);
  }  // swapped
  else {
    digitalWrite(BO1, HIGH);
    digitalWrite(BO2, HIGH);
    duty = 0;
  }
  analogWrite(PWMB, duty);
}

void stopMotors() {
  motorLeft(0);
  motorRight(0);
}

// ═══════════════════════════════════════════════════════════════
//  BUTTONS
// ═══════════════════════════════════════════════════════════════

bool goButtonPressed() {
  return !digitalRead(BTN_GO);
}
bool stopButtonPressed() {
  return !digitalRead(BTN_STOP);
}

// ═══════════════════════════════════════════════════════════════
//  ONBOARD LED
// ═══════════════════════════════════════════════════════════════

void ledFlash() {
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_BUILTIN, i % 2);
    delay(100);
  }
  digitalWrite(LED_BUILTIN, LOW);
}

// ═══════════════════════════════════════════════════════════════
//  STATE MACHINE
// ═══════════════════════════════════════════════════════════════

State state = IDLE;

const char *state_names[] = {
  "IDLE", "LC_OUT", "MID", "LC_BACK", "DONE"
};

// ═══════════════════════════════════════════════════════════════
//  PATH GEOMETRY - NO NEED USER UPDATE, COMPUTED FROM TARGET DIST AND CAN GAP
// ═══════════════════════════════════════════════════════════════

float OFFSET_M = 1.0f;   // computed from CAN_GAP
float Lc_m = 0.0f;       // lane change length
float pathLen_m = 0.0f;  // actual path length
float peakCurvature = 0.0f;
float xThresh[4];

// ═══════════════════════════════════════════════════════════════
//  GLOBALS
// ═══════════════════════════════════════════════════════════════

bool goPressed = false;
bool stopPressed = false;
bool going = false;

uint32_t run_start_us = 0;
uint32_t last_loop_us = 0;
uint32_t last_print_ms = 0;

float state_start_distance = 0.0f;
float target_heading_deg = 0.0f;
float seg_start_dist = 0.0f;

// Heading: BNO085 heading zeroed at run start
float headingDeg = 0.0f;
float yawRateDeg = 0.0f;
float startYaw = 0.0f;        // BNO085 heading at run start
float prevHeadingDeg = 0.0f;  // for midpoint position update

float dist_L = 0.0f, dist_R = 0.0f;  //left and right wheel travel distance
float distance = 0.0f;
float velocity = 0.0f;
float x_track = 0.0f, y_track = 0.0f;  //track x and y position during the run

// Encoder state
long prevTickL = 0, prevTickR = 0;
bool enc_initialized = false;

// Coast tracking
float coast_dx = 0.0f, coast_dy = 0.0f;
float coast_final_x = 0.0f, coast_final_y = 0.0f;
float actual_runtime_s = 0.0f;

float last_turn = 0.0f;  //for logging purpose, to log the turn value

// ═══════════════════════════════════════════════════════════════
//  UTILITY FUNCTIONS
// ═══════════════════════════════════════════════════════════════

static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline float wrapDeg(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

static inline float deg2rad(float d) {
  return d * (float)M_PI / 180.0f;
}
static inline float rad2deg(float r) {
  return r * 180.0f / (float)M_PI;
}

float elapsedS() {
  return (micros() - run_start_us) / 1e6f;
}

// ═══════════════════════════════════════════════════════════════
//  S-CURVE MATH
// ═══════════════════════════════════════════════════════════════

float scurveSlope(float x) {
  if (x <= 0.0f || x >= TARGET_DISTANCE_M) return 0.0f;
  float piLc = (float)M_PI / Lc_m;
  if (x <= Lc_m) {
    return (OFFSET_M / 2.0f) * piLc * sinf(piLc * x);
  } else if (x <= Lc_m + MID_STRAIGHT_M) {
    return 0.0f;
  } else {
    float xr = x - Lc_m - MID_STRAIGHT_M;
    return -(OFFSET_M / 2.0f) * piLc * sinf(piLc * xr);
  }
}

float scurveHeadingDeg(float x) {
  return rad2deg(atanf(scurveSlope(x)));
}

float scurveCurvature(float x) {
  if (x <= 0.0f || x >= TARGET_DISTANCE_M) return 0.0f;
  float dydx, d2ydx2;
  float piLc = (float)M_PI / Lc_m;

  if (x <= Lc_m) {
    dydx = (OFFSET_M / 2.0f) * piLc * sinf(piLc * x);
    d2ydx2 = (OFFSET_M / 2.0f) * piLc * piLc * cosf(piLc * x);
  } else if (x <= Lc_m + MID_STRAIGHT_M) {
    return 0.0f;
  } else {
    float xr = x - Lc_m - MID_STRAIGHT_M;
    dydx = -(OFFSET_M / 2.0f) * piLc * sinf(piLc * xr);
    d2ydx2 = -(OFFSET_M / 2.0f) * piLc * piLc * cosf(piLc * xr);
  }
  float denom = powf(1.0f + dydx * dydx, 1.5f);
  return d2ydx2 / denom;
}

float scurveY(float x) {
  if (x <= 0.0f) return 0.0f;
  if (x >= TARGET_DISTANCE_M) return 0.0f;
  if (x <= Lc_m) {
    return (OFFSET_M / 2.0f) * (1.0f - cosf((float)M_PI * x / Lc_m));
  } else if (x <= Lc_m + MID_STRAIGHT_M) {
    return OFFSET_M;
  } else {
    float xr = x - Lc_m - MID_STRAIGHT_M;
    return OFFSET_M - (OFFSET_M / 2.0f) * (1.0f - cosf((float)M_PI * xr / Lc_m));
  }
}

// ═══════════════════════════════════════════════════════════════
//  STATE TRANSITION LOG  (deferred print)
// ═══════════════════════════════════════════════════════════════

const int MAX_STATE_LOGS = 12;

struct StateLogEntry {
  State fromState, toState;
  float t_s, dist_m, x_m, y_m;
  float x_thresh_m, x_err_m;
  float heading_deg, target_deg, heading_err_deg;
  float velocity_mps;
  float curvature;
  float y_target;
  float y_err;
};

StateLogEntry stateLogs[MAX_STATE_LOGS];
int stateLogCount = 0;
bool runLogReady = false;
bool runLogPrinted = false;


// ═══════════════════════════════════════════════════════════════
//  TELEMETRY LOG  (deferred print, ~50ms intervals)
// ═══════════════════════════════════════════════════════════════

const int MAX_TLOG = 500;
struct TelemetryEntry {
  float t_s, x, y, y_target, y_err;
  float heading, heading_target, heading_err;
  float velocity, turn_cmd;
};
TelemetryEntry tlog[MAX_TLOG];
int tlogCount = 0;
uint32_t last_tlog_ms = 0;


int threshIndexForState(State s) {
  switch (s) {
    case LANE_CHANGE_OUT: return 0;
    case MID_STRAIGHT: return 1;
    case LANE_CHANGE_BACK: return 2;
    default: return 0;
  }
}

void clearStateLogs() {
  stateLogCount = 0;
  runLogReady = false;
  runLogPrinted = false;
}

void logStateChange(State fromS, State toS) {
  if (stateLogCount >= MAX_STATE_LOGS) return;
  StateLogEntry &e = stateLogs[stateLogCount++];

  int ti = threshIndexForState(fromS);

  e.fromState = fromS;
  e.toState = toS;
  e.t_s = (fromS == IDLE) ? 0.0f : elapsedS();
  e.dist_m = distance;
  e.x_m = x_track;
  e.y_m = y_track;
  e.x_thresh_m = xThresh[ti];
  e.x_err_m = x_track - xThresh[ti];
  e.heading_deg = headingDeg;
  e.target_deg = target_heading_deg;
  e.heading_err_deg = wrapDeg(target_heading_deg - headingDeg);
  e.velocity_mps = velocity;
  e.curvature = scurveCurvature(x_track);
  e.y_target = scurveY(x_track);
  e.y_err = y_track - e.y_target;
}

void printStateLogs() {
  Serial.println();
  Serial.println("════════════════════════════════════════════════════════");
  Serial.println("  S-CURVE RUN LOG");
  Serial.println("════════════════════════════════════════════════════════");
  Serial.printf("  Target dist: %.2f m   Target time: %.1f s\n",
                TARGET_DISTANCE_M, TARGET_TIME_S);
  Serial.printf("  Offset: %.3f m   MidStraight: %.3f m   Lc: %.3f m\n",
                OFFSET_M, MID_STRAIGHT_M, Lc_m);
  Serial.printf("  Path length: %.3f m   Cruise speed: %.3f m/s\n",
                pathLen_m, CRUISE_SPEED);
  Serial.printf("  Ramp dist: %.3f m   Motor bias R: %.4f\n",
                RAMP_DIST_M, MOTOR_BIAS_R);
  Serial.println();

  for (int i = 0; i < stateLogCount; i++) {
    const StateLogEntry &e = stateLogs[i];
    Serial.printf("%d) %s -> %s\n", i, state_names[e.fromState], state_names[e.toState]);
    Serial.printf("   t=%.3f s  dist=%.3f m  vel=%.3f m/s\n", e.t_s, e.dist_m, e.velocity_mps);
    Serial.printf("   x=%.3f  xThresh=%.3f  xErr=%.3f m\n", e.x_m, e.x_thresh_m, e.x_err_m);
    Serial.printf("   y=%.3f  yTarget=%.3f  yErr=%.3f m\n", e.y_m, e.y_target, e.y_err);
    Serial.printf("   hdg=%.2f  target=%.2f  err=%.2f deg\n",
                  e.heading_deg, e.target_deg, e.heading_err_deg);
    Serial.printf("   curvature=%.4f 1/m\n", e.curvature);
    Serial.println();
  }

  Serial.println("────────────────────────────────────────────────────────");
  Serial.printf("  Coast dx: %.4f m   dy: %.4f m\n", coast_dx, coast_dy);
  Serial.printf("  Final x (w/coast): %.3f m\n", coast_final_x);
  Serial.printf("  Final y (w/coast): %.3f m\n", coast_final_y);
  Serial.printf("  Total runtime: %.3f s\n", actual_runtime_s);
  Serial.println("════════════════════════════════════════════════════════");
  Serial.println();

  Serial.println("──── TELEMETRY LOG ────");
  Serial.println("  t,      x,      y,     yT,     yE,    hdg,   hdgT,   hdgE,   vel,   turn");
  for (int i = 0; i < tlogCount; i++) {
    const TelemetryEntry &e = tlog[i];
    Serial.printf("%6.3f, %6.3f, %7.3f, %6.3f, %7.4f, %7.2f, %7.2f, %7.2f, %6.3f, %7.4f\n",
                  e.t_s, e.x, e.y, e.y_target, e.y_err,
                  e.heading, e.heading_target, e.heading_err,
                  e.velocity, e.turn_cmd);
  }
}
// print run log when connected by USB
void serviceDeferredLogPrint() {
  if (runLogReady && !runLogPrinted && Serial) {
    delay(200);
    printStateLogs();
    runLogPrinted = true;
  }
}

// ═══════════════════════════════════════════════════════════════
//  IMU + ENCODER UPDATE
//  Uses BNO085 heading directly (no gyro integration needed)
//  Encoders provide distance + velocity only,not used for heading
// ═══════════════════════════════════════════════════════════════

void updateIMUandEncoders(float dt) {
  // --- Heading from BNO085 (zeroed at run start) ---
  float newHeading = wrapDeg(yawDeg - startYaw);
  yawRateDeg = yawRateDPS;  // gyro rate for D term

  // --- Encoder distance ---
  noInterrupts();
  long curL = leftTicks;
  long curR = rightTicks;
  interrupts();

  if (!enc_initialized) {
    prevTickL = curL;
    prevTickR = curR;
    enc_initialized = true;
    headingDeg = newHeading;
    prevHeadingDeg = newHeading;
    return;
  }

  long dTickL = (curL - prevTickL) * ENC_L_SIGN;
  long dTickR = (curR - prevTickR) * ENC_R_SIGN;
  prevTickL = curL;
  prevTickR = curR;

  float wheel_dL = (float)dTickL * DIST_PER_TICK;
  float wheel_dR = (float)dTickR * DIST_PER_TICK;

  dist_L += wheel_dL;
  dist_R += wheel_dR;
  distance = 0.5f * (dist_L + dist_R);

  float ds = 0.5f * (wheel_dL + wheel_dR);
  if (dt > 0) velocity = ds / dt;

  // --- Position update using midpoint heading ---
  float dHeading = wrapDeg(newHeading - prevHeadingDeg);
  float midHeading = prevHeadingDeg + 0.5f * dHeading;
  float hMidRad = deg2rad(midHeading);
  x_track += ds * cosf(hMidRad);
  y_track += ds * sinf(hMidRad);

  prevHeadingDeg = newHeading;
  headingDeg = newHeading;
}

// ═══════════════════════════════════════════════════════════════
//  DRIVE FUNCTIONS
// ═══════════════════════════════════════════════════════════════

void driveSpeed(float speed_mps, float heading_target_deg, float dt) {
  float base_ff = PWR_BIAS + PWR_SLOPE * speed_mps;  //base speed to PWR

  float speed_err = speed_mps - velocity;  //velocity track speed at the moment.
  speed_integral += speed_err * dt;
  speed_integral = clampf(speed_integral, -SPEED_I_MAX, SPEED_I_MAX);
  float base = base_ff + KP_SPEED * speed_err + KI_SPEED * speed_integral;
  base = clampf(base, PWR_MIN, PWR_MAX);  //bass is updated based on speed PID

  float errDeg = wrapDeg(heading_target_deg - headingDeg);
  heading_integral += errDeg * dt;
  heading_integral = clampf(heading_integral, -HEADING_I_MAX, HEADING_I_MAX);
  float turn = KP_HEADING * errDeg + KI_HEADING * heading_integral - KD_HEADING * yawRateDeg;
  turn = clampf(turn, -0.30f, 0.30f);  // turn is updated based on heading PID

  float left = clampf(base - turn, -PWR_MAX, PWR_MAX);
  float right = clampf(base + turn, -PWR_MAX, PWR_MAX);

  motorRight(right * MOTOR_R_DIR * MOTOR_BIAS_R);
  motorLeft(left * MOTOR_L_DIR * MOTOR_BIAS_L);
}

//drivespeed, including feed forward for curve following
void driveSpeedFF(float speed_mps, float heading_target_deg, float ff_turn, float dt) {
  float base_ff = PWR_BIAS + PWR_SLOPE * speed_mps;

  float speed_err = speed_mps - velocity;
  speed_integral += speed_err * dt;
  speed_integral = clampf(speed_integral, -SPEED_I_MAX, SPEED_I_MAX);
  float base = base_ff + KP_SPEED * speed_err + KI_SPEED * speed_integral;
  base = clampf(base, PWR_MIN, PWR_MAX);

  float errDeg = wrapDeg(heading_target_deg - headingDeg);
  heading_integral += errDeg * dt;
  heading_integral = clampf(heading_integral, -HEADING_I_MAX, HEADING_I_MAX);
  float turn = KP_HEADING * errDeg + KI_HEADING * heading_integral
               - KD_HEADING * yawRateDeg + ff_turn;
  turn = clampf(turn, -0.30f, 0.30f);
  last_turn = turn;

  float left = clampf(base - turn, -PWR_MAX, PWR_MAX);
  float right = clampf(base + turn, -PWR_MAX, PWR_MAX);

  motorRight(right * MOTOR_R_DIR * MOTOR_BIAS_R);
  motorLeft(left * MOTOR_L_DIR * MOTOR_BIAS_L);
}

//drive S curve with cross track correction for y
void driveSCurve(float speed_mps, float dt) {
  float hdgTarget = scurveHeadingDeg(x_track);
  // Cross-track correction
  float y_err = scurveY(x_track) - y_track;
  hdgTarget += KP_CROSS * y_err;

  target_heading_deg = hdgTarget;

  float kappa = scurveCurvature(x_track);
  float expectedYawDegS = rad2deg(speed_mps * kappa);
  float ff_turn = KFF_CURV * expectedYawDegS;

  driveSpeedFF(speed_mps, hdgTarget, ff_turn, dt);
}

// ═══════════════════════════════════════════════════════════════
//  SPEED CALCULATOR  (two-pass with correct ramp time)
//
//  The ramp is linear in DISTANCE: speed(x) = V_min + (V_cruise - V_min) * (x / RAMP_DIST)
//  Time for distance-linear ramp: integrate dt = dx / v(x)
//    t_ramp = integral_0^D [ dx / (V_min + (V_cruise - V_min) * x/D) ]
//           = D / (V_cruise - V_min) * ln(V_cruise / V_min)
//  This is slower than the linear-in-time average (0.5*(V_min+V_cruise))
//  because the vehicle spends more time at low speed.
// ═══════════════════════════════════════════════════════════════
// ═══════════════════════════════════════════════════════════════
//  ARC LENGTH HELPER
// ═══════════════════════════════════════════════════════════════

float arcLengthBetween(float x0, float x1) {
  const int N = 200;
  float dx = (x1 - x0) / N;
  float len = 0.0f;
  for (int i = 0; i < N; i++) {
    float x = x0 + (i + 0.5f) * dx;
    float slope = scurveSlope(x);
    len += sqrtf(1.0f + slope * slope) * dx;
  }
  return len;
}

// ═══════════════════════════════════════════════════════════════
//  RAMP TIME (exact for distance-linear speed ramp)
//
//  Speed profile: v(s) = V_min + (V_cruise - V_min) * s / D
//  Time: t = integral_0^D ds/v(s) = D/(V_c - V_min) * ln(V_c/V_min)
// ═══════════════════════════════════════════════════════════════

float rampTime(float ramp_dist, float v_min, float v_cruise) {
  if (v_cruise <= v_min + 0.01f) return ramp_dist / v_min;
  return ramp_dist / (v_cruise - v_min) * logf(v_cruise / v_min);
}

// ═══════════════════════════════════════════════════════════════
//  SPEED CALCULATOR
//
//  All distances in arc-length (what the wheels actually travel).
//  Ramp boundaries defined in x-distance (for state machine),
//  then converted to arc-length for time/speed calculation.
//  Iterates 5 passes to converge cruise speed.
// ═══════════════════════════════════════════════════════════════

void computeSpeeds() {
  // --- Total path arc length ---
  pathLen_m = arcLengthBetween(0.0f, TARGET_DISTANCE_M);

  // --- Ramp x-distances (unchanged, used by state machine) ---
  RAMP_DIST_M = clampf(RAMP_FRAC * Lc_m, RAMP_DIST_MIN, 0.50f * Lc_m);
  float rampDn_xDist = min(RAMP_DIST_M, 0.50f * Lc_m);

  // --- Convert ramp x-boundaries to arc lengths ---
  float arcRampUp = arcLengthBetween(0.0f, RAMP_DIST_M);
  float arcRampDn = arcLengthBetween(TARGET_DISTANCE_M - rampDn_xDist, TARGET_DISTANCE_M);
  float arcCruise = pathLen_m - arcRampUp - arcRampDn;

  // --- Initial estimate ---
  CRUISE_SPEED = clampf(pathLen_m / TARGET_TIME_S, 0.25f, V_MAX_CRUISE);
  //float overhead = -1.1f * CRUISE_SPEED + 0.2f;  //fitted from runs

  // --- Iterate to converge ---
  for (int pass = 0; pass < 5; pass++) {
    //overhead = -1.1f * CRUISE_SPEED + 0.2f;
    float rampUp_time = rampTime(arcRampUp, RAMP_UP_MIN_SPEED, CRUISE_SPEED);
    float rampDn_time = rampTime(arcRampDn, RAMP_DOWN_MIN_SPEED, CRUISE_SPEED);

    float cruise_time = TARGET_TIME_S - rampUp_time - rampDn_time - TRANSITION_OVERHEAD_S;
    if (cruise_time > 0.1f) {
      CRUISE_SPEED = clampf(arcCruise / cruise_time, 0.25f, V_MAX_CRUISE);
    }
  }

  // --- Motor bias from final cruise speed ---
  MOTOR_BIAS_R = computeMotorBiasR(CRUISE_SPEED);

  // --- Print results ---
  Serial.printf("  Path length:  %.3f m\n", pathLen_m);
  Serial.printf("  Ramp dist (x): %.3f m\n", RAMP_DIST_M);
  Serial.printf("  Arc ramp up:  %.3f m   Arc ramp dn: %.3f m   Arc cruise: %.3f m\n",
                arcRampUp, arcRampDn, arcCruise);
  Serial.printf("  Cruise speed: %.3f m/s\n", CRUISE_SPEED);
  Serial.printf("  Motor bias R: %.4f\n", MOTOR_BIAS_R);

  // Debug: show ramp times
  float rampUp_t = rampTime(arcRampUp, RAMP_UP_MIN_SPEED, CRUISE_SPEED);
  float rampDn_t = rampTime(arcRampDn, RAMP_DOWN_MIN_SPEED, CRUISE_SPEED);
  Serial.printf("  Ramp-up time: %.3f s   Ramp-dn time: %.3f s\n", rampUp_t, rampDn_t);
  Serial.printf("  Cruise time:  %.3f s\n", TARGET_TIME_S - rampUp_t - rampDn_t - TRANSITION_OVERHEAD_S);
}


// ═══════════════════════════════════════════════════════════════
//  STATE MACHINE HELPERS
// ═══════════════════════════════════════════════════════════════

void enterState(State s) {
  logStateChange(state, s);
  state = s;
  state_start_distance = distance;
  seg_start_dist = distance;
  // NOTE: speed_integral and heading_integral NOT reset
  // S-curve has no heading jumps — bumpless transfer
}

// ═══════════════════════════════════════════════════════════════
//  ROUTE PLANNER
// ═══════════════════════════════════════════════════════════════

bool planRoute() {
  OFFSET_M = 1.0f - CAN_GAP / 2.0f;
  Lc_m = (TARGET_DISTANCE_M - MID_STRAIGHT_M) / 2.0f;

  if (Lc_m < 0.5f) {
    Serial.println("PLAN FAIL: TARGET_DISTANCE too short.");
    return false;
  }

  float piLc = (float)M_PI / Lc_m;
  peakCurvature = (OFFSET_M / 2.0f) * piLc * piLc;

  xThresh[0] = Lc_m;
  xThresh[1] = Lc_m + MID_STRAIGHT_M;
  xThresh[2] = TARGET_DISTANCE_M;

  float peakSlope = (OFFSET_M / 2.0f) * piLc;
  float peakHeadDeg = rad2deg(atanf(peakSlope));

  Serial.println("\n=== S-CURVE ROUTE PLAN ===");
  Serial.printf("  Distance: %.2f m   Offset: %.3f m\n", TARGET_DISTANCE_M, OFFSET_M);
  Serial.printf("  Lc: %.3f m   Mid: %.3f m\n", Lc_m, MID_STRAIGHT_M);
  Serial.printf("  Peak curvature: %.4f 1/m   (R=%.2f m)\n",
                peakCurvature, 1.0f / peakCurvature);
  Serial.printf("  Peak heading: %.1f deg\n", peakHeadDeg);
  Serial.printf("  LC_OUT end:   x=%.3f\n", xThresh[0]);
  Serial.printf("  MID end:      x=%.3f\n", xThresh[1]);
  Serial.printf("  LC_BACK end:  x=%.3f\n", xThresh[2]);
  Serial.println("==========================\n");

  return true;
}

// ═══════════════════════════════════════════════════════════════
//  RESET RUN
// ═══════════════════════════════════════════════════════════════

void resetRun() {
  clearStateLogs();
  enc_initialized = false;
  prevTickL = prevTickR = 0;
  dist_L = dist_R = 0.0f;
  distance = 0.0f;
  velocity = 0.0f;
  x_track = 0.0f;
  y_track = 0.0f;
  speed_integral = 0.0f;
  heading_integral = 0.0f;
  run_start_us = micros();
  last_loop_us = run_start_us;

  state = IDLE;
  state_start_distance = 0.0f;
  target_heading_deg = 0.0f;

  noInterrupts();
  leftTicks = rightTicks = 0;
  interrupts();

  // Zero heading at current BNO085 reading
  pollIMU();
  startYaw = yawDeg;
  headingDeg = 0.0f;
  prevHeadingDeg = 0.0f;

  tlogCount = 0;
  last_tlog_ms = 0;
}

// ═══════════════════════════════════════════════════════════════
//  STATE MACHINE
// ═══════════════════════════════════════════════════════════════

void updateStateMachine(float dt) {

  // Safety stop
  if (state != IDLE && state != DONE) {
    if (elapsedS() > TARGET_TIME_S + 10.0f || distance > TARGET_DISTANCE_M + 2.0f) {
      Serial.println("!!! SAFETY STOP !!!");
      stopMotors();
      going = false;
      ledSetMode(LED_ERROR);
      enterState(DONE);
      runLogReady = true;
      return;
    }
  }

  switch (state) {

    case IDLE:
      {
        stopMotors();
        if (going) enterState(LANE_CHANGE_OUT);
        break;
      }

    case LANE_CHANGE_OUT:
      {
        float ramp_prog = clampf(x_track / RAMP_DIST_M, 0.0f, 1.0f);
        float spd = RAMP_UP_MIN_SPEED + (CRUISE_SPEED - RAMP_UP_MIN_SPEED) * ramp_prog;
        driveSCurve(spd, dt);
        if (x_track >= xThresh[0]) enterState(MID_STRAIGHT);
        break;
      }

    case MID_STRAIGHT:
      {
        driveSCurve(CRUISE_SPEED, dt);
        if (x_track >= xThresh[1]) enterState(LANE_CHANGE_BACK);
        break;
      }

    case LANE_CHANGE_BACK:
      {
        float remaining = TARGET_DISTANCE_M - x_track;

        // Stop condition
        if (remaining <= STOP_OFFSET_M) {
          stopMotors();
          going = false;
          actual_runtime_s = elapsedS();

          // Track coasting
          float coast_start_x = x_track;
          float coast_start_y = y_track;
          uint32_t coast_start_us = micros();
          uint32_t coast_last_us = coast_start_us;

          while ((micros() - coast_start_us) / 1e6f < 3.0f) {
            uint32_t now = micros();
            if (now - coast_last_us >= LOOP_US) {
              float cdt = (now - coast_last_us) / 1e6f;
              coast_last_us = now;
              pollIMU();
              updateIMUandEncoders(cdt);
              if (velocity < 0.01f) break;
            }
          }

          coast_dx = x_track - coast_start_x;
          coast_dy = y_track - coast_start_y;
          coast_final_x = x_track;
          coast_final_y = y_track;

          ledSetMode(LED_DONE);  //set LED to done
          enterState(DONE);
          runLogReady = true;

          // Blink to confirm done
          ledFlash();
          break;
        }

        // Speed: cruise, ramp down near end
        float effective_remaining = remaining - STOP_OFFSET_M;
        float v_cmd;
        if (effective_remaining < RAMP_DIST_M) {
          float ramp_prog = clampf(effective_remaining / RAMP_DIST_M, 0.0f, 1.0f);
          v_cmd = RAMP_DOWN_MIN_SPEED + (CRUISE_SPEED - RAMP_DOWN_MIN_SPEED) * ramp_prog;
        } else {
          v_cmd = CRUISE_SPEED;
        }

        // During ramp-down: allow zero motor power if overspeeding
        if (effective_remaining < RAMP_DIST_M && velocity > v_cmd + 0.05f) {
          float hdgTarget = scurveHeadingDeg(x_track);
          float y_err = scurveY(x_track) - y_track;  //adding cross track to ramp down
          hdgTarget += KP_CROSS * y_err;
          target_heading_deg = hdgTarget;
          float kappa = scurveCurvature(x_track);
          float ff_turn = KFF_CURV * rad2deg(velocity * kappa);

          float base = PWR_BIAS + PWR_SLOPE * v_cmd;
          base = clampf(base, 0.0f, PWR_MAX);

          float errDeg = wrapDeg(hdgTarget - headingDeg);
          heading_integral += errDeg * dt;
          heading_integral = clampf(heading_integral, -HEADING_I_MAX, HEADING_I_MAX);
          float turn = KP_HEADING * errDeg + KI_HEADING * heading_integral
                       - KD_HEADING * yawRateDeg + ff_turn;
          turn = clampf(turn, -0.30f, 0.30f);
          last_turn = turn;

          float left = clampf(base - turn, 0.0f, PWR_MAX);
          float right = clampf(base + turn, 0.0f, PWR_MAX);
          motorRight(right * MOTOR_R_DIR * MOTOR_BIAS_R);
          motorLeft(left * MOTOR_L_DIR * MOTOR_BIAS_L);
        } else {
          driveSCurve(v_cmd, dt);
        }
        break;
      }

    case DONE:
      stopMotors();
      break;
  }
}

// ═══════════════════════════════════════════════════════════════
//  BUTTON HANDLING
// ═══════════════════════════════════════════════════════════════

void handleButtons() {
  // GO (press-and-release)
  if (goButtonPressed()) {
    goPressed = true;
  } else if (goPressed && !goButtonPressed()) {
    goPressed = false;
    delay(50);  // ← add this line

    if (going) return;

    resetRun();
    going = true;


    if (!planRoute()) {
      going = false;
      ledSetMode(LED_ERROR);  //something wrong with route
      return;
    }
    computeSpeeds();

    //digitalWrite(LED_BUILTIN, HIGH);
    ledSetMode(LED_RUNNING);

    enterState(LANE_CHANGE_OUT);
    Serial.println(">>> RUN STARTED <<<");
  }

  // STOP (press-and-release)
  if (stopButtonPressed()) {
    stopPressed = true;
  } else if (stopPressed && !stopButtonPressed()) {
    stopPressed = false;
    delay(50);  // ← add this line

    if (going) {
      going = false;
      stopMotors();
      enterState(IDLE);
      runLogReady = true;
      Serial.println(">>> MANUAL STOP <<<");
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}

// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  ledSetMode(LED_INIT);  // ← blinking while BNO connects
  ledUpdate();

  // Replace delay(2000) with a loop that blinks LED
  for (int i = 0; i < 100; i++) {
    ledUpdate();
    delay(10);
  }
  //digitalWrite(LED_BUILTIN, LOW);

  // Motors
  pinMode(AO1, OUTPUT);
  pinMode(AO2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BO1, OUTPUT);
  pinMode(BO2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  analogWriteFreq(20000);
  analogWriteRange(255);
  stopMotors();

  // Encoders
  pinMode(L_ENC_A, INPUT_PULLUP);
  pinMode(L_ENC_B, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP);
  pinMode(R_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightISR, RISING);

  // Buttons
  pinMode(BTN_GO, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);

  // IMU with retry
  bool imuOk = false;
  for (int attempt = 0; attempt < 5; attempt++) {
    ledUpdate();  // ← blink during retries
    Serial.printf("BNO085 init attempt %d...\n", attempt + 1);
    if (initIMU()) {
      imuOk = true;
      break;
    }
    //delay(1000);
    for (int i = 0; i < 100; i++) {
      ledUpdate();
      delay(10);
    }
  }
  if (!imuOk) {
    ledSetMode(LED_ERROR);
    Serial.println("BNO085 FAILED after 5 attempts!");
    while (1) {
      //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      ledUpdate();
      delay(10);
    }
  }
  Serial.println("BNO085 OK");

  // Let IMU settle
  for (int i = 0; i < 400; i++) {
    pollIMU();
    ledUpdate();
    delay(10);
  }

  ledSetMode(LED_READY);  // ← heartbeat, safe to press GO

  Serial.println();
  Serial.println("╔══════════════════════════════════════════════╗");
  Serial.println("║  S-CURVE ROUTE — READY                       ║");
  Serial.printf("║  Distance: %.1f m   Time: %.1f s              ║\n",
                TARGET_DISTANCE_M, TARGET_TIME_S);
  Serial.println("║  Press GO (GP15) to start                   ║");
  Serial.println("║  Press STOP (GP14) to abort                 ║");
  Serial.println("╚══════════════════════════════════════════════╝");
  Serial.println();

  last_loop_us = micros();
}

// ═══════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════════

void loop() {
  pollIMU();
  ledUpdate();
  handleButtons();
  serviceDeferredLogPrint();

  if (!going) return;

  uint32_t now = micros();
  if (now - last_loop_us < LOOP_US) return;

  float dt = (now - last_loop_us) / 1e6f;
  last_loop_us = now;

  updateIMUandEncoders(dt);
  updateStateMachine(dt);

  // Telemetry
  if (millis() - last_print_ms > 500) {
    last_print_ms = millis();
    Serial.printf("%s | d:%.2f x:%.2f y:%.2f yT:%.2f | H:%.1f T:%.1f | v:%.2f k:%.3f\n",
                  state_names[state], distance, x_track, y_track, scurveY(x_track),
                  headingDeg, target_heading_deg, velocity, scurveCurvature(x_track));
  }
  // Telemetry logging (every 50ms)
  if (tlogCount < MAX_TLOG && (millis() - last_tlog_ms >= 100)) {
    last_tlog_ms = millis();
    TelemetryEntry &e = tlog[tlogCount++];
    e.t_s = elapsedS();
    e.x = x_track;
    e.y = y_track;
    e.y_target = scurveY(x_track);
    e.y_err = e.y_target - y_track;
    e.heading = headingDeg;
    e.heading_target = target_heading_deg;
    e.heading_err = wrapDeg(target_heading_deg - headingDeg);
    e.velocity = velocity;
    //e.turn_cmd       = KP_HEADING * e.heading_err;
    e.turn_cmd = last_turn;
  }
}
