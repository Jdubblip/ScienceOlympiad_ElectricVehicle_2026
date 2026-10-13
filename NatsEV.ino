#include <math.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

enum State {
  IDLE,
  LANE_CHANGE_OUT,
  MID_STRAIGHT,
  LANE_CHANGE_BACK,
  DONE
};

enum LedMode {
  LED_INIT,
  LED_READY,
  LED_RUNNING,
  LED_DONE,
  LED_ERROR
};

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

float TARGET_DISTANCE_M = 9.68f;
float TARGET_TIME_S = 10.50f;
float STOP_OFFSET_M = 0.0f;
float CAN_GAP = 0.10f;
float MID_STRAIGHT_M = 0.50f;

const float WHEEL_DIAM_M = 0.041275f * (6.964f / 7.0f) * (9.98f / 10.0f);
const float WHEEL_RADIUS_M = WHEEL_DIAM_M * 0.5f;
const float GEAR_RATIO = 1.0f;
const float TRACK_WIDTH_M = 0.049f;

const float TICKS_PER_REV = 154.35f;
const float DIST_PER_TICK = (PI * WHEEL_DIAM_M) / TICKS_PER_REV;

const float KP_HEADING = 0.027f;
const float KD_HEADING = 0.0013f;
const float KI_HEADING = 0.005f;
const float HEADING_I_MAX = 0.05f;
float heading_integral = 0.0f;
const float KFF_CURV = 0.003f;
const float KP_CROSS = 20.0f;

const float KP_SPEED = 0.50f;
const float KI_SPEED = 0.80f;
const float SPEED_I_MAX = 0.15f;
float speed_integral = 0.0f;

const float RAMP_DIST_MIN = 0.40f;
const float RAMP_UP_MIN_SPEED = 0.20f;
const float RAMP_DOWN_MIN_SPEED = 0.12f;
const float RAMP_FRAC = 0.25f;
float RAMP_DIST_M;
float CRUISE_SPEED = 0.65f;
const float V_MAX_CRUISE = 1.40f;
const float TRANSITION_OVERHEAD_S = -0.5f;

const int ENC_L_SIGN = 1;
const int ENC_R_SIGN = 1;
const int MOTOR_L_DIR = 1;
const int MOTOR_R_DIR = 1;

const float MOTOR_BIAS_L = 1.00f;
float MOTOR_BIAS_R = 1.00f;

float computeMotorBiasR(float cruise_speed) {
  float bias = 1.04f - 0.03f * cruise_speed;
  return constrain(bias, 1.00f, 1.05f);
}

const float PWR_BIAS = 0.111f;
const float PWR_SLOPE = 0.592f;
const float PWR_MIN = 0.15f;
const float PWR_MAX = 0.90f;

const uint32_t LOOP_US = 2000;

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
      if (elapsed >= 80) {
        led_timer_ms = millis();
        led_on = !led_on;
      }
      break;

    case LED_READY:
      if (!led_on && elapsed >= 2000) {
        led_on = true;
        led_timer_ms = millis();
      } else if (led_on && elapsed >= 300) {
        led_on = false;
        led_timer_ms = millis();
      }
      break;

    case LED_RUNNING:
      led_on = true;
      break;

    case LED_DONE:
      if (!led_on && elapsed >= 2000) {
        led_on = true;
        led_timer_ms = millis();
      } else if (led_on && elapsed >= 300) {
        led_on = false;
        led_timer_ms = millis();
      }
      break;

    case LED_ERROR:
      if (elapsed >= 80) {
        led_timer_ms = millis();
        led_on = !led_on;
      }
      break;
  }

  digitalWrite(LED_BUILTIN, led_on);
}

Adafruit_BNO08x bno(-1);
sh2_SensorValue_t _evt;
float yawDeg = 0.0f;
float yawRateDPS = 0.0f;

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

volatile long leftTicks = 0;
volatile long rightTicks = 0;

void leftISR() {
  digitalRead(L_ENC_B) ? leftTicks++ : leftTicks--;
}
void rightISR() {
  digitalRead(R_ENC_B) ? rightTicks++ : rightTicks--;
}

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
  }
  else if (pwr < -0.001f) {
    digitalWrite(BO1, HIGH);
    digitalWrite(BO2, LOW);
  }
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

bool goButtonPressed() {
  return !digitalRead(BTN_GO);
}
bool stopButtonPressed() {
  return !digitalRead(BTN_STOP);
}

void ledFlash() {
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_BUILTIN, i % 2);
    delay(100);
  }
  digitalWrite(LED_BUILTIN, LOW);
}

State state = IDLE;

const char *state_names[] = {
  "IDLE", "LC_OUT", "MID", "LC_BACK", "DONE"
};

float OFFSET_M = 1.0f;
float Lc_m = 0.0f;
float pathLen_m = 0.0f;
float peakCurvature = 0.0f;
float xThresh[4];

bool goPressed = false;
bool stopPressed = false;
bool going = false;

uint32_t run_start_us = 0;
uint32_t last_loop_us = 0;
uint32_t last_print_ms = 0;

float state_start_distance = 0.0f;
float target_heading_deg = 0.0f;
float seg_start_dist = 0.0f;

float headingDeg = 0.0f;
float yawRateDeg = 0.0f;
float startYaw = 0.0f;
float prevHeadingDeg = 0.0f;

float dist_L = 0.0f, dist_R = 0.0f;
float distance = 0.0f;
float velocity = 0.0f;
float x_track = 0.0f, y_track = 0.0f;

long prevTickL = 0, prevTickR = 0;
bool enc_initialized = false;

float coast_dx = 0.0f, coast_dy = 0.0f;
float coast_final_x = 0.0f, coast_final_y = 0.0f;
float actual_runtime_s = 0.0f;

float last_turn = 0.0f;

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
  Serial.println("========================================================");
  Serial.println("  S-CURVE RUN LOG");
  Serial.println("========================================================");
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

  Serial.println("--------------------------------------------------------");
  Serial.printf("  Coast dx: %.4f m   dy: %.4f m\n", coast_dx, coast_dy);
  Serial.printf("  Final x (w/coast): %.3f m\n", coast_final_x);
  Serial.printf("  Final y (w/coast): %.3f m\n", coast_final_y);
  Serial.printf("  Total runtime: %.3f s\n", actual_runtime_s);
  Serial.println("========================================================");
  Serial.println();

  Serial.println("---- TELEMETRY LOG ----");
  Serial.println("  t,      x,      y,     yT,     yE,    hdg,   hdgT,   hdgE,   vel,   turn");
  for (int i = 0; i < tlogCount; i++) {
    const TelemetryEntry &e = tlog[i];
    Serial.printf("%6.3f, %6.3f, %7.3f, %6.3f, %7.4f, %7.2f, %7.2f, %7.2f, %6.3f, %7.4f\n",
                  e.t_s, e.x, e.y, e.y_target, e.y_err,
                  e.heading, e.heading_target, e.heading_err,
                  e.velocity, e.turn_cmd);
  }
}
void serviceDeferredLogPrint() {
  if (runLogReady && !runLogPrinted && Serial) {
    delay(200);
    printStateLogs();
    runLogPrinted = true;
  }
}

void updateIMUandEncoders(float dt) {
  float newHeading = wrapDeg(yawDeg - startYaw);
  yawRateDeg = yawRateDPS;

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

  float dHeading = wrapDeg(newHeading - prevHeadingDeg);
  float midHeading = prevHeadingDeg + 0.5f * dHeading;
  float hMidRad = deg2rad(midHeading);
  x_track += ds * cosf(hMidRad);
  y_track += ds * sinf(hMidRad);

  prevHeadingDeg = newHeading;
  headingDeg = newHeading;
}

void driveSpeed(float speed_mps, float heading_target_deg, float dt) {
  float base_ff = PWR_BIAS + PWR_SLOPE * speed_mps;

  float speed_err = speed_mps - velocity;
  speed_integral += speed_err * dt;
  speed_integral = clampf(speed_integral, -SPEED_I_MAX, SPEED_I_MAX);
  float base = base_ff + KP_SPEED * speed_err + KI_SPEED * speed_integral;
  base = clampf(base, PWR_MIN, PWR_MAX);

  float errDeg = wrapDeg(heading_target_deg - headingDeg);
  heading_integral += errDeg * dt;
  heading_integral = clampf(heading_integral, -HEADING_I_MAX, HEADING_I_MAX);
  float turn = KP_HEADING * errDeg + KI_HEADING * heading_integral - KD_HEADING * yawRateDeg;
  turn = clampf(turn, -0.30f, 0.30f);

  float left = clampf(base - turn, -PWR_MAX, PWR_MAX);
  float right = clampf(base + turn, -PWR_MAX, PWR_MAX);

  motorRight(right * MOTOR_R_DIR * MOTOR_BIAS_R);
  motorLeft(left * MOTOR_L_DIR * MOTOR_BIAS_L);
}

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

void driveSCurve(float speed_mps, float dt) {
  float hdgTarget = scurveHeadingDeg(x_track);
  float y_err = scurveY(x_track) - y_track;
  hdgTarget += KP_CROSS * y_err;

  target_heading_deg = hdgTarget;

  float kappa = scurveCurvature(x_track);
  float expectedYawDegS = rad2deg(speed_mps * kappa);
  float ff_turn = KFF_CURV * expectedYawDegS;

  driveSpeedFF(speed_mps, hdgTarget, ff_turn, dt);
}

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

float rampTime(float ramp_dist, float v_min, float v_cruise) {
  if (v_cruise <= v_min + 0.01f) return ramp_dist / v_min;
  return ramp_dist / (v_cruise - v_min) * logf(v_cruise / v_min);
}

void computeSpeeds() {
  pathLen_m = arcLengthBetween(0.0f, TARGET_DISTANCE_M);

  RAMP_DIST_M = clampf(RAMP_FRAC * Lc_m, RAMP_DIST_MIN, 0.50f * Lc_m);
  float rampDn_xDist = min(RAMP_DIST_M, 0.50f * Lc_m);

  float arcRampUp = arcLengthBetween(0.0f, RAMP_DIST_M);
  float arcRampDn = arcLengthBetween(TARGET_DISTANCE_M - rampDn_xDist, TARGET_DISTANCE_M);
  float arcCruise = pathLen_m - arcRampUp - arcRampDn;

  CRUISE_SPEED = clampf(pathLen_m / TARGET_TIME_S, 0.25f, V_MAX_CRUISE);

  for (int pass = 0; pass < 5; pass++) {
    float rampUp_time = rampTime(arcRampUp, RAMP_UP_MIN_SPEED, CRUISE_SPEED);
    float rampDn_time = rampTime(arcRampDn, RAMP_DOWN_MIN_SPEED, CRUISE_SPEED);

    float cruise_time = TARGET_TIME_S - rampUp_time - rampDn_time - TRANSITION_OVERHEAD_S;
    if (cruise_time > 0.1f) {
      CRUISE_SPEED = clampf(arcCruise / cruise_time, 0.25f, V_MAX_CRUISE);
    }
  }

  MOTOR_BIAS_R = computeMotorBiasR(CRUISE_SPEED);

  Serial.printf("  Path length:  %.3f m\n", pathLen_m);
  Serial.printf("  Ramp dist (x): %.3f m\n", RAMP_DIST_M);
  Serial.printf("  Arc ramp up:  %.3f m   Arc ramp dn: %.3f m   Arc cruise: %.3f m\n",
                arcRampUp, arcRampDn, arcCruise);
  Serial.printf("  Cruise speed: %.3f m/s\n", CRUISE_SPEED);
  Serial.printf("  Motor bias R: %.4f\n", MOTOR_BIAS_R);

  float rampUp_t = rampTime(arcRampUp, RAMP_UP_MIN_SPEED, CRUISE_SPEED);
  float rampDn_t = rampTime(arcRampDn, RAMP_DOWN_MIN_SPEED, CRUISE_SPEED);
  Serial.printf("  Ramp-up time: %.3f s   Ramp-dn time: %.3f s\n", rampUp_t, rampDn_t);
  Serial.printf("  Cruise time:  %.3f s\n", TARGET_TIME_S - rampUp_t - rampDn_t - TRANSITION_OVERHEAD_S);
}

void enterState(State s) {
  logStateChange(state, s);
  state = s;
  state_start_distance = distance;
  seg_start_dist = distance;
}

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

  pollIMU();
  startYaw = yawDeg;
  headingDeg = 0.0f;
  prevHeadingDeg = 0.0f;

  tlogCount = 0;
  last_tlog_ms = 0;
}

void updateStateMachine(float dt) {

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

        if (remaining <= STOP_OFFSET_M) {
          stopMotors();
          going = false;
          actual_runtime_s = elapsedS();

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

          ledSetMode(LED_DONE);
          enterState(DONE);
          runLogReady = true;

          ledFlash();
          break;
        }

        float effective_remaining = remaining - STOP_OFFSET_M;
        float v_cmd;
        if (effective_remaining < RAMP_DIST_M) {
          float ramp_prog = clampf(effective_remaining / RAMP_DIST_M, 0.0f, 1.0f);
          v_cmd = RAMP_DOWN_MIN_SPEED + (CRUISE_SPEED - RAMP_DOWN_MIN_SPEED) * ramp_prog;
        } else {
          v_cmd = CRUISE_SPEED;
        }

        if (effective_remaining < RAMP_DIST_M && velocity > v_cmd + 0.05f) {
          float hdgTarget = scurveHeadingDeg(x_track);
          float y_err = scurveY(x_track) - y_track;
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

void handleButtons() {
  if (goButtonPressed()) {
    goPressed = true;
  } else if (goPressed && !goButtonPressed()) {
    goPressed = false;
    delay(50);

    if (going) return;

    resetRun();
    going = true;

    if (!planRoute()) {
      going = false;
      ledSetMode(LED_ERROR);
      return;
    }
    computeSpeeds();

    ledSetMode(LED_RUNNING);

    enterState(LANE_CHANGE_OUT);
    Serial.println(">>> RUN STARTED <<<");
  }

  if (stopButtonPressed()) {
    stopPressed = true;
  } else if (stopPressed && !stopButtonPressed()) {
    stopPressed = false;
    delay(50);

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

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  ledSetMode(LED_INIT);
  ledUpdate();

  for (int i = 0; i < 100; i++) {
    ledUpdate();
    delay(10);
  }

  pinMode(AO1, OUTPUT);
  pinMode(AO2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BO1, OUTPUT);
  pinMode(BO2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  analogWriteFreq(20000);
  analogWriteRange(255);
  stopMotors();

  pinMode(L_ENC_A, INPUT_PULLUP);
  pinMode(L_ENC_B, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP);
  pinMode(R_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightISR, RISING);

  pinMode(BTN_GO, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);

  bool imuOk = false;
  for (int attempt = 0; attempt < 5; attempt++) {
    ledUpdate();
    Serial.printf("BNO085 init attempt %d...\n", attempt + 1);
    if (initIMU()) {
      imuOk = true;
      break;
    }
    for (int i = 0; i < 100; i++) {
      ledUpdate();
      delay(10);
    }
  }
  if (!imuOk) {
    ledSetMode(LED_ERROR);
    Serial.println("BNO085 FAILED after 5 attempts!");
    while (1) {
      ledUpdate();
      delay(10);
    }
  }
  Serial.println("BNO085 OK");

  for (int i = 0; i < 400; i++) {
    pollIMU();
    ledUpdate();
    delay(10);
  }

  ledSetMode(LED_READY);

  Serial.println();
  Serial.println("==============================");
  Serial.println("S-CURVE ROUTE - READY");
  Serial.printf("Distance: %.1f m   Time: %.1f s\n",
                TARGET_DISTANCE_M, TARGET_TIME_S);
  Serial.println("Press GO (GP15) to start");
  Serial.println("Press STOP (GP14) to abort");
  Serial.println("==============================");
  Serial.println();

  last_loop_us = micros();
}

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

  if (millis() - last_print_ms > 500) {
    last_print_ms = millis();
    Serial.printf("%s | d:%.2f x:%.2f y:%.2f yT:%.2f | H:%.1f T:%.1f | v:%.2f k:%.3f\n",
                  state_names[state], distance, x_track, y_track, scurveY(x_track),
                  headingDeg, target_heading_deg, velocity, scurveCurvature(x_track));
  }
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
    e.turn_cmd = last_turn;
  }
}
