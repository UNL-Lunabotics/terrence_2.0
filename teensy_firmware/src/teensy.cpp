// teensy_firmware/src/teensy.cpp
// Minimal 3-pin version:
//  - Pin 6: left motor PWM
//  - Pin 7: right motor PWM
//  - Pin 8: scoop motor PWM
// No direction pins, no encoders.

#include <Arduino.h>
#include <Servo.h>

// =================== USER CONFIG ===================================

// Servo stuff??
Servo leftRoboclaw;
Servo rightRoboclaw;

// Servo timings
constexpr int TIMING_FULL_REVERSE   = 1000;
constexpr int TIMING_STOP           = 1500;
constexpr int TIMING_FULL_FORWARD   = 2000;
constexpr int TIMING_DELTA   = TIMING_FULL_FORWARD - TIMING_FULL_REVERSE;

// Motor pins
constexpr int LEFT_PWM_PIN   = 9;
constexpr int RIGHT_PWM_PIN  = 6;
constexpr int SCOOP_PWM_PIN  = 7;

// Max command magnitude expected from ROS (e.g. rad/s or arbitrary units)
constexpr float MAX_WHEEL_CMD = 10.0f;
constexpr float MAX_SCOOP_CMD = 10.0f;

// If you literally have no way to reverse (one PWM pin, driver only cares about duty),
// set this true and we’ll clamp negative commands to 0.
constexpr bool ONE_DIRECTION_ONLY = false;

// ===================================================================

// Last commanded values (just for debugging)
float cmd_left  = 0.0f;
float cmd_right = 0.0f;
float cmd_scoop = 0.0f;

float pwm_last  = 0.0;

// Forward declarations
void handleSerialLine(String & line);
void setMotorPwmOnly(int pwm_pin, float cmd, float max_cmd);

String line_buffer;

void setup()
{
  Serial.begin(57600);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  //pinMode(LEFT_PWM_PIN, OUTPUT);
  //pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(SCOOP_PWM_PIN, OUTPUT);

  rightRoboclaw.attach(RIGHT_PWM_PIN);
  leftRoboclaw.attach(LEFT_PWM_PIN);

  rightRoboclaw.writeMicroseconds(TIMING_STOP);
  leftRoboclaw.writeMicroseconds(TIMING_STOP);
}

void loop()
{
  // Read serial input line by line
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    Serial.print(c);
    if (c == '\n' || c == '\r') {
      if (line_buffer.length() > 0) {
        handleSerialLine(line_buffer);
        line_buffer = "";
      }
    } else {
      line_buffer += c;
    }
  }

  delay(1);  // tiny breather
}

void handleSerialLine(String & line)
{
  if (line.length() == 0) return;

  char cmd = line.charAt(0);

  if (cmd == 'm') {
    // "m <left> <right> <scoop>"
    float left = 0.0f, right = 0.0f, scoop = 0.0f;
    int parsed = sscanf(line.c_str(), "m %f %f %f", &left, &right, &scoop);

    if (parsed >= 2) {
      cmd_left  = left;
      cmd_right = right;

      if (parsed == 3) {
        cmd_scoop = scoop;
      }

      setMotorPwmOnly(0, cmd_left,  MAX_WHEEL_CMD);
      setMotorPwmOnly(1, cmd_right, MAX_WHEEL_CMD);
      // setMotorPwmOnly(SCOOP_PWM_PIN, cmd_scoop, MAX_SCOOP_CMD);
    }
  }
  else if (cmd == 'e') {
    // Encoder request: we have no encoders, so just return zeros.
    // This keeps the ROS hardware plugin from choking on missing data,
    // but gives you no real odometry.
    long enc_left = 0;
    long enc_right = 0;
    long enc_scoop = 0;

    Serial.print("e ");
    Serial.print(enc_left);
    Serial.print(" ");
    Serial.print(enc_right);
    Serial.print(" ");
    Serial.println(enc_scoop);
  }
  else if (cmd == 'z') {
    // Zero encoders – no-op here, but keep for protocol compatibility.
    // (nothing to do)
  }
  // Add more commands if needed
}

void setMotorPwmOnly(int motorIndex, float cmd, float max_cmd)
{
  // If the hardware only supports one direction, clamp negatives to 0
  if (ONE_DIRECTION_ONLY && cmd < 0.0f) {
    cmd = 0.0f;
  }

  // Saturate
  if (cmd > max_cmd) cmd = max_cmd;
  if (cmd < -max_cmd) cmd = -max_cmd;

  // Convert |cmd| to 0–255 duty cycle
  float mag = fabsf(cmd) / max_cmd;
  if (mag > 1.0f) mag = 1.0f;
  float pwm = ((float) mag * (float) TIMING_DELTA) + TIMING_FULL_REVERSE;

  Serial.println(cmd);

  switch (motorIndex) {
    case 0:
      leftRoboclaw.writeMicroseconds(pwm);
      break;
    case 1:
      rightRoboclaw.writeMicroseconds(pwm);
      break;
    default:
      Serial.println("Wrong!!!");
  }
}