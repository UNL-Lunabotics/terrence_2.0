// teensy_firmware/src/teensy.cpp

#include <Arduino.h>

// ======= USER CONFIG: pins, encoder CPR, etc. ==================

// Motor driver pins: fill these in for your hardware
// Left drive motor
constexpr int LEFT_PWM_PIN   = 2;
constexpr int LEFT_DIR_PIN   = 3;
constexpr int LEFT_ENC_A_PIN = 4;
constexpr int LEFT_ENC_B_PIN = 5;

// Right drive motor
constexpr int RIGHT_PWM_PIN   = 6;
constexpr int RIGHT_DIR_PIN   = 7;
constexpr int RIGHT_ENC_A_PIN = 8;
constexpr int RIGHT_ENC_B_PIN = 9;

// Scoop motor
constexpr int SCOOP_PWM_PIN   = 10;
constexpr int SCOOP_DIR_PIN   = 11;
constexpr int SCOOP_ENC_A_PIN = 12;
constexpr int SCOOP_ENC_B_PIN = 13;

// Encoder counts per rev (must match enc_counts_per_rev in ros2_control.xacro)
constexpr long ENCODER_CPR = 3450;

// Simple velocity-to-PWM scaling (tune this!)
constexpr float MAX_WHEEL_RAD_PER_SEC = 10.0f;
constexpr float MAX_SCOOP_CMD         = 10.0f;

// ===============================================================

volatile long enc_left_counts  = 0;
volatile long enc_right_counts = 0;
volatile long enc_scoop_counts = 0;

// Last commanded velocities / commands (for debugging)
float cmd_left  = 0.0f;
float cmd_right = 0.0f;
float cmd_scoop = 0.0f;

// Forward declarations
void handleSerialLine(const String & line);
void setMotor(int pwm_pin, int dir_pin, float cmd, float max_cmd);
void setupEncoders();

// ===== Encoder ISRs (you must adapt to your wiring) ============

// These are placeholders; you'll want proper quadrature decode.
// For quick-and-dirty: count A rising edges as +1/-1 depending on B.
void IRAM_ATTR leftEncAISR()
{
  int b = digitalRead(LEFT_ENC_B_PIN);
  enc_left_counts += (b ? -1 : 1);
}

void IRAM_ATTR rightEncAISR()
{
  int b = digitalRead(RIGHT_ENC_B_PIN);
  enc_right_counts += (b ? -1 : 1);
}

void IRAM_ATTR scoopEncAISR()
{
  int b = digitalRead(SCOOP_ENC_B_PIN);
  enc_scoop_counts += (b ? -1 : 1);
}

void setupEncoders()
{
  pinMode(LEFT_ENC_A_PIN, INPUT_PULLUP);
  pinMode(LEFT_ENC_B_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B_PIN, INPUT_PULLUP);
  pinMode(SCOOP_ENC_A_PIN, INPUT_PULLUP);
  pinMode(SCOOP_ENC_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A_PIN), leftEncAISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A_PIN), rightEncAISR, RISING);
  attachInterrupt(digitalPinToInterrupt(SCOOP_ENC_A_PIN), scoopEncAISR, RISING);
}

// ===============================================================

void setup()
{
  Serial.begin(57600);
  while (!Serial) {
    ; // wait for USB
  }

  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(SCOOP_PWM_PIN, OUTPUT);
  pinMode(SCOOP_DIR_PIN, OUTPUT);

  setupEncoders();

  // Coasts to a stop on boot
  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);
  analogWrite(SCOOP_PWM_PIN, 0);
}

// Simple line buffer
String line_buffer;

void loop()
{
  // Read serial input line-by-line
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (line_buffer.length() > 0) {
        handleSerialLine(line_buffer);
        line_buffer = "";
      }
    } else {
      line_buffer += c;
    }
  }

  // (Optionally add a small delay to reduce CPU usage)
  delay(1);
}

void handleSerialLine(const String & line)
{
  if (line.length() == 0) return;

  // First character is command
  char cmd = line.charAt(0);

  if (cmd == 'm') {
    // "m <left> <right> <scoop>"
    // parse floats from rest of line
    float left = 0.0f, right = 0.0f, scoop = 0.0f;

    // crude parsing using sscanf-style function; String::c_str() is fine
    int parsed = sscanf(line.c_str(), "m %f %f %f", &left, &right, &scoop);
    if (parsed >= 2) {
      cmd_left  = left;
      cmd_right = right;
      if (parsed == 3) {
        cmd_scoop = scoop;
      }

      setMotor(LEFT_PWM_PIN,  LEFT_DIR_PIN,  cmd_left,  MAX_WHEEL_RAD_PER_SEC);
      setMotor(RIGHT_PWM_PIN, RIGHT_DIR_PIN, cmd_right, MAX_WHEEL_RAD_PER_SEC);
      setMotor(SCOOP_PWM_PIN, SCOOP_DIR_PIN, cmd_scoop, MAX_SCOOP_CMD);
    }
  }
  else if (cmd == 'e') {
    // Request encoder values
    long l, r, s;
    noInterrupts();
    l = enc_left_counts;
    r = enc_right_counts;
    s = enc_scoop_counts;
    interrupts();

    Serial.print("e ");
    Serial.print(l);
    Serial.print(" ");
    Serial.print(r);
    Serial.print(" ");
    Serial.println(s);
  }
  else if (cmd == 'z') {
    // Optional: zero encoders
    noInterrupts();
    enc_left_counts  = 0;
    enc_right_counts = 0;
    enc_scoop_counts = 0;
    interrupts();
  }
  // You can add more commands here as needed.
}

void setMotor(int pwm_pin, int dir_pin, float cmd, float max_cmd)
{
  // Saturate
  if (cmd > max_cmd) cmd = max_cmd;
  if (cmd < -max_cmd) cmd = -max_cmd;

  // Direction
  bool dir = (cmd >= 0.0f);
  digitalWrite(dir_pin, dir ? HIGH : LOW);

  // Scale |cmd| -> [0, 255]
  float mag = fabsf(cmd) / max_cmd;
  if (mag > 1.0f) mag = 1.0f;
  int pwm = static_cast<int>(mag * 255.0f);

  analogWrite(pwm_pin, pwm);
}
