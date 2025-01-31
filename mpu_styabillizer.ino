#include <Wire.h>
#include <math.h>
#include <HardwareSerial.h>
#include <mrm-robot.h>
//#include <mrm-board.h>
#include <mrm-can-bus.h>
#include <Arduino.h>

// MPU6050 Configuration
const int MPU_ADDR = 0x68;
float accel_scale = 16384.0; // For ±2g range
float gyro_scale = 131.0;    // For ±250°/s range

// IMU Variables
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float roll, pitch;
float compAngleRoll, compAnglePitch;
unsigned long last_time = 0;
float dt;

// Receiver Pins
const int THROTTLE_PIN = 2;
const int AILERON_PIN = 3;
const int ELEVATOR_PIN = 4;
const int RUDDER_PIN = 5;

// PWM Input Variables
volatile unsigned long throttle_in, aileron_in, elevator_in, rudder_in;
volatile unsigned long pwm_throttle, pwm_aileron, pwm_elevator, pwm_rudder;

// Servo Output Pins
const int THROTTLE_OUT = 12;
const int AILERON_OUT = 13;
const int ELEVATOR_OUT = 14;
const int RUDDER_OUT = 15;

// PID Constants (You'll need to tune these)
float Kp = 0.5;
float Ki = 0.01;
float Kd = 0.1;

// PID Variables
float error, lastError, integral, derivative;

// PWM Read ISRs
void pwmRead(int pin, volatile unsigned long &prev_time, volatile unsigned long &pwm_value);
void writeServo(int channel, float value);
void readMPU6050();
void readThrottle();
void readAileron();
void readElevator();
void readRudder();

void calculateAngles();
void applyComplementaryFilter();
float stabilize(float command, float currentAngle, int axis);

void setup()
{
  Serial.begin(115200);

  // Initialize PWM input pins
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(AILERON_PIN, INPUT);
  pinMode(ELEVATOR_PIN, INPUT);
  pinMode(RUDDER_PIN, INPUT);

  // Attach interrupts for PWM reading
  attachInterrupt(digitalPinToInterrupt(THROTTLE_PIN), readThrottle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(AILERON_PIN), readAileron, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ELEVATOR_PIN), readElevator, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RUDDER_PIN), readRudder, CHANGE);

  // Initialize MPU6050
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Wake up MPU6050
  Wire.endTransmission(true);

  // Configure servo outputs
  ledcSetup(0, 50, 16); // Channel 0, 50Hz, 16-bit resolution
  ledcAttachPin(THROTTLE_OUT, 0);
  ledcSetup(1, 50, 16);
  ledcAttachPin(AILERON_OUT, 1);
  ledcSetup(2, 50, 16);
  ledcAttachPin(ELEVATOR_OUT, 2);
  ledcSetup(3, 50, 16);
  ledcAttachPin(RUDDER_OUT, 3);
}

void loop()
{
  // Calculate delta time
  unsigned long current_time = micros();
  dt = (current_time - last_time) / 1000000.0;
  last_time = current_time;

  // Read IMU data
  readMPU6050();
  calculateAngles();
  applyComplementaryFilter();

  // Read PWM values (atomic operation to prevent interrupt conflicts)
  noInterrupts();
  float throttle = pwm_throttle;
  float aileron = pwm_aileron;
  float elevator = pwm_elevator;
  float rudder = pwm_rudder;
  interrupts();

  // Convert PWM to -1 to 1 range
  float aileron_cmd = map(aileron, 1000, 2000, -1.0, 1.0);
  float elevator_cmd = map(elevator, 1000, 2000, -1.0, 1.0);
  float rudder_cmd = map(rudder, 1000, 2000, -1.0, 1.0);

  // Stabilization calculations
  float aileron_corrected = stabilize(aileron_cmd, compAngleRoll, 0);    // Roll axis
  float elevator_corrected = stabilize(elevator_cmd, compAnglePitch, 1); // Pitch axis

  // Mix commands with stabilization
  float aileron_output = aileron_cmd + aileron_corrected;
  float elevator_output = elevator_cmd + elevator_corrected;
  float rudder_output = rudder_cmd;

  // Constrain outputs
  aileron_output = constrain(aileron_output, -1.0, 1.0);
  elevator_output = constrain(elevator_output, -1.0, 1.0);
  rudder_output = constrain(rudder_output, -1.0, 1.0);

  // Write outputs to servos
  writeServo(THROTTLE_OUT, throttle);
  writeServo(AILERON_OUT, aileron_output);
  writeServo(ELEVATOR_OUT, elevator_output);
  writeServo(RUDDER_OUT, rudder_output);

  delay(10); // Main loop rate ~100Hz
}
// PWM Read ISRs
void readThrottle() { pwmRead(THROTTLE_PIN, throttle_in, pwm_throttle); }
void readAileron() { pwmRead(AILERON_PIN, aileron_in, pwm_aileron); }
void readElevator() { pwmRead(ELEVATOR_PIN, elevator_in, pwm_elevator); }
void readRudder() { pwmRead(RUDDER_PIN, rudder_in, pwm_rudder); }

void pwmRead(int pin, volatile unsigned long &prev_time, volatile unsigned long &pwm_value)
{
  if (digitalRead(pin))
  {
    prev_time = micros();
  }
  else
  {
    pwm_value = micros() - prev_time;
  }
}

// IMU Functions
void readMPU6050()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);

  accX = Wire.read() << 8 | Wire.read();
  accY = Wire.read() << 8 | Wire.read();
  accZ = Wire.read() << 8 | Wire.read();
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();

  // Convert to proper units
  accX /= accel_scale;
  accY /= accel_scale;
  accZ /= accel_scale;
  gyroX /= gyro_scale;
  gyroY /= gyro_scale;
  gyroZ /= gyro_scale;
}

void calculateAngles()
{
  roll = atan2(accY, accZ) * 180 / PI;
  pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / PI;
}

void applyComplementaryFilter()
{
  compAngleRoll = 0.98 * (compAngleRoll + gyroX * dt) + 0.02 * roll;
  compAnglePitch = 0.98 * (compAnglePitch + gyroY * dt) + 0.02 * pitch;
}

// Stabilization Function
float stabilize(float command, float currentAngle, int axis)
{
  static float lastError[2] = {0};
  static float integral[2] = {0};

  float setpoint = command * 30.0; // Scale command to angle (adjust as needed)
  error = setpoint - currentAngle;

  integral[axis] += error * dt;
  derivative = (error - lastError[axis]) / dt;

  float output = Kp * error + Ki * integral[axis] + Kd * derivative;
  lastError[axis] = error;

  return constrain(output, -15.0, 15.0) / 30.0; // Scale output to match command range
}

// Servo Output Function
void writeServo(int channel, float value)
{
  if (channel == THROTTLE_OUT)
  {
    // Directly map throttle (0-100%)
    uint16_t pulse = map(value, 1000, 2000, 3273, 6553); // 50Hz PWM
    ledcWrite(0, pulse);
  }
  else
  {
    // Convert -1 to 1 to PWM
    uint16_t pulse = map(value * 1000, -1000, 1000, 3273, 6553);
    switch (channel)
    {
    case AILERON_OUT:
      ledcWrite(1, pulse);
      break;
    case ELEVATOR_OUT:
      ledcWrite(2, pulse);
      break;
    case RUDDER_OUT:
      ledcWrite(3, pulse);
      break;
    }
  }
}
