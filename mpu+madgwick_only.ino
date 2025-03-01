#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "MadgwickAHRS.h"

// Create sensor and filter objects
Adafruit_MPU6050 mpu;
Madgwick filter;

// Constants
#define SAMPLE_RATE 100  // Hz (100 updates per second)
#define LOOP_DELAY (1000 / SAMPLE_RATE)
#define CALIBRATION_TIME 3000  // 3 seconds

// Offset variables
float rollOffset = 0, pitchOffset = 0, yawOffset = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Initialize MPU6050
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050! Check connections.");
        while (1);
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Start Madgwick filter
    filter.begin(SAMPLE_RATE);

    Serial.println("Starting calibration...");
    calibrateMPU6050();
    Serial.println("Calibration complete!");
}

void loop() {
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate >= LOOP_DELAY) {
        lastUpdate = millis();  // Update last run time

        // Read MPU6050 sensor data
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        // Convert gyro data to radians per second
        float gx = g.gyro.x * DEG_TO_RAD;
        float gy = g.gyro.y * DEG_TO_RAD;
        float gz = g.gyro.z * DEG_TO_RAD;

        // Convert accelerometer data to g's
        float ax = a.acceleration.x;
        float ay = a.acceleration.y;
        float az = a.acceleration.z;

        // Apply Madgwick filter
        filter.updateIMU(gx, gy, gz, ax, ay, az);

        // Get orientation and apply offset correction
        float roll = filter.getRoll() - rollOffset;
        float pitch = filter.getPitch() - pitchOffset;
        float yaw = filter.getYaw() - yawOffset;

        // Print corrected orientation data
        Serial.print("Roll: "); Serial.print(roll);
        Serial.print(" | Pitch: "); Serial.print(pitch);
        Serial.print(" | Yaw: "); Serial.println(yaw);
    }
}

// Function to calibrate MPU6050
void calibrateMPU6050() {
    float rollSum = 0, pitchSum = 0, yawSum = 0;
    int sampleCount = 0;
    unsigned long startTime = millis();

    while (millis() - startTime < CALIBRATION_TIME) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        float gx = g.gyro.x * DEG_TO_RAD;
        float gy = g.gyro.y * DEG_TO_RAD;
        float gz = g.gyro.z * DEG_TO_RAD;
        float ax = a.acceleration.x;
        float ay = a.acceleration.y;
        float az = a.acceleration.z;

        filter.updateIMU(gx, gy, gz, ax, ay, az);

        // Accumulate orientation values
        rollSum += filter.getRoll();
        pitchSum += filter.getPitch();
        yawSum += filter.getYaw();
        sampleCount++;

        delay(10);  // Small delay to match sampling rate
    }

    // Calculate average offsets
    rollOffset = rollSum / sampleCount;
    pitchOffset = pitchSum / sampleCount;
    yawOffset = yawSum / sampleCount;

    Serial.println("Calibration offsets set:");
    Serial.print("Roll Offset: "); Serial.println(rollOffset);
    Serial.print("Pitch Offset: "); Serial.println(pitchOffset);
    Serial.print("Yaw Offset: "); Serial.println(yawOffset);
}
