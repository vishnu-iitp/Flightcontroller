#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include "MPU6050.h"

MPU6050 mpu;
WebServer server(80);

const char* ssid = "YourSSID";
const char* password = "YourPassword";

String page = R"=====(
<!DOCTYPE html>
<html>
<head>
<style>
  #cube {
    width: 100px;
    height: 100px;
    background: linear-gradient(to right, #ff6700, #ffd900);
    transform-style: preserve-3d;
    margin: 100px auto;
    transition: transform 0.1s;
  }
</style>
</head>
<body>
<div id="cube"></div>
<script>
  async function getOrientation() {
    let response = await fetch('/orientation');
    let data = await response.json();
    let cube = document.getElementById("cube");
    // Update rotation using pitch, roll, yaw
    cube.style.transform = `rotateX(${data.pitch}deg) rotateY(${data.yaw}deg) rotateZ(${data.roll}deg)`;
  }
  setInterval(getOrientation, 200);
</script>
</body>
</html>
)=====";

void setup() {
  Wire.begin();
  Serial.begin(115200);
  mpu.initialize();
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  server.on("/", []() {
    server.send(200, "text/html", page);
  });
  server.on("/orientation", []() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float pitch = atan2(mpu.getAccelerationY(), mpu.getAccelerationZ()) * 180 / PI;
    float roll = atan2(-mpu.getAccelerationX(), sqrt(mpu.getAccelerationY() * mpu.getAccelerationY() + mpu.getAccelerationZ() * mpu.getAccelerationZ())) * 180 / PI;
    float yaw = 0; // Yaw calculation requires magnetometer data
    String json = "{\"pitch\":" + String(pitch) + ",\"roll\":" + String(roll) + ",\"yaw\":" + String(yaw) + "}";
    server.send(200, "application/json", json);
  });
  server.begin();
}

void loop() {
  server.handleClient();
}

/*
; PlatformIO Project Configuration File
;
;   Build options: build flags, source filter
;   Upload options: custom upload port, speed and extra flags
;   Library options: dependencies, extra library storages
;   Advanced options: extra scripting
;
; Please visit documentation for the other options and examples
; https://docs.platformio.org/page/projectconf.html

[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps = 
	WiFi
    WebServer
    MPU6050
*/
