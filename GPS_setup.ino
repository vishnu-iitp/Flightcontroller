#include <TinyGPS++.h>
#include <HardwareSerial.h>

// Create GPS object
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);  // UART1 for GPS

void setup() {
    Serial.begin(115200);     // Serial monitor
    gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // GPS: 9600 baud, RX=16, TX=17

    Serial.println("GPS module setup...");
}

void loop() {
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
    }

    if (gps.location.isUpdated()) {
        Serial.print("Latitude: ");
        Serial.print(gps.location.lat(), 6);
        Serial.print(" Longitude: ");
        Serial.println(gps.location.lng(), 6);
    }

    if (gps.date.isUpdated()) {
        Serial.print("Date: ");
        Serial.print(gps.date.day());
        Serial.print("/");
        Serial.print(gps.date.month());
        Serial.print("/");
        Serial.println(gps.date.year());
    }

    if (gps.time.isUpdated()) {
        Serial.print("Time: ");
        Serial.print(gps.time.hour());
        Serial.print(":");
        Serial.print(gps.time.minute());
        Serial.print(":");
        Serial.println(gps.time.second());
    }
}
/*; PlatformIO Project Configuration File
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
	;pribanosati/mrm-can-bus@^0.0.6
	;pribanosati/mrm-robot@^0.0.22
	;pribanosati/mrm-lid-d@^0.0.1
	mikalhart/TinyGPSPlus*/
