#include <HardwareSerial.h>

#define IBUS_BUFFER_SIZE 32
uint8_t ibusBuffer[IBUS_BUFFER_SIZE];
uint16_t channels[14];  // FlySky supports up to 14 channels

HardwareSerial mySerial(1);  // UART1 (RX = GPIO16)

void setup() {
    Serial.begin(115200);  // Debug output
    mySerial.begin(115200, SERIAL_8N1, 16, -1);  // UART1, RX on GPIO16

    Serial.println("FlySky iBUS Receiver Ready!");
}

void loop() {
    if (mySerial.available() >= IBUS_BUFFER_SIZE) {  // Read iBUS frame (32 bytes)
        mySerial.readBytes(ibusBuffer, IBUS_BUFFER_SIZE);

        // Validate iBUS Frame (Header must be 0x20, 0x40)
        if (ibusBuffer[0] == 0x20 && ibusBuffer[1] == 0x40) {
            for (int i = 0; i < 14; i++) {
                channels[i] = ibusBuffer[2 + i * 2] | (ibusBuffer[3 + i * 2] << 8);
            }

            // Print channel values
            Serial.print("CH1: "); Serial.print(channels[0]);
            Serial.print(" | CH2: "); Serial.print(channels[1]);
            Serial.print(" | CH3: "); Serial.print(channels[2]);
            Serial.print(" | CH4: "); Serial.print(channels[3]);
            Serial.println();
        }
    }
}
