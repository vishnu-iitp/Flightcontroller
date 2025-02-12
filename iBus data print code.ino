#include <HardwareSerial.h>

HardwareSerial mySerial(2);  // Using UART2 (RX on GPIO16)

#define IBUS_BUFFER_SIZE 32
uint8_t ibusBuffer[IBUS_BUFFER_SIZE];
uint16_t channels[14];  // FlySky supports up to 14 channels

void setup() {
  Serial.begin(115200);  // For debugging
  mySerial.begin(115200, SERIAL_8N1, 16, -1);  // UART2, RX on GPIO16

  Serial.println("FlySky iBUS Receiver Ready!");
}

void loop() {
  if (mySerial.available() >= IBUS_BUFFER_SIZE) {  // Read iBUS frame (32 bytes)
    mySerial.readBytes(ibusBuffer, IBUS_BUFFER_SIZE);

    if (ibusBuffer[0] == 0x20 && ibusBuffer[1] == 0x40) {  // iBUS Frame Header Check
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
