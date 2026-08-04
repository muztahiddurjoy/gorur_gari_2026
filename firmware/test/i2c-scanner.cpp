#include <Arduino.h>
#include <Wire.h>

// Uncomment and adjust these lines if you want to use custom SDA/SCL pins
// #define I2C_SDA 8
// #define I2C_SCL 9

// If using the second I2C controller (Wire1), define this
// #define USE_WIRE1

void setup() {
  Serial.begin(115200);
  // Wait for serial monitor to connect (optional, only needed for native USB)

}

void loop() {
  delay(2000);

  Serial.println("\n--- I2C Scanner ---");

#ifdef USE_WIRE1
  // Initialize I2C bus 1 (default pins: SDA=10, SCL=11 on many ESP32-S3 boards)
  Wire1.begin();
  Serial.println("Scanning on I2C bus 1 (Wire1)...");
#else
  // Initialize I2C bus 0 (default pins: SDA=8, SCL=9 on many ESP32-S3 boards)
  #if defined(I2C_SDA) && defined(I2C_SCL)
    Wire.begin(I2C_SDA, I2C_SCL);
  #else
    Wire.begin();
  #endif
  Serial.println("Scanning on I2C bus 0 (Wire)...");
#endif

  Serial.println("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F");
  Serial.print("0x0");
  for (uint8_t addr = 0; addr < 16; addr++) {
    Serial.print("   -");
  }
  Serial.println();

  uint8_t deviceCount = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    if (addr % 16 == 0) {
      Serial.printf("\n0x%X", addr);
    }

    #ifdef USE_WIRE1
      Wire1.beginTransmission(addr);
      uint8_t error = Wire1.endTransmission();
    #else
      Wire.beginTransmission(addr);
      uint8_t error = Wire.endTransmission();
    #endif

    if (error == 0) {
      Serial.printf(" 0x%02X", addr);
      deviceCount++;
    } else {
      Serial.print("  --");
    }
  }
  Serial.printf("\n\nScan complete. %d device(s) found.\n", deviceCount);
    delay(3000);
}