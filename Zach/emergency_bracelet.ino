#include <ArduinoBLE.h>
#include "Arduino_BMI270_BMM150.h"

#define THRESHOLD_ACCELERATION 2.0  // Set threshold for high acceleration
#define OUTPUT_PIN 3

BLEService emergencyService("6969");  // Custom service for emergency signal
BLECharacteristic distressSignal("2A56", BLEWrite | BLERead, 1); // Emergency signal characteristic
byte distressSignalValue = 1;  // Using a byte for a single value

float last_x = 0.0, last_y = 0.0, last_z = 0.0;
bool alarm_triggered = false;

void setup() {
  Serial.begin(9600);
    while (!Serial);  // Wait for Serial to initialize

  if (!IMU.begin()) {
      Serial.println("Failed to initialize IMU!");
      while (1);  // Stop execution if IMU initialization fails
  }
  pinMode(OUTPUT_PIN, OUTPUT);
  
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1);
  }
  
  BLE.setLocalName("EmergencyDevice");
  BLE.setAdvertisedService(emergencyService);
  emergencyService.addCharacteristic(distressSignal);
  BLE.addService(emergencyService);
  BLE.advertise();
  
  Serial.println("BLE Emergency Device Started.");
}

void loop() {
  BLE.poll(); // Keep BLE communication alive
    float x, y, z;

    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(x, y, z);
        if (sqrt(sq(x) + sq(y) + sq(z)) >= THRESHOLD_ACCELERATION) {
            tone(OUTPUT_PIN, 262, 3000);
            distressSignal.writeValue(&distressSignalValue, sizeof(distressSignalValue));

        }

        last_x = x;
        last_y = y;
        last_z = z;
    }

}
