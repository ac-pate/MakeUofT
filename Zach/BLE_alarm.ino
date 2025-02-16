#include <ArduinoBLE.h>
#include "Arduino_BMI270_BMM150.h"

#define THRESHOLD_ACCELERATION 3.0
#define OUTPUT_PIN 2

BLEService emergencyService("2025");
BLECharacteristic distressSignal("2A56", BLEWrite | BLENotify, 1); 
byte distressSignalValue = 1; 

float last_x = 0.0, last_y = 0.0, last_z = 0.0;
bool alarm_triggered = false;

void setup() {
  // Serial.begin(9600);
  // while (!Serial);

  // if (!IMU.begin()) {
  //   Serial.println("Failed to initialize IMU!");
  //   while (1);
  // }

  pinMode(OUTPUT_PIN, OUTPUT);
  
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
  }
  
  BLE.setLocalName("EmergencyDevice");
  BLE.setAdvertisedService(emergencyService);
  emergencyService.addCharacteristic(distressSignal);
  BLE.addService(emergencyService);
  BLE.advertise();
  }

void loop() {
  BLE.poll(); // Keep BLE communication alive

  float x, y, z;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
      // Serial.println(sqrt(sq(x) + sq(y) + sq(z)));
    if (sqrt(sq(x) + sq(y) + sq(z)) >= THRESHOLD_ACCELERATION) {
      tone(OUTPUT_PIN, 262, 3000);
      distressSignal.writeValue(&distressSignalValue, sizeof(distressSignalValue));
    }

    last_x = x;
    last_y = y;
    last_z = z;
  }
}
