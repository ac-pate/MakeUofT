#include <Arduino.h>
#include <ArduinoBLE.h>
#include "Arduino_BMI270_BMM150.h"

// --- Fall parameters ---
#define THRESHOLD_ACCELERATION 4  // Set threshold for high acceleration
#define OUTPUT_PIN 4

// --- Pin definitions ---
#define VIB_PIN_LEFT   2    // Left motor control pin (PWM-capable)
#define VIB_PIN_RIGHT  12   // Right motor control pin (PWM-capable)
#define IN1            3    // Control pin for driver (if needed)
#define IN3            11   // Control pin for driver (if needed)

// --- Fixed timing parameter ---
#define PULSE_ON_TIME   10   // Duration of the vibration pulse (ms)

// --- Distance mapping range ---
#define MIN_DISTANCE_VALUE 0     
#define MAX_DISTANCE_VALUE 1000  
#define STOP_DISTANCE 200  // Stop vibrating when command value is at or below 200 (i.e. object is close)

// Global configuration parameters
int g_minOffTime = 50;   // Minimum off-time delay (ms)
int g_maxOffTime = 2000; // Maximum off-time delay (ms)
int g_intensity = 255;   // PWM intensity (0-255)
BLEService emergencyService("6969");  // Custom service for emergency signal
BLECharacteristic distressSignal("2A56", BLEWrite | BLENotify, 1); // Emergency signal characteristic
byte distressSignalValue = 1;

float last_x = 0.0, last_y = 0.0, last_z = 0.0;
bool alarm_triggered = false;

//---------------------------------------------------
// Structure to hold independent motor state
//---------------------------------------------------
struct MotorState {
  uint8_t pin;
  int offTime;
  bool isPulsing;
  unsigned long lastUpdate;
};

MotorState leftMotor = {VIB_PIN_LEFT, 0, false, 0};
MotorState rightMotor = {VIB_PIN_RIGHT, 0, false, 0};

//---------------------------------------------------
// Function: initMotorControl
// Purpose: Initialize pins and set initial states.
//---------------------------------------------------
void initMotorControl() {
  pinMode(IN1, OUTPUT);
  pinMode(IN3, OUTPUT);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN3, HIGH);
  pinMode(VIB_PIN_LEFT, OUTPUT);
  pinMode(VIB_PIN_RIGHT, OUTPUT);
  analogWrite(VIB_PIN_LEFT, 0);
  analogWrite(VIB_PIN_RIGHT, 0);
}

//---------------------------------------------------
// Function: mapDistanceToOffTime
// Purpose: Map a given command value to an off-time delay
//---------------------------------------------------
int mapDistanceToOffTime(int distance) {
  return constrain(map(distance, MIN_DISTANCE_VALUE, MAX_DISTANCE_VALUE, g_maxOffTime, g_minOffTime), g_minOffTime, g_maxOffTime);
}

//---------------------------------------------------
// Function: updateMotor
// Purpose: Update the state of a motor non-blockingly.
//---------------------------------------------------
void updateMotor(MotorState &motor, int distance) {
  if (distance <= STOP_DISTANCE) {
    analogWrite(motor.pin, 0);
    return;
  }
  motor.offTime = mapDistanceToOffTime(distance);
  unsigned long now = millis();
  if (motor.isPulsing && (now - motor.lastUpdate >= PULSE_ON_TIME)) {
    analogWrite(motor.pin, 0);
    motor.isPulsing = false;
    motor.lastUpdate = now;
  } else if (!motor.isPulsing && (now - motor.lastUpdate >= motor.offTime)) {
    analogWrite(motor.pin, g_intensity);
    motor.isPulsing = true;
    motor.lastUpdate = now;
  }
}

//---------------------------------------------------
// Function: controlMotors
// Purpose: Update each motor's state independently based on the provided left and right command values.
//---------------------------------------------------
void controlMotors(int leftDistance, int rightDistance) {
  updateMotor(leftMotor, leftDistance);
  updateMotor(rightMotor, rightDistance);
}

//---------------------------------------------------
// Function: setup
// Purpose: Initialize system and serial communication.
//---------------------------------------------------
void setup() {
  Serial.begin(115200);
  initMotorControl();
  if (!IMU.begin()) {
      Serial.println("Failed to initialize IMU!");
      while (1);  // Stop execution if IMU initialization fails
  }
  pinMode(OUTPUT_PIN, OUTPUT);
  
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1);
  }
  //Configure BLE service
  BLE.setLocalName("EmergencyDevice");
  BLE.setAdvertisedService(emergencyService);
  emergencyService.addCharacteristic(distressSignal);
  BLE.addService(emergencyService);
  BLE.advertise();
}

//---------------------------------------------------
// Function: loop
// Purpose: Receive command values from Python and control motors.
//---------------------------------------------------
void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    int commaIndex = data.indexOf(',');
    if (commaIndex > 0) {
      int leftDistance = data.substring(0, commaIndex).toInt();
      int rightDistance = data.substring(commaIndex + 1).toInt();
      controlMotors(leftDistance, rightDistance);
    }
  }

    BLE.poll(); // Keep BLE communication alive
    float x, y, z;
    //Check for alert
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(x, y, z);
        if (sqrt(sq(x) + sq(y) + sq(z)) >= THRESHOLD_ACCELERATION) {
            Serial.println(sqrt(sq(x) + sq(y) + sq(z)));
            tone(OUTPUT_PIN, 262, 3000);
            distressSignal.writeValue(&distressSignalValue, sizeof(distressSignalValue));
        }
        last_x = x;
        last_y = y;
        last_z = z;
    }
}
