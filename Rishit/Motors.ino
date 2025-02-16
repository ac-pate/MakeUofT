#include <Arduino.h>

// --- Pin definitions ---
#define VIB_PIN1        2     // Vibration motor control pin (PWM-capable)
#define SENSOR_PIN     A0    // Analog input for the distance sensor (0-4095 range)
#define IN1             3    // Additional digital pin (set HIGH as needed)

#define VIB_PIN2        12     // Vibration motor control pin (PWM-capable)
#define IN3             11    // Additional digital pin (set HIGH as needed)


// --- Haptic pulse parameters ---
#define PULSE_ON_TIME   10    // Motor ON time in milliseconds (duration of vibration pulse)
#define MIN_OFF_TIME    50    // Minimum pause (ms) when object is very close
#define MAX_OFF_TIME    2000  // Maximum pause (ms) when object is far away

// --- Vibration intensity (PWM duty cycle value) ---
#define VIB_INTENSITY   255   // Full intensity (for an 8-bit PWM, 255 is full ON)

void setup() {
  Serial.begin(115200);
  Serial.println("Car Reverse Haptic Feedback Demo Starting...");
  
  // Set SENSOR_PIN as input (analogRead sets the mode automatically, but we'll be explicit)
  pinMode(SENSOR_PIN, INPUT);
  
  // Set IN pin as output and drive it HIGH (if used for controlling the driver, for example)
  pinMode(IN1, OUTPUT);
  pinMode(IN3, OUTPUT);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN3, HIGH);
  
  // Set the vibration motor pin as output
  pinMode(VIB_PIN1, OUTPUT);
  pinMode(VIB_PIN2, OUTPUT);
  
  // Optionally set the analogWrite resolution to 8 bits (default on many boards)
  analogWriteResolution(8);  // Now analogWrite() will use values from 0 to 255
  
  // Start with the motor off
  analogWrite(VIB_PIN1, 0);
  analogWrite(VIB_PIN2, 0);
}

void loop() {
  // Read the distance sensor (expected range: 0 to 4095)
  int sensorValue = analogRead(SENSOR_PIN);
  Serial.print("Sensor Value: "); 
  Serial.println(sensorValue);
  
  // Map the sensor reading to a delay (off time) between pulses.
  // When sensorValue is low (object is very close) → short off time (fast pulses).
  // When sensorValue is high (object is far) → long off time (slow pulses).
  int offTime = map(sensorValue, 0, 4095, MIN_OFF_TIME, MAX_OFF_TIME);
  offTime = constrain(offTime, MIN_OFF_TIME, MAX_OFF_TIME);
  Serial.print("Beep Off Time: ");
  Serial.println(offTime);
  
  // Activate the vibration motor for a short pulse at full intensity
  analogWrite(VIB_PIN1, VIB_INTENSITY);
  analogWrite(VIB_PIN2, VIB_INTENSITY);
  delay(PULSE_ON_TIME);
  
  // Turn off the motor (stop vibration)
  analogWrite(VIB_PIN1, 0);
  analogWrite(VIB_PIN2, 0);
  
  // Wait for the computed off-time before repeating the pulse
  delay(offTime);
}
