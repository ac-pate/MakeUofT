#include <Arduino.h>

// --- Pin definitions ---
#define VIB_PIN1        2     // Vibration motor 1 control pin (PWM-capable)
#define VIB_PIN2        12    // Vibration motor 2 control pin (PWM-capable)
#define IN1             3     // Control pin for driver (if needed)
#define IN3             11    // Control pin for driver (if needed)

// --- Fixed timing parameter ---
#define PULSE_ON_TIME    10    // Duration of the vibration pulse (ms)

// --- Distance mapping range ---
// The distances provided to the API are expected to be in this range.
#define MIN_DISTANCE_VALUE 0     
#define MAX_DISTANCE_VALUE 1000  

// Global configuration parameters (could also be passed every cycle)
int g_minOffTime;  // in ms, e.g., 50
int g_maxOffTime;  // in ms, e.g., 2000
int g_intensity;   // PWM intensity, e.g., 255

//---------------------------------------------------
// Structure to hold independent motor state
//---------------------------------------------------
struct MotorState {
  uint8_t pin;             // PWM pin for the motor
  int offTime;             // Computed off-time delay for this motor
  bool isPulsing;          // true if the motor is currently ON
  unsigned long lastUpdate; // Time when the motor state last changed
};

MotorState motor1 = {VIB_PIN1, 0, false, 0};
MotorState motor2 = {VIB_PIN2, 0, false, 0};

//---------------------------------------------------
// Function: initMotorControl
// Purpose: Initialize pins and set initial states.
//---------------------------------------------------
void initMotorControl() {
  // Set up control pins if needed
  pinMode(IN1, OUTPUT);
  pinMode(IN3, OUTPUT);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN3, HIGH);

  // Set up motor control pins
  pinMode(VIB_PIN1, OUTPUT);
  pinMode(VIB_PIN2, OUTPUT);
  
  // Use 8-bit PWM resolution (0 to 255)
  analogWriteResolution(8);
  
  // Turn off both motors initially
  analogWrite(VIB_PIN1, 0);
  analogWrite(VIB_PIN2, 0);
  
  // Initialize motor state timestamps
  motor1.lastUpdate = millis();
  motor2.lastUpdate = millis();
  motor1.isPulsing = false;
  motor2.isPulsing = false;
}

//---------------------------------------------------
// Function: mapDistanceToOffTime
// Purpose: Map a given distance (within MIN_DISTANCE_VALUE and MAX_DISTANCE_VALUE)
//          to an off-time delay between pulses, constrained between minOffTime and maxOffTime.
//          A smaller (closer) distance results in a shorter off-time.
//---------------------------------------------------
int mapDistanceToOffTime(int distance, int minOffTime, int maxOffTime) {
  int t = map(distance, MIN_DISTANCE_VALUE, MAX_DISTANCE_VALUE, minOffTime, maxOffTime);
  return constrain(t, minOffTime, maxOffTime);
}

//---------------------------------------------------
// Function: updateMotor
// Purpose: Update the state of a motor (pulse ON or OFF) non-blockingly.
// Parameters:
//    motor      - Reference to the motor state
//    distance   - The measured (or provided) distance for this motor
//    minOffTime - Minimum off-time delay (ms)
//    maxOffTime - Maximum off-time delay (ms)
//    intensity  - PWM intensity to use when pulsing (0-255)
//---------------------------------------------------
void updateMotor(MotorState &motor, int distance, int minOffTime, int maxOffTime, int intensity) {
  // Compute off-time for this motor based on distance.
  motor.offTime = mapDistanceToOffTime(distance, minOffTime, maxOffTime);
  
  unsigned long now = millis();
  
  if (motor.isPulsing) {
    // If the motor is ON, check if the pulse duration has elapsed.
    if (now - motor.lastUpdate >= PULSE_ON_TIME) {
      analogWrite(motor.pin, 0); // Turn off motor.
      motor.isPulsing = false;
      motor.lastUpdate = now;
    }
  } else {
    // If the motor is OFF, check if the off-time delay has passed.
    if (now - motor.lastUpdate >= motor.offTime) {
      analogWrite(motor.pin, intensity); // Turn motor ON.
      motor.isPulsing = true;
      motor.lastUpdate = now;
    }
  }
}

//---------------------------------------------------
// Function: controlMotors
// Purpose: Given two distance values, update each motor's state independently.
// Parameters:
//    distanceX  - Distance for motor 1.
//    distanceY  - Distance for motor 2.
//    minOffTime - Minimum off-time delay (ms).
//    maxOffTime - Maximum off-time delay (ms).
//    intensity  - PWM intensity for pulse.
//---------------------------------------------------
void controlMotors(int distanceX, int distanceY, int minOffTime, int maxOffTime, int intensity) {
  updateMotor(motor1, distanceX, minOffTime, maxOffTime, intensity);
  updateMotor(motor2, distanceY, minOffTime, maxOffTime, intensity);
}

//---------------------------------------------------
// Example usage in setup() and loop()
//---------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) {} // Wait for Serial port to be ready.
  Serial.println("Car Reverse Haptic Feedback - Independent Motor Pulsing");
  
  // Set desired off-time range and intensity.
  g_minOffTime = 50;
  g_maxOffTime = 2000;
  g_intensity = 255;
  
  initMotorControl();
}

void loop() {
  // For demonstration, we simulate two distance values.
  // In your real application, these would be provided by higher-level logic.
  int distanceX = 300;   // e.g., left side distance (closer -> faster pulses)
  int distanceY = 800;   // e.g., right side distance (farther -> slower pulses)

  // Optionally print the simulated distances.
  Serial.print("DistanceX: ");
  Serial.print(distanceX);
  Serial.print(" | DistanceY: ");
  Serial.println(distanceY);
  
  // Update both motors concurrently (non-blocking).
  controlMotors(distanceX, distanceY, g_minOffTime, g_maxOffTime, g_intensity);
  
  // Small delay for loop iteration.
  delay(10);
}
