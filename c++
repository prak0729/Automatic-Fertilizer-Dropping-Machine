// Project: Automatic Targeted Fertilizer Dropping Machine (AFDM)
// Microcontroller: Arduino UNO
// Guide: Prof. Ashwini Wali (As per report)

#include <Servo.h> 

// --- Pin Definitions ---
// Ultrasonic Sensor (HC-SR04)
[cite_start]const int trigPin = 8; [cite: 1138]
[cite_start]const int echoPin = 9; [cite: 1139]

// Servo Motor (Fertilizer Dispenser)
Servo myservo; [cite_start]// Servo object [cite: 1139]
const int servoPin = 10;
int pos = 90; [cite_start]// variable to store the servo position (Closed/Default position) [cite: 1139]
const int dispensePos = 0; // Open/Dispense position (Adjust as needed)
const int closedPos = 90; // Closed/Default position

// Motor Driver (e.g., L298N) Pins (Motors)
const int pwm = 3; [cite_start]// PWM pin for motor speed control [cite: 1140]
const int left_motor_1 = 4; [cite_start]// Left motor IN1 [cite: 1140, 1141]
const int left_motor_2 = 5; [cite_start]// Left motor IN2 [cite: 1141]
const int right_motor_1 = 6; [cite_start]// Right motor IN3 (inferred from common practice, was '6' in original) [cite: 1141]
const int right_motor_2 = 7; [cite_start]// Right motor IN4 (inferred from common practice, was '7' in original) [cite: 1142]

// ESP32-CAM Plant Detection Signal
const int img = 12; [cite_start]// Input pin for plant detection signal from ESP32-CAM [cite: 1142]

// --- Global Variables ---
char BluetoothData; [cite_start]// The Bluetooth data received [cite: 1140]
bool automode = false; [cite_start]// Flag to toggle between manual and autonomous modes [cite: 1142]
const int motor_speed = 110; // Motor speed (0-255). [cite_start]Set at 110 in setup. [cite: 1166, 1168]
const int stop_distance_cm = 10; // Distance to stop for fertilizer drop (Adjust as needed)

// --- Function Declarations ---
void forward();
void backward();
void left();
void right();
void stop_motor();
long us(); // Ultrasonic distance measurement

// --- SETUP ---
void setup() {
  [cite_start]myservo.attach(servoPin); [cite: 1143]
  [cite_start]Serial.begin(9600); [cite: 1143]
  myservo.write(closedPos); [cite_start]// Close the dispenser initially [cite: 1143, 1144]

  // Set motor control pins as OUTPUT
  [cite_start]pinMode(left_motor_1, OUTPUT); [cite: 1144]
  pinMode(left_motor_2, OUTPUT);
  pinMode(right_motor_1, OUTPUT);
  pinMode(right_motor_2, OUTPUT);
  [cite_start]pinMode(pwm, OUTPUT); [cite: 1166]
  
  // Set Ultrasonic pins
  [cite_start]pinMode(trigPin, OUTPUT); [cite: 1144]
  [cite_start]pinMode(echoPin, INPUT); [cite: 1144]
  
  // Set ESP32-CAM signal pin
  [cite_start]pinMode(img, INPUT); [cite: 1144]
  
  // Stop all motors initially
  [cite_start]digitalWrite(left_motor_1, LOW); [cite: 1166]
  [cite_start]digitalWrite(left_motor_2, LOW); [cite: 1166]
  [cite_start]digitalWrite(right_motor_1, LOW); [cite: 1166]
  [cite_start]digitalWrite(right_motor_2, LOW); [cite: 1166]
  
  // Set initial motor speed
  analogWrite(pwm, motor_speed); [cite_start]// 110 is the set speed for 0-255 range [cite: 1166]
}

// --- MAIN LOOP ---
void loop() {
  
  // 1. Bluetooth Manual Control Logic
  if (Serial.available()) {
    BluetoothData = Serial.read(); [cite_start]// Get next character from bluetooth [cite: 1167]

    // Manual Movement Commands (Based on common 'Series Bluetooth Terminal' inputs)
    [cite_start]if (BluetoothData == '1') { // Assuming '1' is Forward [cite: 1168]
      automode = false;
      forward();
    [cite_start]} else if (BluetoothData == '2') { // Assuming '2' is Backward [cite: 1168]
      automode = false;
      backward();
    } else if (BluetoothData == '3') { // Inferred: Left
      automode = false;
      left();
    } else if (BluetoothData == '4') { // Inferred: Right
      automode = false;
      right();
    } else if (BluetoothData == '5') { // Inferred: Stop
      automode = false;
      stop_motor();
    } else if (BluetoothData == 'A') { // Inferred: Toggle Autonomous Mode
      automode = !automode;
      stop_motor();
      delay(500);
    }
  }

  // 2. Autonomous Mode Logic
  if (automode) {
    int plant_detected = digitalRead(img); // Check if ESP32-CAM detects a plant
    long distance_cm = us(); // Get distance to object

    [cite_start]if (plant_detected == HIGH) { // Plant detected [cite: 1248]
      
      if (distance_cm > stop_distance_cm && distance_cm < 200) {
        [cite_start]// Move forward if plant is detected but not close enough [cite: 1249]
        forward();
      } else if (distance_cm <= stop_distance_cm) {
        [cite_start]// Stop and drop fertilizer if close enough [cite: 1250]
        stop_motor();
        
        [cite_start]// Dispense Fertilizer [cite: 1250]
        myservo.write(dispensePos); // Open the dispenser
        delay(1500); // Wait for fertilizer to drop
        myservo.write(closedPos); // Close the dispenser
        delay(500); // Wait for servo to close

        // Move away/to the next plant (optional, but logical for continuous operation)
        // backward(); 
        // delay(500);
        // stop_motor();
        
        // Continue moving forward to look for next plant
        forward();
      } else {
        // If plant detected, but distance measurement is too far (e.g., > 200cm), just move forward slowly
        forward();
      }
    } else {
      // No plant detected, move forward slowly or stop (move forward is more practical for searching)
      forward();
    }
  }

  delay(50); // Small delay to prevent jitter
}

// --- HELPER FUNCTIONS ---

// Motor Control Functions
void forward() {
  analogWrite(pwm, motor_speed);
  digitalWrite(left_motor_1, HIGH);
  digitalWrite(left_motor_2, LOW);
  digitalWrite(right_motor_1, HIGH);
  digitalWrite(right_motor_2, LOW);
}

void backward() {
  analogWrite(pwm, motor_speed);
  digitalWrite(left_motor_1, LOW);
  digitalWrite(left_motor_2, HIGH);
  digitalWrite(right_motor_1, LOW);
  digitalWrite(right_motor_2, HIGH);
}

void left() {
  analogWrite(pwm, motor_speed);
  digitalWrite(left_motor_1, LOW);
  digitalWrite(left_motor_2, HIGH);
  digitalWrite(right_motor_1, HIGH);
  digitalWrite(right_motor_2, LOW);
}

void right() {
  analogWrite(pwm, motor_speed);
  digitalWrite(left_motor_1, HIGH);
  digitalWrite(left_motor_2, LOW);
  digitalWrite(right_motor_1, LOW);
  digitalWrite(right_motor_2, HIGH);
}

void stop_motor() {
  digitalWrite(left_motor_1, LOW);
  digitalWrite(left_motor_2, LOW);
  digitalWrite(right_motor_1, LOW);
  digitalWrite(right_motor_2, LOW);
}

// Ultrasonic Sensor Function
long us() {
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH for 10 microsecond
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH);
  // Calculating the distance (Speed of sound is 343m/s or 0.034cm/µs)
  long distance = duration * 0.034 / 2; 
  return distance;
}
