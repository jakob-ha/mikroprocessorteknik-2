#include <SoftwareSerial.h>
#include <Servo.h>

// --- PIN CONFIGURATION ---
// Romeo BLE Built-in Motor Driver Pins
const int E1 = 5;     // M1 Speed Control (Left Motors)
const int M1 = 4;     // M1 Direction Control
const int E2 = 6;     // M2 Speed Control (Right Motors)
const int M2 = 7;     // M2 Direction Control

// Peripheral Component Pins
const int QTR_LED_PIN = 4; // QTR IR LED Emitter Control
const int SERVO_PIN = 9;   // DSS-P05 Servo Signal Pin

// URM37 Ultrasonic Pins (SoftwareSerial)
SoftwareSerial urmSerial(2, 3); // RX (Pin 2), TX (Pin 3)
Servo myServo;

// --- CALIBRATION & THRESHOLDS ---
const int LINE_THRESHOLD = 500;  // Analog values > 500 mean black line
const int BASE_SPEED = 130;      // Base motor speed (0-255)
const int TURN_SPEED = 90;       // Speed during sharp pivot turns
const int PROXIMITY_LIMIT = 20;  // Stop distance in centimeters

// Robot System States
enum RobotState {
  LINE_FOLLOWING,
  OBSTACLE_CHECK,
  BYPASS_MANEUVER
};
RobotState currentState = LINE_FOLLOWING;

// URM37 Request Command Packet
uint8_t pingCmd[4] = {0x22, 0x00, 0x00, 0x22}; 

void setup() {
  // Setup Motor Control Pins
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);
  
  // Setup QTR Emitter Pin
  pinMode(QTR_LED_PIN, OUTPUT);
  digitalWrite(QTR_LED_PIN, HIGH); // Turn on IR illumination
  
  // Setup Servo
  myServo.attach(SERVO_PIN);
  myServo.write(90); // Center the ultrasonic sensor straight ahead
  
  // Initialize Serial Connections
  Serial.begin(9600);    // Hardware serial for PC tracking
  urmSerial.begin(9600); // Software serial for URM37 data
  
  delay(1000); // System safety settle time
}

void loop() {
  switch (currentState) {
    
    case LINE_FOLLOWING: {
      // 1. Scan for upfront blockages
      int forwardDist = getDistance();
      if (forwardDist > 0 && forwardDist < PROXIMITY_LIMIT) {
        motorsStop();
        currentState = OBSTACLE_CHECK;
        break;
      }
      
      // 2. Read central 4 sensors of QTR-8 for navigation guidance
      int leftOuter  = analogRead(A0);
      int leftInner  = analogRead(A1);
      int rightInner = analogRead(A2);
      int rightOuter = analogRead(A3);
      
      // 3. Evaluate positioning relative to the line
      if (leftInner > LINE_THRESHOLD && rightInner > LINE_THRESHOLD) {
        // Centered perfectly on line -> Drive Straight
        setMotors(HIGH, BASE_SPEED, HIGH, BASE_SPEED);
      } 
      else if (leftInner > LINE_THRESHOLD) {
        // Drifting right -> Steer slightly left
        setMotors(HIGH, BASE_SPEED - 40, HIGH, BASE_SPEED + 20);
      } 
      else if (rightInner > LINE_THRESHOLD) {
        // Drifting left -> Steer slightly right
        setMotors(HIGH, BASE_SPEED + 20, HIGH, BASE_SPEED - 40);
      }   
      else if (leftOuter > LINE_THRESHOLD) {
        // Sharp left departure -> Sharp pivot left
        setMotors(LOW, TURN_SPEED, HIGH, TURN_SPEED);
      } 
      else if (rightOuter > LINE_THRESHOLD) {
        // Sharp right departure -> Sharp pivot right
        setMotors(HIGH, TURN_SPEED, LOW, TURN_SPEED);
      }
      else {
        // Lost line completely -> Creep forward slowly to reacquire
        setMotors(HIGH, 80, HIGH, 80);
      }
      break;
    }

    case OBSTACLE_CHECK: {
      // Look left
      myServo.write(30);
      delay(500);
      int leftDist = getDistance();
      
      // Look right
      myServo.write(150);
      delay(500);
      int rightDist = getDistance();
      
      // Return sensor to front index
      myServo.write(90);
      delay(300);
      
      // Decision Logic: Maneuver to the side with more clearance
      if (leftDist > rightDist) {
        bypassObstacle(true);  // Circumvent via the left side
      } else {
        bypassObstacle(false); // Circumvent via the right side
      }
      
      currentState = LINE_FOLLOWING; // Reset back to track finding
      break;
    }
  }
  delay(20); // Small loop stabilization tick
}

// --- NAVIGATION UTILITY FUNCTIONS ---

// Helper function to commit wheel states cleanly
void setMotors(int leftDir, int leftSpeed, int rightDir, int rightSpeed) {
  digitalWrite(M1, leftDir);
  analogWrite(E1, leftSpeed);
  digitalWrite(M2, rightDir);
  analogWrite(E2, rightSpeed);
}

void motorsStop() {
  analogWrite(E1, 0);
  analogWrite(E2, 0);
}

// Executes a timed dead-reckoning curve around the object to find the line again
void bypassObstacle(boolean goLeft) {
  if (goLeft) {
    // 1. Pivot Left away from obstacle
    setMotors(LOW, TURN_SPEED, HIGH, TURN_SPEED);  delay(600);
    // 2. Arc Forward around the obstacle
    setMotors(HIGH, BASE_SPEED + 30, HIGH, BASE_SPEED - 30); delay(1200);
    // 3. Pivot Right back towards original vector
    setMotors(HIGH, TURN_SPEED, LOW, TURN_SPEED);  delay(600);
  } else {
    // 1. Pivot Right away from obstacle
    setMotors(HIGH, TURN_SPEED, LOW, TURN_SPEED);  delay(600);
    // 2. Arc Forward around the obstacle
    setMotors(HIGH, BASE_SPEED - 30, HIGH, BASE_SPEED + 30); delay(1200);
    // 3. Pivot Left back towards original vector
    setMotors(LOW, TURN_SPEED, HIGH, TURN_SPEED);  delay(600);
  }
  motorsStop();
  delay(200);
}

// --- SENSOR READING UTILITY FUNCTIONS ---

int getDistance() {
  // Clear lingering data from software serial rx buffer
  while(urmSerial.available()) urmSerial.read();
  
  // Issue request packet
  urmSerial.write(pingCmd, 4);
  
  // Block with timeout for returning 4-byte string
  unsigned long start = millis();
  while(urmSerial.available() < 4) {
    if (millis() - start > 80) return -1; // Serial timeout protection
  }
  
  uint8_t header = urmSerial.read();
  uint8_t highByte = urmSerial.read();
  uint8_t lowByte = urmSerial.read();
  uint8_t checksum = urmSerial.read();
  
  // Reconstruct 16-bit integer value
  int highInt = highByte;
  int lowInt = lowByte;
  int cm = (highInt << 8) + lowInt;
  
  return cm;
}
