#include <QTRSensors.h>

// --- PIN CONFIGURATION ---
// Romeo BLE Built-in Motor Driver Pins
const int E1 = 6;     // M1 Speed Control (Left Motors)
const int M1 = 7;     // M1 Direction Control
const int E2 = 5;     // M2 Speed Control (Right Motors)
const int M2 = 4;     // M2 Direction Control

// Peripheral Component Pins
const int triggerPin = 9; // Connected to URM37 COMP/TRIG
const int echoPin = 12;    // Connected to URM37 ECHO

QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

// --- CALIBRATION & THRESHOLDS ---
const int LINE_THRESHOLD = 750;  // Analog values > 500 mean black line
const int BASE_SPEED = 180;      // Base motor speed (0-255)
const int TURN_SPEED = 200;       // Speed during sharp pivot turns
const int PROXIMITY_LIMIT = 20;  // Stop distance in centimeters

// Robot System States
enum RobotState {
  LINE_FOLLOWING,
  OBSTACLE_CHECK,
};

RobotState currentState = LINE_FOLLOWING;


void setup() {

  Serial.begin(9600);
  
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A1 ,A2 ,A3, A4, A5}, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);

  // Setup Pins
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);

  digitalWrite(triggerPin, HIGH);
    
  delay(1000); // System safety settle time
}

void loop() {
  switch (currentState) {
    
    case LINE_FOLLOWING: {
      // 1. Scan for upfront blockages
      int forwardDist = getDistance();
      if (forwardDist > 15 && forwardDist < PROXIMITY_LIMIT) {
        motorsStop();
        currentState = OBSTACLE_CHECK;
        break;
      }
      
      uint16_t position = qtr.readLineBlack(sensorValues);

      // 2. Read central 4 sensors of QTR-8 for navigation guidance
      int leftOuter  = sensorValues[0];
      int leftInner  = sensorValues[1];
      int middle = sensorValues[2];
      int rightInner = sensorValues[3];
      int rightOuter = sensorValues[4];
      
      // 3. Evaluate positioning relative to the line
      if ((leftInner > LINE_THRESHOLD && middle > LINE_THRESHOLD) || (middle > LINE_THRESHOLD && rightInner > LINE_THRESHOLD)) {
        // Centered perfectly on line -> Drive Straight
        setMotors(HIGH, BASE_SPEED, HIGH, BASE_SPEED);
      } 
      else if (leftInner > LINE_THRESHOLD) {
        // Drifting right -> Steer slightly left
        setMotors(HIGH, BASE_SPEED -100, HIGH, BASE_SPEED + 40);
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
        setMotors(HIGH, 130, HIGH, 130);
      }
      break;
    }

    case OBSTACLE_CHECK: {
        bypassObstacle(true);  // Circumvent via the left side 
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
  motorsStop();
  delay(5000);}
}

// --- SENSOR READING UTILITY FUNCTIONS ---

int getDistance() {
  // Clear lingering data from software serial rx buffer
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(10); 
  digitalWrite(triggerPin, HIGH);
  
  // Measure the duration of the low pulse returned by the ECHO pin
  // URM37 outputs a low pulse proportional to distance: 50us per centimeter
  unsigned long duration = pulseIn(echoPin, LOW, 30000); // 30ms timeout
  
  if (duration == 0) {
    Serial.println("Error: No pulse detected / Out of range");
    return 100;
  } else {
    // Calculate distance in centimeters
    unsigned long distance = duration / 50; 
    
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  
  return distance;}
}
