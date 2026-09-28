#include <Servo.h>
#include <QTRSensors.h>

// --- PIN CONFIGURATION ---
// Romeo BLE Built-in Motor Driver Pins
const int E1 = 6;
const int M1 = 7;
const int E2 = 5;
const int M2 = 4;

// Peripheral Component Pins
const int TRIG_PIN = 9;
const int ECHO_PIN = 12;
const int SERVO_PIN = 10;  //????????????????????????????????????????????????

QTRSensors qtr;
Servo myservo;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

// --- CALIBRATION & THRESHOLDS ---
const int MAX_SPEED = 255;
const float PROXIMITY_LIMIT = 15.0;
const float MIN_DISTANCE = 5.0;
const int SERVO_FRONT = 90;   //??????????????????????????????????????????
const int SERVO_RIGHT = 180;  //??????????????????????????????????????????
const int MAX_POS = 1000 * SensorCount;

// Robot System States
enum RobotState {
  LINE_FOLLOWING,
  OBSTACLE_CIRCUMVENTION_START,
  OBSTACLE_CIRCUMVENTION,
  OBSTACLE_CIRCUMVENTION_END,
  STOP,
  BACKWARDS,
};

RobotState currentState = LINE_FOLLOWING;

bool backwards = false;

int command = 1;
int check = 1;

int currentSpeed = MAX_SPEED;

float lastDistance = 20.0;

bool progressCheck = false;
bool progressCheckNumber = 0;

// --- PD Gains (Tune these!) ---
float Kp = 5.0;  // Proportional gain
float Kd = 3.0;  // Derivative gain

// --- Variables ---
float lastError = 0;

uint16_t lastPosition = MAX_POS / 2;
uint16_t position = MAX_POS / 2;

void setup() {

  Serial.begin(115200);

  myservo.attach(SERVO_PIN);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5 }, SensorCount);
  qtr.setEmitterPin(2);
  delay(500);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);

  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);

  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(TRIG_PIN, HIGH);

  delay(1000);  // System safety settle time
}

void loop() {
  switch (currentState) {

    case LINE_FOLLOWING:
      {
        float forwardDist = getDistance();
        if (forwardDist > MIN_DISTANCE && forwardDist < PROXIMITY_LIMIT) {
          motorsStop();
          myservo.write(SERVO_RIGHT);
          delay(1000);
          setMotors(LOW, currentSpeed, HIGH, currentSpeed);
          progressCheckNumber = 0;
          currentState = OBSTACLE_CIRCUMVENTION_START;
          break;
        }

        position = qtr.readLineBlack(sensorValues);
        Serial.println(position);

        positionDrive(position);

        break;
      }
    case OBSTACLE_CIRCUMVENTION_START:
      {
        float distance = getDistance();
        if (distance > lastDistance) { progressCheckNumber++; }
        if (progressCheckNumber > 3) { currentState = OBSTACLE_CIRCUMVENTION; 
        progressCheckNumber = 0;}
        break;
      }
    case OBSTACLE_CIRCUMVENTION:
      {
        float currentDist = getDistance();

        // Calculate how far off we are (Error)
        // Assumes sensor is on the LEFT side.
        // If too close, error is negative. If too far, error is positive.
        float error = currentDist - PROXIMITY_LIMIT;

        // Calculate derivative (rate of change)
        float derivative = error - lastError;

        // Calculate the steering correction
        float correction = (Kp * error) + (Kd * derivative);

        // Constraint correction to prevent motor errors
        correction = constrain(correction, -100, 100);

        uint8_t posSubstitute = (correction + 100) * 30;

        positionDrive(posSubstitute);

        lastError = error;

        lastPosition = position;
        position = qtr.readLineBlack(sensorValues);
        if (position != lastPosition){progressCheckNumber++;}
        if (progressCheckNumber > 3)
        {currentState = OBSTACLE_CIRCUMVENTION_END;
        progressCheckNumber = 0;}
        break;
      }
    case OBSTACLE_CIRCUMVENTION_END:
      {
        myservo.write(SERVO_FRONT);
        delay(1000);
        setMotors(LOW, currentSpeed, HIGH, currentSpeed);
        delay(1000);
        currentState = LINE_FOLLOWING;
        break;
      }
    case STOP:
      {
        motorsStop();
        delay(1000);
        break;
      }
    case BACKWARDS:
      {
        setMotors(LOW, currentSpeed, LOW, currentSpeed);
        delay(2000);
        currentState = STOP;
        command = 2;
        check = 2;
        break;
      }
  }

  if (Serial.available() > 0) {
    int newValue = Serial.parseInt();
    if (newValue != 0) {
      command = newValue;
    }
  }
  delay(20);  // Small loop stabilization tick

  if (command != check) {
    if (command == 1) {
      currentState = LINE_FOLLOWING;
    }
    if (command == 2) {
      currentState = STOP;
    }
    if (command == 3) {
      currentState = BACKWARDS;
    }
    check = command;
  }
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

void positionDrive(int position) {
  if (position == (MAX_POS / 2)) {
    setMotors(HIGH, currentSpeed, HIGH, currentSpeed);
  }

  if (position < (MAX_POS / 4)) {
    setMotors(HIGH, currentSpeed, LOW, (currentSpeed * ((MAX_POS / 4) - position)) / (MAX_POS / 4));
  }

  if (position > ((3 * MAX_POS) / 4)) {
    setMotors(LOW, (currentSpeed * (position - (3 * MAX_POS) / 4)) / (MAX_POS / 4), HIGH, currentSpeed);
  }

  if (position < (MAX_POS / 2)) {
    setMotors(HIGH, currentSpeed, HIGH, currentSpeed - ((currentSpeed * ((MAX_POS / 2) - position)) / (MAX_POS / 4)));
  }

  if (position > (MAX_POS / 2)) {
    setMotors(HIGH, currentSpeed - ((currentSpeed * (position - (MAX_POS / 2))) / (MAX_POS / 4)), HIGH, currentSpeed);
  }
}

// --- SENSOR READING UTILITY FUNCTIONS ---

float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);  // 30ms timeout
  if (duration == 0) return lastDistance;          // Ignore timeouts to prevent wild jumps

  lastDistance = duration * 0.0343 / 2;
  return duration * 0.0343 / 2;
}
