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
const int SERVO_PIN = 10;
const int BUTTON_PIN = A7;

QTRSensors qtr;
Servo myservo;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

// --- CALIBRATION & THRESHOLDS ---
const int MAX_SPEED = 255;
const float PROXIMITY_LIMIT = 15.0;
const float MIN_DISTANCE = 5.0;
const int SERVO_FRONT = 90;
const int SERVO_RIGHT = 0;
const int MAX_POS = 1000 * (SensorCount - 1);

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

float distance = 20.0;
float lastDistance = 20.0;


bool progressCheck = false;
bool progressCheckNumber = 0;

float Kp = 5.0;  // Proportional gain
float Kd = 3.0;  // Derivative gain

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

  currentSpeed = 150;

  delay(1000);  // System safety settle time
}

void loop() {
  switch (currentState) {

    case LINE_FOLLOWING:
      {
        Serial.print("LINE_FOLLOWING:");
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
        for (uint8_t i = 0; i < SensorCount; i++) {
          Serial.print(sensorValues[i]);
          Serial.print('\t');
        }
        Serial.println(position);

        positionDrive(position);

        break;
      }
    case OBSTACLE_CIRCUMVENTION_START:
      {
        Serial.print("OBSTACLE_CIRCUMVENTION_START:");
        distance = getDistance();
        delay(10);
        if (distance < 30.0) {
          currentState = OBSTACLE_CIRCUMVENTION;
        }
        break;
      }
    case OBSTACLE_CIRCUMVENTION:
      {
        Serial.print("OBSTACLE_CIRCUMVENTION:");
        float currentDist = getDistance();

        float error = currentDist - PROXIMITY_LIMIT;

        float derivative = error - lastError;

        float correction = (Kp * error) + (Kd * derivative);

        correction = constrain(correction, -100, 100);

        int posSubstitute = (correction + 100) * 25;

        Serial.print("PosSubstitute:");
        Serial.print(posSubstitute);
        Serial.print("\n");
        positionDrive(posSubstitute);

        lastError = error;

        position = qtr.readLineBlack(sensorValues);
        for (uint8_t i = 0; i < SensorCount; i++) {

          if (sensorValues[i] > 500) {
            currentState = OBSTACLE_CIRCUMVENTION_END;
          }
        }

        break;
      }
    case OBSTACLE_CIRCUMVENTION_END:
      {
        Serial.print("OBSTACLE_CIRCUMVENTION_END:");
        myservo.write(SERVO_FRONT);
        delay(1000);
        setMotors(LOW, currentSpeed, HIGH, currentSpeed);
        delay(1500);
        currentState = LINE_FOLLOWING;
        break;
      }
    case STOP:
      {
        Serial.print("STOP:");
        motorsStop();
        delay(1000);
        break;
      }
    case BACKWARDS:
      {
        Serial.print("BACKWARDS:");
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

  int buttonValue = analogRead(BUTTON_PIN);

  if (buttonValue >= 125 && buttonValue <= 165) {
    command = 1;
  } else if (buttonValue >= 310 && buttonValue <= 350) {
    command = 2;
  }

  delay(25);

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
  float max_pos_f = (float)MAX_POS;
  float current_speed_f = (float)currentSpeed;

  if (position == (MAX_POS / 2)) {
    setMotors(HIGH, currentSpeed, HIGH, currentSpeed);
  }

  else if (position < (MAX_POS / 4)) {
    float targetSpeed = (current_speed_f * ((max_pos_f / 4.0f) - position)) / (max_pos_f / 4.0f);
    setMotors(LOW, (int)targetSpeed, HIGH, currentSpeed);
  }

  else if (position > ((3 * MAX_POS) / 4)) {
    float targetSpeed = (current_speed_f * (position - (3.0f * max_pos_f / 4.0f))) / (max_pos_f / 4.0f);
    setMotors(HIGH, currentSpeed, LOW, (int)targetSpeed);
  }

  else if (position < (MAX_POS / 2)) {
    float targetSpeed = current_speed_f - ((current_speed_f * ((max_pos_f / 2.0f) - position)) / (max_pos_f / 4.0f));
    setMotors(HIGH, (int)targetSpeed, HIGH, currentSpeed);
  }

  else if (position > (MAX_POS / 2)) {
    float targetSpeed = current_speed_f - ((current_speed_f * (position - (max_pos_f / 2.0f))) / (max_pos_f / 4.0f));
    setMotors(HIGH, currentSpeed, HIGH, (int)targetSpeed);
  }
}

float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  digitalWrite(TRIG_PIN, HIGH);
  unsigned long LowLevelTime = pulseIn(ECHO_PIN, LOW);
  float lowLevelTime = (float)LowLevelTime;
  if (lowLevelTime >= 50000) {
    Serial.println("Out of Range");
    return lastDistance;
  }
  lastDistance = lowLevelTime / 50;
  return lowLevelTime / 50;
}
