
// --- PIN CONFIGURATION ---
// Romeo BLE Built-in Motor Driver Pins
const int E1 = 5;  // M1 Speed Control (Left Motors)
const int M1 = 4;  // M1 Direction Control
const int E2 = 6;  // M2 Speed Control (Right Motors)
const int M2 = 7;  // M2 Direction Control

// --- CALIBRATION & THRESHOLDS ---
const int BASE_SPEED = 180;  // Base motor speed (0-255)
const int TURN_SPEED = 255;  // Speed during sharp pivot turns

void setup() {

  Serial.begin(9600);

  Serial.println("init");
  // Setup Pins
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);

  delay(1000);  // System safety settle time
}

void loop() {
  Serial.print("Checkpoint middle \n");
  setMotors(HIGH, BASE_SPEED, HIGH, BASE_SPEED);
  delay(5000);
  Serial.print("Checkpoint drifting left \n");
  setMotors(HIGH, BASE_SPEED - 100, HIGH, BASE_SPEED + 40);
  delay(5000);
  Serial.print("Checkpoint drifting right \n");
  setMotors(HIGH, BASE_SPEED + 40, HIGH, BASE_SPEED - 100);
  delay(5000);
  Serial.print("Checkpoint departing left \n");
  setMotors(LOW, TURN_SPEED, HIGH, TURN_SPEED);
  delay(5000);
  Serial.print("Checkpoint departing right \n");
  setMotors(HIGH, TURN_SPEED, LOW, TURN_SPEED);
  delay(5000);
  Serial.print("Checkpoint lostline \n");
  setMotors(HIGH, 130, HIGH, 130);
  delay(5000);
  motorsStop();
  delay(5000);

}

void setMotors(int leftDir, int leftSpeed, int rightDir, int rightSpeed) {

    digitalWrite(M1, leftDir);
    analogWrite(E1, leftSpeed);
    digitalWrite(M2, rightDir);
    analogWrite(E2, rightSpeed);
  }

  void motorsStop() {
    Serial.print("Motor stop \n");

    analogWrite(E1, 0);
    analogWrite(E2, 0);
  }
