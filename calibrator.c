// --- PIN CONFIGURATION ---
// Romeo BLE Built-in Motor Driver Pins
const int E1 = 6;     // M1 Speed Control (Left Motors)
const int M1 = 7;     // M1 Direction Control
const int E2 = 5;     // M2 Speed Control (Right Motors)
const int M2 = 4;     // M2 Direction Control

void setup() {

    Serial.begin(9600);

// Setup Pins
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);
}

void loop() {
  Serial.println("1");
  setMotors(HIGH, 0, HIGH, 200);
  delay(2000);
  Serial.println("2");
  setMotors(HIGH, 200, HIGH, 0);
  delay(2000);
  Serial.println("3");
  setMotors(HIGH, 0, LOW, 200);
  delay(2000);
}

void setMotors(int leftDir, int leftSpeed, int rightDir, int rightSpeed) {
  digitalWrite(M1, leftDir);
  analogWrite(E1, leftSpeed);
  digitalWrite(M2, rightDir);
  analogWrite(E2, rightSpeed);
}
