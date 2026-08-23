#include <QTRSensors.h>

// Create an instance of the QTRSensors class
QTRSensors qtr;

// Define the number of sensors used in the array
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);

  // Configure the object for the Analog variant (QTR-8A)
  qtr.setTypeAnalog();
  
  // Set the specific analog pins connected to sensors 1 through 8
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  
  // Set the digital pin controlling the IR emitters
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Turn on the built-in LED to signal that calibration has started
  digitalWrite(LED_BUILTIN, HIGH);

  // --- CALIBRATION PHASE ---
  // You must move the sensor array back and forth over your line 
  // (black and white surfaces) while this built-in LED is ON.
  Serial.println("Starting calibration... Move sensor over the line.");
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate(); // Takes about 10 seconds total (400 iterations * ~25ms)
  }
  
  // Turn off the built-in LED to signal calibration is complete
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Calibration complete.");
  
  // Print calibrated minimum values to the Serial Monitor for verification
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // Print calibrated maximum values to the Serial Monitor
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  delay(1000);
}

void loop() {
  // Read the calibrated line position.
  // readLineBlack() tracks a black line on a white background.
  // Use readLineWhite() if you are tracking a white line on a black background.
  // Returns a value from 0 to 7000 (3500 means the line is perfectly centered).
  uint16_t position = qtr.readLineBlack(sensorValues);

  // Print individual calibrated sensor readings (0 to 1000)
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  
  // Print the calculated center position of the line
  Serial.print("Position: ");
  Serial.println(position);

  delay(250);
}
