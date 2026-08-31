int myVariable = 1000; // The variable you want to change

void setup() {
  Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
 // Romeo BLE chip communicates at 115200 baud
}

void loop() {
  // Check if you sent a command over Bluetooth
  if (Serial.available() > 0) {
    // Read the incoming integer value from your app
    int newValue = Serial.parseInt(); 
    
    // Update your running variable instantly
    if (newValue != 0) { 
      myVariable = newValue;
      Serial.print("Variable updated to: ");
      Serial.println(myVariable);
    }
  }
  
  delay(myVariable);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(myVariable);
  digitalWrite(LED_BUILTIN, LOW);
}
