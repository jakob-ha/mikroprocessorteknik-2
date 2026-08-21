// Pin definitions
const int triggerPin = 9; // Connected to URM37 COMP/TRIG
const int echoPin = 12;    // Connected to URM37 ECHO

void setup() {
  // Initialize Serial Monitor at 115200 baud
  Serial.begin(115200); 
  
  // Set pin modes
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // Ensure trigger pin starts HIGH (URM37 triggers on a LOW pulse)
  digitalWrite(triggerPin, HIGH);
  
  Serial.println("URM37 Ultrasonic Sensor Initialized.");
}

void loop() {
  // Generate a low pulse to trigger the URM37 sensor
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(10); 
  digitalWrite(triggerPin, HIGH);
  
  // Measure the duration of the low pulse returned by the ECHO pin
  // URM37 outputs a low pulse proportional to distance: 50us per centimeter
  unsigned long duration = pulseIn(echoPin, LOW, 30000); // 30ms timeout
  
  if (duration == 0) {
    Serial.println("Error: No pulse detected / Out of range");
  } else {
    // Calculate distance in centimeters
    unsigned long distance = duration / 50; 
    
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }
  
  // Wait 100ms before taking the next reading
  delay(100);
}
