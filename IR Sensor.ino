const int irSensorPinLeft = 13;
const int irSensorPinRight = 12; 
unsigned long previousMillis = 0;
const unsigned long interval = 10; 

void setup() {
  pinMode(irSensorPinLeft, INPUT);
  pinMode(irSensorPinRight, INPUT);
  Serial.begin(115200);
  Serial.println("IR Sensor Initialized");
}

void loop() {
  unsigned long currentMillis = millis();
  int isLeftDetected = digitalRead(irSensorPinLeft);
  int isRightDetected = digitalRead(irSensorPinRight);
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if (isLeftDetected && isRightDetected) {
      Serial.println("Both sensors detected an object");
    } else if (isLeftDetected) {
      Serial.println("Left sensor detected an object");
    } else if (isRightDetected) {
      Serial.println("Right sensor detected an object");
    } else {
      Serial.println("No object detected");
    }
  }
}
