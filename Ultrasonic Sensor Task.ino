const int trigPin = 12;
const int echoPin = 13;
const int IN1 = 2; 
const int IN2 = 10; 
const int ENA = 3;   
const int IN3 = A2; 
const int IN4 = A3;   
const int ENB = 11; 

float duration, distance;
const float distanceThreshold = 25.0; // Distance threshold in centimeters

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  Serial.begin(115200);

  analogWrite(ENA, 150); 
  analogWrite(ENB, 150); 
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration * 0.0343) / 2;

  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance > 0 && distance < distanceThreshold) {
    stopMotor();
    Serial.println("Object detected! Stopping.");
    delay(2000);
  } else {
    moveForward();
  }

  delay(100); // Small delay to avoid rapid updating
}

void stopMotor() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void moveForward() {
  analogWrite(ENA, 150); 
  analogWrite(ENB, 150); 
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
