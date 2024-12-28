#include <NewPing.h>
#include <Servo.h>

#define TRIG_PIN 12
#define ECHO_PIN 13
#define SERVO_PIN 5
#define BUZZER_PIN 6

const int IN1 = 2; 
const int IN2 = 10; 
const int ENA = 3;   
const int IN3 = A2; 
const int IN4 = A3;   
const int ENB = 11; 
int motorSpeedA = 0;
int motorSpeedB = 0;

NewPing sonar(TRIG_PIN, ECHO_PIN, 50);
Servo myServo;

unsigned long previousMillis = 0;
const int scanInterval = 15;

int currentAngle = 0;
bool scanningForward = true;

bool stopSweep = false;

void setup() {
  Serial.begin(9600);
  myServo.attach(SERVO_PIN); 
  myServo.write(90); 
  delay(500);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
}

void motorTurn(int angle) {
  if (angle < 90) {
    motorSpeedA = 255; 
    motorSpeedB = 255;
    analogWrite(ENA, motorSpeedA);
    analogWrite(ENB, motorSpeedB);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    Serial.println("motor turning 1");
  } else if (angle > 90) {
    motorSpeedA = 255; 
    motorSpeedB = 255;
    analogWrite(ENA, motorSpeedA);
    analogWrite(ENB, motorSpeedB);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.println("motor turning 2");
  }
}

void stopMotor() {
  motorSpeedA = 0;
  motorSpeedB = 0;
  analogWrite(ENA, motorSpeedA);
  analogWrite(ENB, motorSpeedB);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void loop() {
  motorSpeedA = 100;
  motorSpeedB = 100;
  analogWrite(ENA, motorSpeedA);
  analogWrite(ENB, motorSpeedB);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  unsigned long currentMillis = millis();
  int distance = sonar.ping_cm();
  if (!stopSweep) {
    if (currentMillis - previousMillis >= scanInterval) {
      previousMillis = currentMillis;
      if (scanningForward) {
        currentAngle += 2;
        if (currentAngle >= 180) scanningForward = false;
      } else {
        currentAngle -= 2;
        if (currentAngle <= 0) scanningForward = true;
      }
    }
  }
  myServo.write(currentAngle);
  
  if (distance > 0 && distance <= 7) {
    stopMotor();
    Serial.println("STOP!!!");
    stopSweep = true;  
  } else if (distance > 7 && distance <= 50) {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(500);
    motorTurn(currentAngle);
    stopSweep = true;
  } else {
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(500);
    stopMotor();
    stopSweep = false;
  }
}
