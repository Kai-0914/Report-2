const int IN1 = 2; 
const int IN2 = 10; 
const int ENA = 3;   
const int IN3 = A2; 
const int IN4 = A3;   
const int ENB = 11;  
const int irLeft = 13;
const int irRight = 12;

int motorSpeedA = 0;
int motorSpeedB = 0;
int motorSpeed = 0;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);
  Serial.begin(9600);

  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void stopMotor(){
  motorSpeedA = 0;
  motorSpeedB = 0;
  analogWrite(ENA, motorSpeedA);
  analogWrite(ENB, motorSpeedB);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void leftMotorFoward(){
  motorSpeedA = 90; 
  analogWrite(ENA, motorSpeedA);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void leftMotorBackward(){
  motorSpeedA = 255; 
  analogWrite(ENA, motorSpeedA);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

void rightMotorFoward(){
  motorSpeedB = 90;
  analogWrite(ENB, motorSpeedB);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void rightMotorBackward(){
  motorSpeedB = 255; 
  analogWrite(ENB, motorSpeedB);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void loop(){
  static int isLeftDetected;
  static int isRightDetected;
  const unsigned long IRresponse = 20;
  static unsigned long lastPrint = 0;

  if (millis() - lastPrint >= IRresponse) {
  lastPrint = millis();
  isLeftDetected = digitalRead(irLeft);
  isRightDetected = digitalRead(irRight);
  }

  if (!isLeftDetected && !isRightDetected) {
      stopMotor();
      return;
    }

  if (isLeftDetected) {
    leftMotorFoward();
  } else {
    leftMotorBackward();
  }

  if (isRightDetected) {
    rightMotorFoward();
  } else {
    rightMotorBackward();
  }
}
