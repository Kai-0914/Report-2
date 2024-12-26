const int IN1 = 2; 
const int IN2 = 10; 
const int ENA = 3;   
const int IN3 = A2; 
const int IN4 = A3;   
const int ENB = 11;

int motorSpeed = 0;
int motorSpeedA = 0;
int motorSpeedB = 0;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
}

void loop() {
  motorSpeedA = 255;
  motorSpeedB = 255;
  analogWrite(ENA, motorSpeedA);
  analogWrite(ENB, motorSpeedB);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
