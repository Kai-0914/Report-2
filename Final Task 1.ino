#include <LiquidCrystal.h>
#include <Wire.h>

#define CLK_Encoder1 A0
#define CLK_Encoder2 A1

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

const int IN1 = 2; 
const int IN2 = 10; 
const int ENA = 3;   
const int IN3 = A2; 
const int IN4 = A3;   
const int ENB = 11;  
const int irLeft = 13;
const int irRight = 12;

const int MPU_ADDR = 0x68;

int16_t accelX, accelY, accelZ;
float accelX_scaled, accelY_scaled, accelZ_scaled;
float smoothAccelX = 0.0, smoothAccelY = 0.0, smoothAccelZ = 0.0;
float alpha = 0.9;
float GyroZ, GyroZ_scaled;
float spinAngle = 0.0;
float GyroErrorZ = 0.0;

unsigned long previousMillis = 0;
const unsigned long interval = 10;
int motorSpeed = 0;
volatile int counter_Encoder1 = 0; 
volatile int counter_Encoder2 = 0; 
int lastStateCLK_Encoder1;
int lastStateCLK_Encoder2;
unsigned long lastTime = 0;
volatile int lastCounter_Encoder1 = 0;
volatile int lastCounter_Encoder2 = 0;
float pulsesPerRevolution = 20.0;
float wheelCircumference = 20.15;
float distance_Encoder1 = 0.0;     
float distance_Encoder2 = 0.0; 
float timeInterval = 100.0; 
float avgDistance; 

unsigned long lastRampTime = 0;
const unsigned long rampInterval = 100;  
int motorSpeedA = 0;
int motorSpeedB = 0;

unsigned long stopTime = 0;
bool carStop = false;
bool carRamp = false;
bool carStop2 = false;
bool carStop3 = false;

void setup() {
  lcd.begin(16, 2);
  lcd.print("3");
  delay(500);
  lcd.clear();
  lcd.print("2");
  delay(200);
  lcd.clear();
  lcd.print("1");
  delay(300);
  lcd.clear();
  pinMode(CLK_Encoder1, INPUT);
  pinMode(CLK_Encoder2, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);

  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x10);
  Wire.endTransmission(true);

  Serial.begin(9600);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
  lastStateCLK_Encoder1 = digitalRead(CLK_Encoder1);
  lastStateCLK_Encoder2 = digitalRead(CLK_Encoder2);
}

float readGyroZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  return (Wire.read() << 8 | Wire.read()) / 131.0;
}

void calculateGyroError() {
  long sum = 0;
  const int samples = 200;
  for (int i = 0; i < samples; i++) {
    sum += (int16_t)readGyroZ() * 131;
    delay(3);
  }
  GyroErrorZ = sum / (float)samples / 131.0;
}

void readMPU6050() {
  static unsigned long lastMPUUpdate = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - lastMPUUpdate >= 500) {
    lastMPUUpdate = currentMillis;
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);

    if (Wire.available() == 6) {
      accelX = Wire.read() << 8 | Wire.read();
      accelY = Wire.read() << 8 | Wire.read();
      accelZ = Wire.read() << 8 | Wire.read();
      accelX_scaled = accelX / 16384.0;
      accelY_scaled = accelY / 16384.0;
      accelZ_scaled = accelZ / 16384.0;

      smoothAccelX = alpha * smoothAccelX + (1 - alpha) * accelX_scaled;
      smoothAccelY = alpha * smoothAccelY + (1 - alpha) * accelY_scaled;
      smoothAccelZ = alpha * smoothAccelZ + (1 - alpha) * accelZ_scaled;
    }
  }
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

void fullSpeed(){
  motorSpeedA = 255;
  motorSpeedB = 255;
  analogWrite(ENA, motorSpeedA);
  analogWrite(ENB, motorSpeedB);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

float updateDistance() {
  unsigned long currentTime = millis();
  int currentStateCLK_Encoder1 = digitalRead(CLK_Encoder1);
  int currentStateCLK_Encoder2 = digitalRead(CLK_Encoder2);

  if (currentStateCLK_Encoder1 != lastStateCLK_Encoder1 && currentStateCLK_Encoder1 == HIGH) {
    counter_Encoder1++;
  }
  lastStateCLK_Encoder1 = currentStateCLK_Encoder1;

  if (currentStateCLK_Encoder2 != lastStateCLK_Encoder2 && currentStateCLK_Encoder2 == HIGH) {
    counter_Encoder2++;
  }
  lastStateCLK_Encoder2 = currentStateCLK_Encoder2;

  if (currentTime - lastTime >= timeInterval) {
    lastTime = currentTime;
    int countDifference_Encoder1 = counter_Encoder1 - lastCounter_Encoder1;
    int countDifference_Encoder2 = counter_Encoder2 - lastCounter_Encoder2;
    distance_Encoder1 += ((countDifference_Encoder1 / pulsesPerRevolution) * wheelCircumference) / 100;
    distance_Encoder2 += ((countDifference_Encoder2 / pulsesPerRevolution) * wheelCircumference) / 100;
    avgDistance = (distance_Encoder1 + distance_Encoder2) / 2.0;
    lastCounter_Encoder1 = counter_Encoder1;
    lastCounter_Encoder2 = counter_Encoder2;
  }
  return avgDistance;
}

void loop() {
  unsigned long currentMillis = millis();
  static unsigned long lastPrint = 0;
  static int isLeftDetected;
  static int isRightDetected;
  const unsigned long IRresponse = 20;

  if (currentMillis - lastPrint >= IRresponse) {
    lastPrint = currentMillis;
    readMPU6050();
    isLeftDetected = digitalRead(irLeft);
    isRightDetected = digitalRead(irRight);
  }
  float tiltAngle = (atan2(accelY_scaled, accelZ_scaled) * 180 / 3.142) * -1;
  float tiltUpThreshold = 25.0;
  float tiltDownThreshold = -15.0;
  float GyroZ_noScaled = readGyroZ();
  calculateGyroError();
  GyroZ_scaled = GyroZ_noScaled - GyroErrorZ;
  unsigned long spinTime = millis();
  static unsigned long lastUpdate = 0;
  float sec = (spinTime - lastUpdate) / 300.0;
  lastUpdate = spinTime;
  spinAngle = spinAngle + (GyroZ_scaled * sec);

  unsigned long LCDmillis = millis();
  static unsigned long lastLCDupdate = 0;
  const unsigned long LCDupdateInterval = 500;
  if (LCDmillis - lastLCDupdate >= LCDupdateInterval) {
    lastLCDupdate = LCDmillis;
    lcd.setCursor(0, 0);
    lcd.clear();
    lcd.print("Angle: ");
    lcd.print(tiltAngle, 1);
  }

  if (!carRamp) {
    motorSpeedA = 100;
    motorSpeedB = 100;
    analogWrite(ENA, motorSpeedA);
    analogWrite(ENB, motorSpeedB);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    if (tiltAngle > tiltUpThreshold) {
        fullSpeed();
        carStop = true;
    } else if (tiltAngle < tiltUpThreshold - 15 && carStop) {
        stopMotor();
        delay(2000);
        spinAngle = 0.0;
        while (spinAngle <= 360) {
          motorSpeedA = 150;
          motorSpeedB = 150;
          analogWrite(ENA, motorSpeedA);
          analogWrite(ENB, motorSpeedB);
          digitalWrite(IN1, LOW);
          digitalWrite(IN2, HIGH);
          digitalWrite(IN3, HIGH);
          digitalWrite(IN4, LOW);
        }
        carRamp = true;
    }
  }

  if (carRamp && !carStop2) {
      motorSpeedA = 80;
      motorSpeedB = 80;
      analogWrite(ENA, motorSpeedA);
      analogWrite(ENB, motorSpeedB);
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    }
    if (tiltAngle < tiltDownThreshold && !carStop2) {
      stopMotor();
      delay(2000);
      carStop2 = true;
    }

  if (carRamp && carStop2) {
    updateDistance();
    Serial.println(avgDistance);
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

    if (avgDistance >= 3.30 && !carStop3) {
      stopMotor();
      delay(2000);
      carStop3 = true;
    }
    
    if (avgDistance >= 3.30 && carStop3) {
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
  }
}
