#include <SoftwareSerial.h>

#define IN1 2
#define IN2 10
#define IN3 A2
#define IN4 A3
#define ENA 3
#define ENB 11

char lastCommand = '\0'; 

SoftwareSerial Bluetooth(0, 1); // RX, TX

void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  Serial.begin(9600);

  Bluetooth.begin(9600); 
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void loop() {
  if (Bluetooth.available()) {
    char receivedChar = Bluetooth.read();
    
    Serial.print("Received: ");
    Serial.println(receivedChar);

    if (receivedChar != lastCommand) {
      lastCommand = receivedChar; 
      
      if (receivedChar == 'F') {
        Serial.println("Moving Forward");
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENA, 170);
        analogWrite(ENB, 170);
      }  
      if (receivedChar == 'B') {
        Serial.println("Moving Backward");
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENA, 170);
        analogWrite(ENB, 170);
      } 
      if (receivedChar == 'L') {
        Serial.println("Turning Left");
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENA, 120);
        analogWrite(ENB, 120);
      } 
      if (receivedChar == 'R') {
        Serial.println("Turning Right");
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENA, 120);
        analogWrite(ENB, 120);
      }  
      if (receivedChar == 'S') { 
        stopMotors();
      }
    }
  }
}
