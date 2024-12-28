#include <NewPing.h>

// Pin Definitions
#define TRIG_PIN 12
#define ECHO_PIN 13
#define BUZZER 6
#define BLUE_LED 9  // Blue LED pin
#define RED_LED 7   // Red LED pin
#define GREEN_LED 8// Green LED pin 

#define ENA 3
#define ENB 11
#define IN1 2
#define IN2 10
#define IN3 A2
#define IN4 A3

// Constants
const unsigned long DISTANCE_CHECK_INTERVAL = 50;
const int OBSTACLE_DISTANCE_RED = 10;  // Red LED threshold
const int OBSTACLE_DISTANCE_BLUE = 30; // Blue LED threshold

// Buzzer Timings
const unsigned long BUZZER_RED_CYCLE = 250;  // 4 times per second
const unsigned long BUZZER_BLUE_CYCLE = 500; // 2 times per second

// Global Variables
unsigned long previousDistanceCheck = 0;
unsigned long previousBuzzerTime = 0;
bool buzzerActive = false;
bool ledActive = false;
int distance = 0;
char currentCommand = 'S';

NewPing sonar(TRIG_PIN, ECHO_PIN, OBSTACLE_DISTANCE_BLUE);

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("Motors stopped");
}

void moveForward(int speed = 100) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Moving forward");
}

void moveBackward(int speed = 100) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Moving backward");
}

void turnRight(int speed = 255) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Turning left");
}

void turnLeft(int speed = 255) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Turning right");
}

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - previousDistanceCheck >= DISTANCE_CHECK_INTERVAL) {
    previousDistanceCheck = currentTime;
    distance = sonar.ping_cm();
    handleObstacle();
  }

  if (Serial.available()) {
    char newCommand = Serial.read();
    if (isValidCommand(newCommand)) {
      currentCommand = newCommand;
      processCommand();
    }
  }

  handleIndicators(currentTime);
}

void handleObstacle() {
  if (distance > 0 && distance <= OBSTACLE_DISTANCE_RED) {
    if (currentCommand == 'F') {
      stopMotors();
      currentCommand = 'S';
      Serial.println("Obstacle detected, stopping motors.");
    }
    activateRedMode();
  } else if (distance > OBSTACLE_DISTANCE_RED && distance <= OBSTACLE_DISTANCE_BLUE) {
    activateBlueMode();
  } else {
    activateGreenMode(); // No obstacle within 30 cm
  }
}

void processCommand() {
  // Prevent forward movement in RED LED mode
  if (distance > 0 && distance <= OBSTACLE_DISTANCE_RED && currentCommand == 'F') {
    stopMotors();
    Serial.println("Forward movement blocked.");
    return;
  }

  switch (currentCommand) {
    case 'F':
      moveForward();
      break;
    case 'B':
      moveBackward();
      break;
    case 'L':
      turnLeft();
      break;
    case 'R':
      turnRight();
      break;
    case 'S':
    default:
      stopMotors();
      break;
  }
}

void handleIndicators(unsigned long currentTime) {
  if (buzzerActive && ledActive) {
    unsigned long buzzerCycle = (distance <= OBSTACLE_DISTANCE_RED) ? BUZZER_RED_CYCLE : BUZZER_BLUE_CYCLE;

    if (currentTime - previousBuzzerTime >= buzzerCycle) {
      digitalWrite(BUZZER, !digitalRead(BUZZER));
      previousBuzzerTime = currentTime;
    }
  } else {
    digitalWrite(BUZZER, LOW);
  }
}

void activateBlueMode() {
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, HIGH);
  buzzerActive = true;
  ledActive = true;
}

void activateRedMode() {
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, HIGH);
  buzzerActive = true;
  ledActive = true;
}

void activateGreenMode() {
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  buzzerActive = false;
  ledActive = false;
}

bool isValidCommand(char command) {
  return command == 'F' || command == 'B' || command == 'L' || command == 'R' || command == 'S';
}
