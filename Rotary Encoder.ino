#define CLK_Encoder1 A0
#define CLK_Encoder2 A1 

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

void setup() {
  Serial.begin(9600);

  pinMode(CLK_Encoder1, INPUT);
  pinMode(CLK_Encoder2, INPUT);

  lastStateCLK_Encoder1 = digitalRead(CLK_Encoder1);
  lastStateCLK_Encoder2 = digitalRead(CLK_Encoder2);

  Serial.println("Encoder 1 | Encoder 2 | Avg Distance (cm)");
}

void loop() {
  float avgDistance = updateDistance();

  Serial.print("Encoder 1 Distance: ");
  Serial.print(distance_Encoder1, 2);  
  Serial.print(" cm\t");

  Serial.print("Encoder 2 Distance: ");
  Serial.print(distance_Encoder2, 2);  
  Serial.print(" cm\t");

  Serial.print("Average Distance: ");
  Serial.print(avgDistance, 2);  
  Serial.println(" cm");

  delay(100);
}
