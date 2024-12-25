#include <Wire.h>

// MPU-6050 I2C address (AD0 pin low)
const int MPU_ADDR = 0x68;

void setup() {
  Wire.begin(); // Initialize I2C communication
  Serial.begin(9600); // Start serial communication at 9600 baud
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // Set to zero to wake up the MPU-6050
  Wire.endTransmission(true);
}

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // Request 14 bytes from MPU-6050

  // Read accelerometer and gyroscope data
  // Each value is a 16-bit signed integer (high byte first)
  int16_t accX = Wire.read() << 8 | Wire.read();
  int16_t accY = Wire.read() << 8 | Wire.read();
  int16_t accZ = Wire.read() << 8 | Wire.read();

  int16_t gyroX = Wire.read() << 8 | Wire.read();
  int16_t gyroY = Wire.read() << 8 | Wire.read();
  int16_t gyroZ = Wire.read() << 8 | Wire.read();

  // Print the sensor values
  Serial.print("Accelerometer: ");
  Serial.print("X = "); Serial.print(accX);
  Serial.print(" | Y = "); Serial.print(accY);
  Serial.print(" | Z = "); Serial.println(accZ);

  Serial.print("Gyroscope: ");
  Serial.print("X = "); Serial.print(gyroX);
  Serial.print(" | Y = "); Serial.print(gyroY);
  Serial.print(" | Z = "); Serial.println(gyroZ);

  delay(1000); // Delay a second for next reading
}
