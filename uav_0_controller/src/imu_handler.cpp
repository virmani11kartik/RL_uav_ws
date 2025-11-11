#include "imu_handler.h"

#ifdef BUILD_TX  // Only compile for transmitter

bool IMUHandler::begin(uint8_t sdaPin, uint8_t sclPin) {
  Serial.println("\n[IMU] Initializing MPU6050...");
  
  // Initialize I2C with custom pins
  Wire.begin(sdaPin, sclPin);
  
  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("[IMU] Failed to find MPU6050 chip!");
    Serial.println("[IMU] Check wiring:");
    Serial.printf("     SDA -> GPIO%d\n", sdaPin);
    Serial.printf("     SCL -> GPIO%d\n", sclPin);
    Serial.println("     VCC -> 3.3V");
    Serial.println("     GND -> GND");
    return false;
  }
  
  Serial.println("[IMU] MPU6050 found!");
  
  // Configure MPU6050 settings for smooth control
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  Serial.println("[IMU] MPU6050 configured:");
  Serial.println("     Accelerometer: ±8G");
  Serial.println("     Gyroscope: ±500°/s");
  Serial.println("     Filter: 21 Hz");
  
  // Perform calibration
  if (!calibrate()) {
    Serial.println("[IMU] Calibration failed!");
    return false;
  }
  
  Serial.println("[IMU] Control Mapping:");
  Serial.println("     Roll:  Tilt angle left/right (like joystick position)");
  Serial.println("     Pitch: Tilt angle forward/back (like joystick position)");
  Serial.println("     Mode: ANGLE-BASED (holds position when tilted)");
  Serial.println();
  
  return true;
}

bool IMUHandler::calibrate() {
  Serial.println("\n[IMU] Calibrating...");
  Serial.println("     Keep the sensor still!");
  delay(1000);
  
  float gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
  float accelXSum = 0, accelYSum = 0, accelZSum = 0;
  const int numSamples = 100;
  
  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    gyroXSum += g.gyro.x;
    gyroYSum += g.gyro.y;
    gyroZSum += g.gyro.z;
    accelXSum += a.acceleration.x;
    accelYSum += a.acceleration.y;
    accelZSum += a.acceleration.z;
    
    delay(10);
  }
  
  gyroXOffset = gyroXSum / numSamples;
  gyroYOffset = gyroYSum / numSamples;
  gyroZOffset = gyroZSum / numSamples;
  accelXOffset = accelXSum / numSamples;
  accelYOffset = accelYSum / numSamples;
  accelZOffset = (accelZSum / numSamples) - 9.81;  // Remove gravity from Z axis
  
  Serial.println("[IMU] Calibration complete!");
  Serial.printf("     Gyro offsets: X=%.3f Y=%.3f Z=%.3f rad/s\n", 
                gyroXOffset, gyroYOffset, gyroZOffset);
  Serial.printf("     Accel offsets: X=%.2f Y=%.2f Z=%.2f m/s²\n", 
                accelXOffset, accelYOffset, accelZOffset);
  
  return true;
}

void IMUHandler::update() {
  // Read IMU data
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  
  // Apply calibration offsets
  float gyroXRaw = gyro.gyro.x - gyroXOffset;  // rad/s
  float gyroYRaw = gyro.gyro.y - gyroYOffset;
  float gyroZRaw = gyro.gyro.z - gyroZOffset;
  
  accelX = accel.acceleration.x - accelXOffset;  // m/s²
  accelY = accel.acceleration.y - accelYOffset;
  accelZ = accel.acceleration.z - accelZOffset;
  
  // Convert gyro from rad/s to degrees/s
  gyroXDeg = gyroXRaw * 57.2958;  // 180/PI
  gyroYDeg = gyroYRaw * 57.2958;
  gyroZDeg = gyroZRaw * 57.2958;
  
  // Calculate actual tilt angles from accelerometer (like a joystick position!)
  // Roll: Tilt angle left/right (positive = right tilt)
  rollAngleDeg = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 57.2958;
  
  // Pitch: Tilt angle forward/back (positive = forward tilt)
  pitchAngleDeg = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 57.2958;
  
  // Map tilt angles to normalized values (-1.0 to 1.0)
  // Now sensitivity is in degrees of tilt (e.g., 45° = full stick deflection)
  rollNormalized = rollAngleDeg / rollSensitivity;
  rollNormalized = constrain(rollNormalized, -1.0, 1.0);
  
  pitchNormalized = pitchAngleDeg / pitchSensitivity;
  pitchNormalized = constrain(pitchNormalized, -1.0, 1.0);
}

float IMUHandler::getRoll() {
  return rollNormalized;
}

float IMUHandler::getPitch() {
  return pitchNormalized;
}

void IMUHandler::getRawGyro(float &gyroX, float &gyroY, float &gyroZ) {
  gyroX = gyroXDeg;
  gyroY = gyroYDeg;
  gyroZ = gyroZDeg;
}

void IMUHandler::getAngles(float &rollAngle, float &pitchAngle) {
  rollAngle = rollAngleDeg;
  pitchAngle = pitchAngleDeg;
}

float IMUHandler::getTiltAngle() {
  // Calculate pitch angle in degrees (positive = nose up)
  return atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 57.2958;
}

#endif // BUILD_TX

