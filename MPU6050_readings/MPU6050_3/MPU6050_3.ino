// #include<Wire.h>
 
// const int MPU_addr=0x68;
// int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
 
// int minVal=265;
// int maxVal=402;
 
// double x;
// double y;
// double z;
 
// void setup(){
// Wire.begin();
// Wire.beginTransmission(MPU_addr);
// Wire.write(0x6B);
// Wire.write(0);
// Wire.endTransmission(true);
// Serial.begin(9600);
// Serial.print("send any charecter to being the program");
// while(!Serial.available()){}
// Serial.parseInt();


// }
// void loop(){
// Wire.beginTransmission(MPU_addr);
// Wire.write(0x3B);
// Wire.endTransmission(false);
// Wire.requestFrom(MPU_addr,14,true);
// AcX=Wire.read()<<8|Wire.read();
// AcY=Wire.read()<<8|Wire.read();
// AcZ=Wire.read()<<8|Wire.read();
// int xAng = map(AcX,minVal,maxVal,-90,90);
// int yAng = map(AcY,minVal,maxVal,-90,90);
// int zAng = map(AcZ,minVal,maxVal,-90,90);
 
// x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
// y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
// z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
// Serial.print(x);
// Serial.print("\t\t");

// Serial.print(y);
// Serial.print("\t\t");

// Serial.println(z);

// delay(10000);

// }

//chatGPT code

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Complementary filter parameters
float alpha = 0.98; // Filter constant (adjust as needed)

// Variables for angle calculation
float lastTime = 0;
float lastPitch = 0;
float lastRoll = 0;

void setup() {
  Wire.begin();
  mpu.initialize();
  Serial.begin(9600);
}

void loop() {
  while(Serial.available()){
    return;
  }
  // Read accelerometer and gyroscope data
  //mpu.getMotion6();
  
  // Calculate elapsed time since the last reading
  float currentTime = millis() / 1000.0;
  float elapsedTime = currentTime - lastTime;
  lastTime = currentTime;

  // Extract accelerometer data
  float accelX = mpu.getAccelerationX();
  float accelY = mpu.getAccelerationY();
  float accelZ = mpu.getAccelerationZ();
  
  // Calculate roll and pitch angles using accelerometer data
  float roll = atan2(accelY, accelZ);
  float pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ));

  // Calculate gyro rates
  float gyroX = mpu.getRotationX() / 131.0; // Sensitivity for MPU6050 at +/- 250 degrees/sec
  float gyroY = mpu.getRotationY() / 131.0;

  // Integrate gyro rates to get roll and pitch angles
  float rollRate = gyroX * elapsedTime;
  float pitchRate = gyroY * elapsedTime;

  // Apply complementary filter
  roll = alpha * (roll + rollRate) + (1 - alpha) * roll;
  pitch = alpha * (pitch + pitchRate) + (1 - alpha) * pitch;

  // Convert angles to degrees
  float rollDeg = degrees(roll);
  float pitchDeg = degrees(pitch);

  Serial.print("Roll: ");
  Serial.print(rollDeg);
  Serial.print(" degrees, Pitch: ");
  Serial.print(pitchDeg);
  Serial.println(" degrees");
  delay(100);
}

