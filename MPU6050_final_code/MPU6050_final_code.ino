//setting up MPU6050
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define INTERRUPT_PIN 2
#define LED_PIN 13

//variables required for MPU6050
bool dmpReady = false;
Quaternion q; 
VectorFloat gravity;
bool blinkState = false;
uint8_t mpuIntStatus;  
uint8_t devStatus;
uint16_t packetSize; 
uint16_t fifoCount;    
volatile bool mpuInterrupt = false;
uint8_t fifoBuffer[64];
float ypr[3]; 
int count = 0;


void dmpDataReady() {
    mpuInterrupt = true;
}

void setup_MPU6050(){
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); 
  while (!Serial.available());               
  while (Serial.available() && Serial.read()); 
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  if (devStatus == 0) {
        mpu.CalibrateAccel(10);
        mpu.CalibrateGyro(10);
        mpu.PrintActiveOffsets();

        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

//getting angles and displaying them
void get_angles(){
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
}
void display_angles(){
  Serial.print("ypr\t");
  Serial.print(ypr[0] * 180/M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180/M_PI);
  Serial.print("\t");
  Serial.println(ypr[2] * 180/M_PI);
}

//setting up motors
#include <Servo.h>
Servo left_motor,right_motor;
int motor_speed = 1200;
int left_motor_pin = 6;
int right_motor_pin = 5;
void setup_motors(){
  left_motor.attach(left_motor_pin);
  right_motor.attach(right_motor_pin);
  Serial.println("attach the battery and then send any integer");
  while(!Serial.available()){
    left_motor.writeMicroseconds(2000);
    right_motor.writeMicroseconds(2000);
  }
  int x = Serial.parseInt();
  Serial.println("send any integer if you have heard 3 beeps");
  while(!Serial.available()){
    left_motor.writeMicroseconds(1000);
    right_motor.writeMicroseconds(1000);
  }
  x = Serial.parseInt();
  Serial.println("starting the loop in 4 seconds");
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  setup_MPU6050();
  //setup_motors();
  delay(4000);
}

//terms needed for balancing 
double previous_time=0,present_time=0;
double set_angle=0, 
       present_angle=0, 
       previous_angle=0, 
       min_pid_i_angle=6;
double kp = 3,
       ki = 1,
       kd = 0.5;
double pid_p,pid_i,pid_d;
double PID;
double set_motor_speed;
int min_motor_speed = 1050;
int max_motor_speed = 1850;
double error, previous_error;


void balance(){
  present_angle = ypr[1];
  present_time = millis();
  error = present_angle- set_angle;
  double difference = present_angle-previous_angle;
  double time_elapsed = present_time-previous_time;
  

  

  pid_p = kp*(error);
 
  if(difference<min_pid_i_angle && difference>-1*(min_pid_i_angle)){
    pid_i += ki*(error);
  }
  if((error*previous_error)<0){
    pid_i = 0;
  }

  pid_d = kd*(error-previous_error)/time_elapsed;
 
  PID = pid_p+pid_i+pid_d;
 //don't forget to check this +,- sign before starting the code by checking the angles using the IMU
 //if the angle in one direction are "+" then the pid sign should be "-" in that direction
 int right_motor_speed = motor_speed;
 int left_motor_speed = motor_speed;
  if(PID<0){
    right_motor_speed = motor_speed-(int)PID;
  }else{
    left_motor_speed = motor_speed+(int)PID;
  }
  
  
  
  if(left_motor_speed<min_motor_speed){
    left_motor_speed = min_motor_speed;
  }
  if(right_motor_speed<min_motor_speed){
    right_motor_speed =  min_motor_speed;
  }
  if(left_motor_speed > max_motor_speed){
    left_motor_speed = max_motor_speed;
  }
  if(right_motor_speed > max_motor_speed){
    right_motor_speed = max_motor_speed;          
  }
 //you should also check if the motors will go with the same speed if same microseconds are given
 //other wise change it accordingly
  left_motor.writeMicroseconds(left_motor_speed);
  right_motor.writeMicroseconds(right_motor_speed);
  count++;
  if(count%1000 == 0){
    Serial.print("1000 readings took a time of ");
    Serial.println(present_time);
    delay(5000);
  }

  previous_error = error;
  previous_angle = present_angle;
  previous_time = present_time;

}
void control_motor_speed(int &motor_speed){
  int speed = Serial.parseInt();
  if(speed>0){
    motor_speed = speed;
  }
}

void loop() {
  if (!dmpReady) return;
  if(Serial.available()){
    control_motor_speed(motor_speed);
  }
  get_angles();
  display_angles();
  balance();
  
}