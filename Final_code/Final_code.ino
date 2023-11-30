//setting up MPU9250
#include "MPU9250.h"
MPU9250 mpu;
//x,y,z are the angles that will be stored
float x,y,z; 

void setup_MPU9250(){
    Wire.begin();
    delay(2000);

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
}

//below are the errors with respect to the initial position
float x_error = 0;
float y_error = 0;
float z_error = 0;

void get_angles(){
  float x_value=0 , y_value=0, z_value=0;
  for(int i=0;i<10;i++){
    if (mpu.update()){
      x_value += mpu.getYaw();
      x_value-=x_error;
      y_value += mpu.getPitch();
      y_value-=y_error;
      z_value += mpu.getRoll();
      z_value -=z_error;
    }
    x = x_value/10;
    y=y_value/10;
    z=z_value/10;
  }
}
void display_angles(){
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(x, 2);
    Serial.print(", ");
    Serial.print(y, 2);
    Serial.print(", ");
    Serial.println(z, 2);
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(x_error, 2);
    Serial.print(", ");
    Serial.print(y_error, 2);
    Serial.print(", ");
    Serial.println(z_error, 2);
}

void calibrate_MPU9250(){
  float error1 =0,error2=0,error3=0;
  for(int i=0;i<300;i++){//just think of changing the value here and for the next loop, by seeing the values of the avg in the get_angle function
    get_angles();
    display_angles();
  }
  for(int i = 0;i<80;i++){
    get_angles();
    display_angles();
    error1 += x;
    error2 += y;
    error3 += z;
  }
  x_error = error1/80;
  y_error = error2/80;
  z_error = error3/80;
}
//setting up motors
#include <Servo.h>
Servo left_motor,right_motor;
int motor_speed = 1200;
int right_motor_speed = motor_speed;
int left_motor_speed = motor_speed;
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
  int x1 = Serial.parseInt();
  Serial.println("send any integer if you have heard 3 beeps");
  while(!Serial.available()){
    left_motor.writeMicroseconds(1000);
    right_motor.writeMicroseconds(1000);
  }
  x1 = Serial.parseInt();
  Serial.println("starting the loop in 4 seconds");
  delay(4000);
}

void setup(){
  Serial.begin(115200);
  setup_MPU9250();
  setup_motors();
  Serial.println("there will be values printing to calibrate, just don't worry about them");
  delay(2000);
  calibrate_MPU9250();
  Serial.println("done calibrating");
  while(!Serial.available());
  Serial.parseInt();
}

//terms needed for balancing 
double previous_time=0,present_time=0;
double set_angle=0, 
       present_angle=0, 
       previous_angle=0, 
       min_pid_i_angle=3;
double kp = 20,
       ki = 0.5,
       kd = 20;//keep this in hold untill you know the values of rate term
double pid_p,pid_i,pid_d;
double PID;
double set_motor_speed;
int min_motor_speed = 1050;
int max_motor_speed = 1850;
double error, previous_error;


void balance(){

  previous_error = error;
  previous_angle = present_angle;
  previous_time = present_time;
  
  present_angle = y;
  present_time = millis();
  error = present_angle- set_angle;
  double difference = present_angle-previous_angle;
  double time_elapsed = present_time-previous_time;
  

  

  pid_p = kp*(error);
 
  if(difference<min_pid_i_angle && difference>-1*(min_pid_i_angle)){
    pid_i += ki*(error);
  }
  if((error*previous_error<0) || (error>-0.3 & error<0.3)){
    pid_i = 0;
  }

  pid_d = kd*(error-previous_error)/time_elapsed;
 
  PID = pid_p+pid_d;
  //don't forget to check this +,- sign before starting the code by checking the angles using the IMU
  //if the angle in one direction are "+" then the pid sign should be "-" in that direction
  right_motor_speed = motor_speed;
  left_motor_speed = motor_speed;
  if(PID>8*motor_speed/100){
    PID = 8*motor_speed/100;
  }else if(PID<-8*motor_speed/100){
    PID = -8*motor_speed/100;
  }
  if(PID<0){
    right_motor_speed = motor_speed-(int)PID;
  }else{
    left_motor_speed = motor_speed+(int)PID;
  }
 
 

 //you should also check if the motors will go with the same speed if same microseconds are given
 //other wise change it accordingly
}
void run_motor(){
  left_motor.writeMicroseconds(left_motor_speed);
  right_motor.writeMicroseconds(right_motor_speed);
}

void control_motor_speed(int &motor_speed){
  int speed = Serial.parseInt();
  if(speed>999){
    motor_speed = speed;
  }
}
int count = 0;
void loop(){
  get_angles();
  display_angles();
  balance();
  if(Serial.available()){
    control_motor_speed(motor_speed);
  }
  run_motor();

}