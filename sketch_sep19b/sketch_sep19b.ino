#include <Servo.h>
Servo motor1;
Servo motor2;
int motor_speed = 1150;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(11500);
  motor1.attach(4);
  motor2.attach(5);
  motor1.writeMicroseconds(1050);
  motor2.writeMicroseconds(1050);
  delay(3000);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()){
    int speed = Serial.parseInt();
    if(speed>0){
      motor_speed = speed;
    }
  }
  motor1.writeMicroseconds(motor_speed);
  motor2.writeMicroseconds(motor_speed);
}
