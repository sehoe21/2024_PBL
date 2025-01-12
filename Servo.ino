/*
서보모터 구동을 위한 c++ 코드
PID/PD Compensator를 추가할 경우 더욱 정밀한 제어가 가능할 것으로 추정
*/

#include "ESP32Servo.h"
#define SERVO_PIN 18

Servo myservo;

const float targetSpeed = 30.0; 
const int pulseWidthCenter = 1500;
const int pulseWidthStep = 10;

void setup()
{
  myservo.setPeriodHertz(50); //주파수 50Hz => 주기 T = 20ms
  myservo.attach(SERVO_PIN, 1000, 2000); //PWM의 최소폭 1ms ~ 최대폭 2ms => 회전속도 80~90RPM = 480~540 degree/sec
}

void loop() 
{

  myservo.writeMicroseconds(2000);
  delay(10);
}

