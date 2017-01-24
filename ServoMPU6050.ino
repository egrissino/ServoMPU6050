#include <Servo.h>
#include <PID.h>
#include <Wire.h>
#include <I2Cdev.h>

#define NOT_AN_INTERRUPT -1

#include "MPU_6050.h"

#define SERVO_PIN 9
#define INTERRUPT_PIN 2

float angle;
bool DMPready;

MPU_6050 mpu(0x68);     // Defult adress for MPU6050
PID pid(0 , 0);         // current position and set point
Servo servo;


void setup() {
  pinMode(A0, OUTPUT);
  digitalWrite(A0, HIGH);
  
  pid.set_P(1);
  pid.set_I(0.2);
  pid.set_D(3);
  
  pid.set_gain(0.1);
  pid.begin(millis());
  
  Serial.begin(9600);
  servo.attach(SERVO_PIN);
  
  Wire.begin();
  mpu.initialize(true, false);
  
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  DMPready = true;
  angle = 70; 
};

void loop() {
  
  mpu.processData();
  
  angle -= pid.update(mpu.Angle.y, millis());
  
  if (angle < 140 || angle > 10) {
    servo.write(angle);
    Serial.println(angle);
  } else {
    if (angle >= 140) {
      angle = 140;
  } else {
      angle = 10;
    }
  }
  
  delay(10);
  
  
};

void dmpDataReady() {
  interrupts();
  mpu.getRawData();
  mpu.resetFIFO();
  noInterrupts();
};
