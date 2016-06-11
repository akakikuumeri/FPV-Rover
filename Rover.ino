//Four motors on the rover are arranged as follows:
//1, 2
//4, 3
#include <AFMotor.h>
#include <Servo.h> 

#define MINTHR 20
#define PANMIDDLE 120

AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

Servo servo1;

const int throttlepin = 14;
const int steerpin = 15;
const int panpin = 16; //camera pan tilt feature is optional

int throttle = 0;
int steer = 0;
int pan = 1500;
int pan_smooth = 1500;

int signal = 0;

#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define OUTPUT_BINARY_ACCELGYRO

int pidsteer = 0;
int steeri = 0;
int zaverage = 0;


void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor test!");

  pinMode(throttlepin, INPUT);
  pinMode(steerpin, INPUT);
  pinMode(panpin, INPUT);
  
  servo1.attach(9);
  
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

 

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}


void loop() {
  //Read RC receiver
  noInterrupts();
  signal = pulseIn(steerpin, HIGH, 100000);
  if (signal != 0) {
    steer = map(signal, 1030, 1930, -255, 255);
  }
  signal = pulseIn(throttlepin, HIGH, 100000);
  if (signal != 0) {
    throttle = map(signal, 1030, 1930, -255, 255);
  }
  pan = pulseIn(panpin, HIGH, 50000);
  interrupts();
  
  
  //Serial.print(map(pan_smooth, 1070, 1940, 0,175));
  //Expand and smooth commands for camera pan to achieve greater range and smoother motion from the servo
  if (pan != 0) {
    pan_smooth = (pan_smooth*5 + pan)/6;
    if (abs(pan - pan_smooth) >15) {//only move if signal is reasonably different from recent average,
      //so dont twitch when still)
      
      //scale the output around the middle, this is like subtrim
      if(pan_smooth < 1500) {
          servo1.write(map(pan_smooth, 1100, 1500, 0,PANMIDDLE));
      } else {
        servo1.write(map(pan_smooth, 1500, 1900, PANMIDDLE,175));
      }
    } 
  }
  
  //For debugging PIDs
  Serial.print("\t throttle: ");
  Serial.print (throttle);     //Print in the value of channel 1
  Serial.print ("\t steer: ");
  Serial.println(steer);
  
  //if (throttle < -MINTHR) {steer = -steer;}//if in reverse, steering should be backwards if you want car-like controls instead of tank-like
  
  //Read gyro
  accelgyro.getRotation(&gx, &gy, &gz);
  Serial.println("Z gyro and then steering");
  Serial.println(gz);
  
  zaverage = gz*0.7 + zaverage *0.3;//step window average
  
  //A super simple PID controller follows
  int steerp = zaverage/200 + steer*0.5;
  
  steeri += steerp; //integrating
  steeri *= 0.8; //fade back
  
  pidsteer = steerp*1.2 + steeri*0.8; //And here we have the PID sum
  Serial.println(pidsteer);
  
  //If steer and throttle are neutral, relax the motors
  if (abs(throttle) < MINTHR && abs(steer) < MINTHR) {
    releaseall();
  } else {
  int leftmotor = throttle + pidsteer;
  int rightmotor = throttle - pidsteer;
  
  if (leftmotor > 300) {// if one of the motors goes beyong throttle range, help it out by lowering the other a bit
    rightmotor-=(leftmotor-300)/2;
  }
  if (rightmotor > 300) {
    leftmotor-=(rightmotor-300)/2;
  }
  if (leftmotor < -300) {// if one of the motors goes beyong throttle range, help it out by lowering the other a bit
    rightmotor+=(leftmotor+300)/2;
  }
  if (rightmotor < -300) {
    leftmotor+=(rightmotor+300)/2;
  }
  
  //steer = steer * (((float)abs((float)throttle)/500.0)+0.6);//exaggerate steer near high throttle, make it smaller near small throtle
  leftmotors(constrain(leftmotor, -255, 255));
  rightmotors(constrain(rightmotor, -255, 255));
  }

}

void leftmotors(int spd) {
  if (spd >= 0) {
    motor1.run(FORWARD);
    motor4.run(FORWARD);
    motor1.setSpeed(spd);
    motor4.setSpeed(spd);
  } else {
    motor1.run(BACKWARD);
    motor4.run(BACKWARD);
    motor1.setSpeed(-spd);
    motor4.setSpeed(-spd);
  }
}
void rightmotors(int spd) {
  if (spd >= 0) {
    motor2.run(FORWARD);
    motor3.run(FORWARD);
    motor2.setSpeed(spd);
    motor3.setSpeed(spd);
  } else {
    motor2.run(BACKWARD);
    motor3.run(BACKWARD);
    motor2.setSpeed(-spd);
    motor3.setSpeed(-spd);
  }
}
void releaseall() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  motor4.setSpeed(0);
}
