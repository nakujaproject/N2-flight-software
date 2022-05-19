#ifndef REACTIONwHEEL_H
#define REACTIONwHEEL_H
#include <Arduino.h>
#include "PID_v1.h"

#include <ESP32Servo.h>
#include "MPU6050_6Axis_MotionApps20.h"

#define sampleTime 10

void RunReactionWheel(MPU6050 mpu, bool dmpReady);
void Write_pwm(float);
double Constrainpwm (double, double, double);

#define OUTPUT_READABLE_YAWPITCHROLL

#define SDA 21
#define SCL 22

struct reactionWheelParams{
    MPU6050 *mpu;
    bool *dmpReady;
};

#define DEBUG 1
#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#define debugf(x, y) Serial.printf(x, y)
#else
#define debug(x)
#define debugln(x)
#define debugf(x, y)
#endif
#endif