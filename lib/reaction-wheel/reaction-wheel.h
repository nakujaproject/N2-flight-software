#ifndef REACTIONwHEEL_H
#define REACTIONwHEEL_H
#include <Arduino.h>
#include "PID_v1.h"

#include <ESP32Servo.h>
#include "MPU6050_6Axis_MotionApps20.h"

#define sampleTime 10


void TaskRollControl( void *pvParameters );
void Write_pwm(float);
double Constrainpwm (double, double, double);





#define OUTPUT_READABLE_YAWPITCHROLL

#define SDA 21
#define SCL 22

// MPU control/status vars




struct reactionWheelParams{
    MPU6050 *mpu;
    bool *dmpReady;
};
#endif