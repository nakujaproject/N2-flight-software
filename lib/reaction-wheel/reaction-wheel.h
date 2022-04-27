#include <Arduino.h>

#define sampleTime 10
double elapsedTime = 0;
double timeCur = 0;
double timePrev = 0;
double anglePrev = 0;
double angleCur = 0;
double _speed = 0;

double rollVel=0;
double pwm = 0;

TaskHandle_t TaskRollControl_Handler;

void TaskRollControl( void *pvParameters );
void write_pwm(float motorspeed);
double constrainpwm (double pwm, double Min, double Max);