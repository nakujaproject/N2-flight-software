#include <Arduino.h>
#include "reaction-wheel.h"
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
double elapsedTime = 0;
double timeCur = 0;
double timePrev = 0;
double anglePrev = 0;
double angleCur = 0;
double _speed = 0;

double rollVel = 0;
double pwm = 0;

double Setpoint = 0;
float Kp = 2.0;
float Ki = 0.0;
float Kd = 1.0;

double Input;
static double Output;

uint16_t fifoCount; // count of all bytes currently in

PID balancePID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int16_t accData[3], gyrData[3];

Servo ESC;

uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion quaternion; // [w, x, y, z]         quaternion container
VectorInt16 aa;        // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;    // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;   // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;   // [x, y, z]            gravity vector
float euler[3];        // [psi, theta, phi]    Euler angle container
// float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};

void Write_pwm(float motorspeed)
{
  ESC.writeMicroseconds(motorspeed);
}
// initialize ESC
void calibrateESC()
{
  debugln("Turn on ESC.");
  ESC.writeMicroseconds(0);
  debugln("Starting Calibration.");
  delay(1000);
  ESC.writeMicroseconds(1832);
  debugln("Writing Full Throttle.");
  delay(1000);
  ESC.writeMicroseconds(1312);
  debugln("Writing Full Reverse.");
  delay(1000);
  ESC.writeMicroseconds(1488);
  debugln("Writing Neutral.");
  delay(1000);
  debugln("Calibration Complete.");
}

double Constrainpwm(double pwm, double Min, double Max)
{
  if (pwm < Min)
    return Min;
  else if (pwm > Max)
    return Max;

  else
    return pwm;
}

void ReactionWheelSetup(void)
{
  ESC.attach(14);
  calibrateESC();
  balancePID.SetMode(AUTOMATIC);         //
  balancePID.SetOutputLimits(-176, 344); // to range from 1312 to 1832( -176,344
}

double GetRollVelocity(MPU6050 mpu, double *currTime, double *currAngle)
{
  static float ypr[3];
  mpu.dmpGetQuaternion(&quaternion, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &quaternion);
  mpu.dmpGetYawPitchRoll(ypr, &quaternion, &gravity);
  Input = ypr[0] * 180 / M_PI;
  *currTime = millis();
  *currAngle = Input;
  elapsedTime = timeCur - timePrev;
  return ((angleCur - anglePrev) / (elapsedTime / 1000.00));
}

void RunReactionWheel(MPU6050 mpu, bool dmpReady)
{
  // if programming failed, don't try to do anything
  if (!dmpReady)
    return;
  // read a packet from FIFO
  if ((mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) && (elapsedTime > sampleTime))
  { // Get the Latest packet
    rollVel = GetRollVelocity(mpu, &timeCur, &angleCur);
    while (Input <= -180)
      Input += 360;
    while (Input > 180)
      Input -= 360;

    debug("Input:\t");
    debugln(Input);

    if (Input < 0)
    {
      balancePID.SetControllerDirection(REVERSE);
    }
    else
    {
      balancePID.SetControllerDirection(DIRECT);
    }

    balancePID.Compute();

    pwm = 1488 + Output;
    Constrainpwm(pwm, 1450, 1510);

    timePrev = timeCur;
    anglePrev = angleCur;

    // pwm, output
    Write_pwm(pwm);
  }
}
