#include <Arduino.h>
#include "reaction-wheel.h"
#include "readsensors.h"

void Write_pwm(float motorspeed)
{
  ESC.writeMicroseconds(motorspeed);
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

void TaskRollControl(void *pvParameters)
{
  (void)pvParameters;
  // TODO move MPU and ESC initialization here
  for (;;)
  {
    // if programming failed, don't try to do anything
    if (!dmpReady)
      return;
    // read a packet from FIFO
    if ((mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) && (elapsedTime > sampleTime))
    { // Get the Latest packet
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&quaternion, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &quaternion);
      mpu.dmpGetYawPitchRoll(ypr, &quaternion, &gravity);

      mpu.getMotion6(&accData[0], &accData[1], &accData[2], &gyrData[0], &gyrData[1], &gyrData[2]);

      Input = ypr[0] * 180 / M_PI;
      timePrev = timeCur;
      timeCur = millis();
      angleCur = Input;
      elapsedTime = timeCur - timePrev;
      rollVel = ((angleCur - anglePrev) / (elapsedTime / 1000.00));

      //            Serial.print("ypr\t");
      //            Serial.print(ypr[0] * 180/M_PI);
      //            Serial.print("\t");
      //            Serial.print(ypr[1] * 180/M_PI);
      //            Serial.print("\t");
      //            Serial.println(ypr[2] * 180/M_PI);

      while (Input <= -180)
        Input += 360;
      while (Input > 180)
        Input -= 360;

      Serial.print("Input:\t");
      Serial.println(Input);

      // blink LED to indicate activity
      // blinkState = !blinkState;
      // digitalWrite(LED_PIN, blinkState);

      Setpoint = 0;

      // Serial.print("Input:\t");
      // Serial.println(Input);

      if (Input < 0)
      {
        balancePID.SetControllerDirection(REVERSE);
      }
      else
      {
        balancePID.SetControllerDirection(DIRECT);
      }

      // Serial.print("Output1:\t");
      // Serial.println(Output);

      balancePID.Compute();

      // Serial.print("Output2:\t");
      // Serial.println(Output);
      pwm = 1488 + Output;
      Constrainpwm(pwm, 1450, 1510);

      timePrev = timeCur;
      anglePrev = angleCur;
      // pwm Output
      // Serial.print("pwm: ");
      // Serial.println(pwm);

      // pwm, output
      Write_pwm(pwm);
    }
  }
}
