#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "defs.h"
#include "readsensors.h"

void ejection();
void ejectionTimerCallback(TimerHandle_t ejectionTimerHandle);

// DummyData returns a dummy data sample for LogData
// This is helpful for analysis
struct LogData DummyData()
{
  struct LogData ld;
  ld.timeStamp = 2;
  ld.altitude = 10.5;
  ld.ax = 2.56;
  ld.ay = 2.65;
  ld.az = 1.02;
  ld.gx = 5.8;
  ld.gy = 85.6;
  ld.gz = 2.6;
  ld.filtered_s = 3.56;
  ld.filtered_a = 9.85;
  ld.filtered_v = 1005.3;
  ld.state = 5;
  ld.longitude = 2.4;
  ld.latitude = 3.2;
  return ld;
}

// formart_data This formats data that we are going to save to SD card
// We pass in our SensorReadings, FilteredValues and GPSReadings
// We save all the data points we are collecting
struct LogData formart_data(SensorReadings readings, FilteredValues filtered_values)
{
  struct LogData ld = {0};
  ld.altitude = readings.altitude;
  ld.ax = readings.ax;
  ld.ay = readings.ay;
  ld.az = readings.az;
  ld.gx = readings.gx;
  ld.gy = readings.gy;
  ld.gz = readings.gz;
  ld.filtered_s = filtered_values.displacement;
  ld.filtered_a = filtered_values.acceleration;
  ld.filtered_v = filtered_values.velocity;
  return ld;
}

// formart_send_data This formats data we are going to send over LoRa
// Currently we are sending altitude, state and timeStamp
struct SendValues formart_send_data(LogData readings)
{
  struct SendValues sv = {0};
  sv.altitude = readings.altitude;
  sv.state = readings.state;
  sv.timeStamp = readings.timeStamp;
  sv.latitude = readings.latitude;
  sv.longitude = readings.longitude;
  return sv;
}

// get_base_altitude Finds the average of the current altitude from 100 readings
float get_base_altitude()
{
  float altitude = 0;
  SensorReadings readings;
  for (int i = 0; i < 100; i++)
  {
    readings = get_readings();
    altitude = altitude + readings.altitude;

  }
  altitude = altitude / 100.0;
  debugf("Base Altitude is %.3f\n",altitude);
  return altitude;
}
#endif