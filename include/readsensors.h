#ifndef READSENSORS_H
#define READSENSORS_H

#include <FS.h>
#include <SPI.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
#include "defs.h"
#include <SoftwareSerial.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// using uart 2 for serial communication
SoftwareSerial GPSModule(GPS_RX_PIN, GPS_TX_PIN);

Adafruit_BMP085 bmp;
MPU6050 mpu;

void dmpDataReady()
{
    mpuInterrupt = true;
}

void init_gps()
{
    GPSModule.begin(GPS_BAUD_RATE);
}

void init_mpu()
{
    debugln("MPU6050 test!");

// Initialize MPU6050 accelerometer
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin(SDA, SCL, 400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT_PULLUP);

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0)
    {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        debugln("Enabling DMP");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        debugln("Enabling interrupt detection (Arduino external interrupt ");
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        debugln("DMP ready! Waiting for first interrupt");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        debug("DMP Initialization failed (code ");
        debugln(devStatus);
        debugln("Could not find a valid MPU6050 sensor, check wiring!");
        while (1)
        {
            // TODO: add beep to notify
        }
    }

    debugln("MPU6050 FOUND");
}

void init_bmp()
{
    debugln("BMP180 INITIALIZATION");
    if (!bmp.begin())
    {
        debugln("Could not find a valid BMP085 sensor, check wiring!");
        while (1)
        {
            // TODO: add beep to notify
        }
    }
    else
    {
        debugln("BMP180 FOUND");
    }
}

// function to initialize bmp, mpu, lora module and the sd card module
void init_sensors()
{
    init_gps();
    init_bmp();
    init_mpu();
}

String ConvertLat(String nmea[15])
{
    String posneg = "";
    if (nmea[3] == "S")
    {
        posneg = "-";
    }
    String latfirst;
    float latsecond;
    for (int i = 0; i < nmea[2].length(); i++)
    {
        if (nmea[2].substring(i, i + 1) == ".")
        {
            latfirst = nmea[2].substring(0, i - 2);
            latsecond = nmea[2].substring(i - 2).toFloat();
        }
    }
    latsecond = latsecond / 60;
    String CalcLat = "";

    char charVal[9];
    dtostrf(latsecond, 4, 6, charVal);
    for (int i = 0; i < sizeof(charVal); i++)
    {
        CalcLat += charVal[i];
    }
    latfirst += CalcLat.substring(1);
    latfirst = posneg += latfirst;
    return latfirst;
}

String ConvertLng(String nmea[15])
{
    String posneg = "";
    if (nmea[5] == "W")
    {
        posneg = "-";
    }

    String lngfirst;
    float lngsecond;
    for (int i = 0; i < nmea[4].length(); i++)
    {
        if (nmea[4].substring(i, i + 1) == ".")
        {
            lngfirst = nmea[4].substring(0, i - 2);
            // debugln(lngfirst);
            lngsecond = nmea[4].substring(i - 2).toFloat();
            // debugln(lngsecond);
        }
    }
    lngsecond = lngsecond / 60;
    String CalcLng = "";
    char charVal[9];
    dtostrf(lngsecond, 4, 6, charVal);
    for (int i = 0; i < sizeof(charVal); i++)
    {
        CalcLng += charVal[i];
    }
    lngfirst += CalcLng.substring(1);
    lngfirst = posneg += lngfirst;
    return lngfirst;
}

// Get the gps readings from the gps sensor
struct GPSReadings get_gps_readings()
{
    String nmea[15];
    int stringplace = 0;
    int pos = 0;
    struct GPSReadings gpsReadings;
    GPSModule.flush();
    while (GPSModule.available() > 0)
    {
        GPSModule.read();
    }
    if (GPSModule.find("$GPRMC,"))
    {
        debug("here");
        String tempMsg = GPSModule.readStringUntil('\n');
        for (int i = 0; i < tempMsg.length(); i++)
        {
            if (tempMsg.substring(i, i + 1) == ",")
            {
                nmea[pos] = tempMsg.substring(stringplace, i);
                stringplace = i + 1;
                pos++;
            }
            if (i == tempMsg.length() - 1)
            {
                nmea[pos] = tempMsg.substring(stringplace, i);
            }
        }
        float lati = ConvertLat(nmea).toFloat();
        float lngi = ConvertLng(nmea).toFloat();
        gpsReadings.latitude = lati;
        gpsReadings.longitude = lngi;
    }

    return gpsReadings;
}

// Get the sensor readings
struct SensorReadings get_readings()
{
    struct SensorReadings return_val;

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    return_val.altitude = bmp.readAltitude(SEA_LEVEL_PRESSURE);

    return_val.ax = ax;
    return_val.ay = ay;
    return_val.az = az;

    return_val.gx = gx;
    return_val.gy = gy;
    return_val.gz = gz;

    return return_val;
}

#endif