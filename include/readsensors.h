#ifndef READSENSORS_H
#define READSENSORS_H

// #include <FS.h>
// #include <SD.h>
// #include <SPI.h>
#include <mySD.h>
#include <Adafruit_BMP085.h>
//#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "logdata.h"
#include "defs.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <LoRa.h>

//fc
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// using uart 2 for serial communication
SoftwareSerial GPSModule(GPS_RX_PIN, GPS_TX_PIN); // RX, TX

Adafruit_BMP085 bmp;
//Adafruit_MPU6050 mpu;
MPU6050 mpu;


int updates;
int pos;
int stringplace = 0;

String timeUp;
String nmea[15];
String labels[12]{"Time: ", "Status: ", "Latitude: ", "Hemisphere: ", "Longitude: ", "Hemisphere: ", "Speed: ", "Track Angle: ", "Date: "};


String ConvertLat()
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

String ConvertLng()
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

void dmpDataReady() {
    mpuInterrupt = true;
}

// function to initialize bmp, mpu, lora module and the sd card module
void init_components(SPIClass &spi)
{

    // debugln("GPS INITIALIZATION");
    // GPSModule.begin(9600);
    // debugln("GPS FOUND");

    debugln("BMP180 INITIALIZATION");
    if (!bmp.begin())
    {
        debugln("Could not find a valid BMP085 sensor, check wiring!");
        while (1)
        {
            delay(SHORT_DELAY);
        }
    }
    else
    {
        ;
    }

    debugln("BMP180 FOUND");

    debugln("MPU6050 test!");

    // Initialize MPU6050 accelerometer
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin(SDA, SCL, 400000);
        //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
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

    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        //.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        //Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
        debugln("Could not find a valid MPU6050 sensor, check wiring!");
        while (1)
        {
            delay(SHORT_DELAY);
        }
    }

    debugln("MPU6050 FOUND");
    //mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    //mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    //mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

    debugln("SD_CARD INITIALIZATION");
    if (!SD.begin(SDCARD_CS_PIN, SD_MOSI_PIN, SD_MISO_PIN, SD_SCK_PIN))
    {
        debugln("Could not find a valid SD Card, check wiring!");
        while (1)
        {
            delay(SHORT_DELAY);
        }
    }
    else
    {
        ;
    }
    debugln("SD CARD FOUND");

    debugln("LORA INITIALIZATION");
    debug("Setting up LoRa Sender...");

    LoRa.setPins(LORA_CS_PIN, RESET_LORA_PIN, IRQ_LORA_PIN); // set CS, reset, IRQ pin
    LoRa.setSPI(spi);

    while (!LoRa.begin(LORA_FREQ))
    {
        debug(".");
    }

    debugln();
    debugln("Successfully set up LoRa");

    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setSignalBandwidth(LORA_BW);
    LoRa.setSyncWord(LORA_SYNC_WORD);
    debug("Frequency :");
    debug(LORA_FREQ);
    debug("Bandwidth :");
    debug(LORA_BW);
    debug("SF :");
    debugln(LORA_SF);
    debugln("LORA FOUND");
}

// Get the gps readings from the gps sensor
struct GPSReadings get_gps_readings()
{
    struct GPSReadings gpsReadings;
    Serial.flush();
    GPSModule.flush();
    while (GPSModule.available() > 0)
    {
        GPSModule.read();
    }
    if (GPSModule.find("$GPRMC,"))
    {
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
        updates++;
        float lati = ConvertLat().toFloat();
        float lngi = ConvertLng().toFloat();
        gpsReadings.latitude = lati;
        gpsReadings.longitude = lngi;
    }

    stringplace = 0;
    pos = 0;

    return gpsReadings;
}

// Get the sensor readings
struct SensorReadings get_readings()
{
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    struct SensorReadings return_val;
    //sensors_event_t a, g, temp;
    //mpu.getEvent(&a, &g, &temp);
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