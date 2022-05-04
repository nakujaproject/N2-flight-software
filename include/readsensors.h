#ifndef READSENSORS_H
#define READSENSORS_H

// #include <FS.h>
// #include <SD.h>
// #include <SPI.h>
#include <mySD.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "logdata.h"
#include "defs.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <WiFi.h>
#include <LoRa.h>

// using uart 2 for serial communication
SoftwareSerial GPSModule(GPS_RX_PIN, GPS_TX_PIN); // RX, TX

Adafruit_BMP085 bmp;
Adafruit_MPU6050 mpu;

void createAccessPoint()
{
    // Connect to Wi-Fi network with SSID and password
    debugln("Creating Rocket Access Point...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, key);
    IPAddress IP = WiFi.softAPIP();
    debug("ssid:");
    debugln(ssid);
    debug("key:");
    debugln(key);
    debug("Rocket IP address: ");
    debugln(IP);
    Udp.begin(UDP_PORT);
}

// function to initialize bmp, mpu, lora module and the sd card module
void init_components(SPIClass &spi)
{

    GPSModule.begin(9600);

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
    if (!mpu.begin())
    {
        debugln("Could not find a valid MPU6050 sensor, check wiring!");
        while (1)
        {
            delay(SHORT_DELAY);
        }
    }
    else
    {
        ;
    }

    debugln("MPU6050 FOUND");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

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
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    return_val.altitude = bmp.readAltitude(SEA_LEVEL_PRESSURE);
    return_val.ax = a.acceleration.x;
    return_val.ay = a.acceleration.y;
    return_val.az = a.acceleration.z;

    return_val.gx = g.gyro.x;
    return_val.gy = g.gyro.y;
    return_val.gz = g.gyro.z;

    return return_val;
}

#endif