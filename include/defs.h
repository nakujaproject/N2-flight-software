#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#define SEA_LEVEL_PRESSURE_HPA 1024
#define SEA_LEVEL_PRESSURE 102400

// Timing delays
#define SETUP_DELAY 1000

// define sd card VSPI
#define SDCARD_CS_PIN 5
#define SD_MOSI_PIN 23
#define SD_MISO_PIN 19
#define SD_SCK_PIN 18

// define lora HSPI
#define LORA_CS_PIN 15
#define LORA_MOSI_PIN 13
#define LORA_MISO_PIN 12
#define LORA_SCK_PIN 14
#define RESET_LORA_PIN 25
#define IRQ_LORA_PIN 2

#define GPS_TX_PIN 17
#define GPS_RX_PIN 16

#define SHORT_DELAY 10

#define BAUD_RATE 115200

#define UDP_PORT 4210

// value to detect lif off deviation
#define LIFT_OFF_DEVIATION 2 // ideal: 50 cm

#define EJECTION_PIN 4

const char *telemetryLogFile = "telemetry.txt";
const char *gpsLogFile = "gps.txt";

struct LogData
{
    int counter;
    float altitude;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float filtered_s;
    float filtered_v;
    float filtered_a;
    int state;
};

struct SensorReadings
{
    float altitude;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
};
struct GPSReadings
{
    float latitude;
    float longitude;
    float speed;
    int satellites;
    float altitude;
};

struct FilteredValues
{
    float displacement;
    float velocity;
    float acceleration;
};

#endif