#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#define DEBUG 1
#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

#define SEA_LEVEL_PRESSURE 102400

// Timing delays
#define SETUP_DELAY 5000

// define sd card VSPI
const uint8_t SDCARD_CS_PIN =  5;
const uint8_t SD_MOSI_PIN = 23;
const uint8_t SD_MISO_PIN = 19;
const uint8_t SD_SCK_PIN = 18;

// lora HSPI
const uint8_t LORA_CS_PIN = 15;
const uint8_t LORA_MOSI_PIN = 13;
const uint8_t LORA_MISO_PIN = 12;
const uint8_t LORA_SCK_PIN = 14;
const uint8_t RESET_LORA_PIN = 25;
const uint8_t IRQ_LORA_PIN = 2;

const uint8_t GPS_TX_PIN = 17;
const uint8_t GPS_RX_PIN = 16;

#define SHORT_DELAY 10

#define BAUD_RATE 115200

// Pin to start ignition
#define EJECTION_PIN 4


static float BASE_ALTITUDE;

static float MAX_ALTITUDE;

// Lora
const long LORA_FREQ = 868E6; // frequechy 868 MHz
const int LORA_SF = 7;        // spread factor
const long LORA_BW = 125E3;   // bandwidth 125 kHz

#define LORA_SYNC_WORD 0xF3
struct LogData
{
    uint64_t timeStamp;
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
    float latitude;
    float longitude;
    float gpsAltitude;
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

struct SendValues
{
    uint64_t timeStamp;
    float altitude;
    uint16_t state;
};

#endif