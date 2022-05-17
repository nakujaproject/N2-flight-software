#include <Arduino.h>

/*
 * Check brownout issues to prevent ESP32 from re-booting unexpectedly
 */

#include "../include/kalmanfilter.h"
#include "../include/checkState.h"
#include "../include/logdata.h"
#include "../include/readsensors.h"
#include "../include/transmitwifi.h"
#include "../include/defs.h"
#include <SPI.h>


static const BaseType_t pro_cpu = 0;

static const BaseType_t app_cpu = 1;

TimerHandle_t ejectionTimerHandle = NULL;

portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;


TaskHandle_t WiFiTelemetryTaskHandle;

TaskHandle_t GetDataTaskHandle;

TaskHandle_t SDWriteTaskHandle;

TaskHandle_t GPSTaskHandle;

// if 1 chute has been deployed
uint8_t isChuteDeployed = 0;

float BASE_ALTITUDE = 0;

volatile int state = 0;


static uint16_t wifi_queue_length = 100;
static uint16_t sd_queue_length = 300;
static uint16_t gps_queue_length = 100;


static QueueHandle_t wifi_telemetry_queue;
static QueueHandle_t sdwrite_queue;
static QueueHandle_t gps_queue;


// callback for done ejection
void ejectionTimerCallback(TimerHandle_t ejectionTimerHandle)
{
    digitalWrite(EJECTION_PIN, LOW);
    isChuteDeployed = 1;
}

// Ejection Fires the explosive charge using a relay or a mosfet
void ejection()
{
    if (isChuteDeployed == 0)
    {
        digitalWrite(EJECTION_PIN, HIGH);
        // TODO: is 3 seconds enough?
        ejectionTimerHandle = xTimerCreate("EjectionTimer", 3000 / portTICK_PERIOD_MS, pdFALSE, (void *)0, ejectionTimerCallback);
        xTimerStart(ejectionTimerHandle, portMAX_DELAY);
    }
}



struct LogData readData()
{
    struct LogData ld = {0};
    struct SensorReadings readings = {0};
    struct FilteredValues filtered_values = {0};

    readings = get_readings();

    // TODO: very important to know the orientation of the altimeter
    filtered_values = kalmanUpdate(readings.altitude, readings.ay);

    // using mutex since we are modifying a volatile var

    portENTER_CRITICAL(&mutex);
    state = checkState(filtered_values.displacement, filtered_values.velocity, state);
    portEXIT_CRITICAL(&mutex);

    ld = formart_data(readings, filtered_values);
    ld.state = state;
    ld.timeStamp = millis();

    return ld;
}

void GetDataTask(void *parameter)
{

    struct LogData ld = {0};
    struct SendValues sv = {0};
    for (;;)
    {
        // debugf("data task core %d\n", xPortGetCoreID());

        ld = readData();
        sv = formart_send_data(ld);
       // debugln(ld.timeStamp);
        if (xQueueSend(wifi_telemetry_queue, (void *)&sv, 0) != pdTRUE)
        {
            debugln("Telemetry Queue Full!");
        }
        if (xQueueSend(sdwrite_queue, (void *)&ld, 0) != pdTRUE)
        {
            debugln("SD card Queue Full!");
        }
        // yield to other task such as IDLE task
        vTaskDelay(74 / portTICK_PERIOD_MS);
    }
}
void readGPSTask(void *parameter)
{
    debugf("gps task core %d\n", xPortGetCoreID());
    struct GPSReadings gpsReadings = {0};
    for (;;)
    {
        gpsReadings = get_gps_readings();
        // debugf("latitude %.8f\n",gpsReadings.latitude);
        // debugf("longitude %.8f\n",gpsReadings.longitude);
        
        if (xQueueSend(gps_queue, (void *)&gpsReadings, 0) != pdTRUE)
        {
             debugln("GPS card Queue Full!");
        }
        // yield to other task such as IDLE task
        vTaskDelay(60 / portTICK_PERIOD_MS);
    }
}

void WiFiTelemetryTask(void *parameter)
{
    struct SendValues sv = {0};
    struct SendValues svRecords[5];
    struct GPSReadings gpsReadings = {0};
    float latitude = 0;
    float longitude = 0;

    for (;;)
    {
    
        for (int i = 0; i < 5; i++)
        {
            xQueueReceive(wifi_telemetry_queue, (void *)&sv, 10);
            
                svRecords[i] = sv;
                svRecords[i].latitude = latitude;
                svRecords[i].longitude = longitude;
              // debugln(svRecords[i].timeStamp);
            if (xQueueReceive(gps_queue, (void *)&gpsReadings, 10) == pdTRUE)
            {
                latitude = gpsReadings.latitude;
                longitude = gpsReadings.longitude;
            }
           debugln(svRecords[i].state);
           
        }

         handleWiFi(svRecords);

        // yield to other task such as IDLE task
        vTaskDelay(36 / portTICK_PERIOD_MS);
    }
}


void SDWriteTask(void *parameter)
{

    struct LogData ld = {0};
    struct LogData ldRecords[5];
    struct GPSReadings gps = {0};
    float latitude = 0;
    float longitude = 0;

    for (;;)
    {
        // debugf("sd task core %d\n", xPortGetCoreID());
        for (int i = 0; i < 5; i++)
        {
            if (xQueueReceive(sdwrite_queue, (void *)&ld, 10) == pdTRUE)
            {
                ldRecords[i] = ld;
                ldRecords[i].latitude = latitude;
                ldRecords[i].longitude = longitude;
            }
            if (xQueueReceive(gps_queue, (void *)&gps, 10) == pdTRUE)
            {
                latitude = gps.latitude;
                longitude = gps.longitude;
            }
            // debugln(ldRecords[i].timeStamp);
        }
        appendToFile(ldRecords);

        // yield to other task such as IDLE task
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void setup()
{

    Serial.begin(BAUD_RATE);

    // set up slave select pins as outputs as the Arduino API
    //pinMode(SDCARD_CS_PIN, OUTPUT); // VSPI SS for use by SDCARD

    // set up parachute pin
    pinMode(EJECTION_PIN, OUTPUT);

    init_components();


    // get the base_altitude
    BASE_ALTITUDE = get_base_altitude();

    // debugln(sizeof(SendValues)); //24 bytes
    // debugln(sizeof(LogData)); //64 bytes
    // debugln(sizeof(GPSReadings));//20 bytes

    
    wifi_telemetry_queue = xQueueCreate(wifi_queue_length, sizeof(SendValues));
    sdwrite_queue = xQueueCreate(sd_queue_length, sizeof(LogData));
    gps_queue = xQueueCreate(gps_queue_length, sizeof(GPSReadings));

    // initialize core tasks
    // TODO: optimize the stackdepth
   
    xTaskCreatePinnedToCore(GetDataTask, "GetDataTask", 3000, NULL, 1, &GetDataTaskHandle, 0);
    xTaskCreatePinnedToCore(WiFiTelemetryTask, "WiFiTelemetryTask", 4000, NULL, 1, &WiFiTelemetryTaskHandle, 0);
    xTaskCreatePinnedToCore(readGPSTask, "ReadGPSTask", 3000, NULL, 1, &GPSTaskHandle, 1);
    xTaskCreatePinnedToCore(SDWriteTask, "SDWriteTask", 4000, NULL, 1, &SDWriteTaskHandle, 1);

    // Delete setup and loop tasks
    vTaskDelete(NULL);
}
void loop()
{
}