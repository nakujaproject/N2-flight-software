#include <Arduino.h>
/*
 * Check brownout issues to prevent ESP32 from re-booting unexpectedly
 */
#include "../include/functions.h"
#include "../include/kalmanfilter.h"
#include "../include/checkState.h"
#include "../include/logdata.h"
#include "../include/readsensors.h"
#include "../include/transmitlora.h"
#include "../include/defs.h"
#include <SPI.h>
#include <reaction-wheel.h>

static const BaseType_t pro_cpu = 0;

static const BaseType_t app_cpu = 1;

TaskHandle_t LoRaTelemetryTaskHandle;

TaskHandle_t GetDataTaskHandle;

TaskHandle_t SDWriteTaskHandle;

TaskHandle_t TaskRollControl_Handler;

static uint8_t queue_length = 10;
static QueueHandle_t telemetry_queue;
static QueueHandle_t sdwrite_queue;

portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;

volatile int state;

// uninitalised pointers to SPI objects
SPIClass *hspi = NULL;

struct LogData readData()
{
    struct LogData ld;
    struct SensorReadings readings;
    struct FilteredValues filtered_values;
    struct GPSReadings gpsReadings;

    readings = get_readings();

    // TODO: very important to know the orientation of the altimeter
    filtered_values = kalmanUpdate(readings.altitude, readings.ay);

    // using mutex since we are modifying a volatile var

    portENTER_CRITICAL(&mutex);
    state = checkState(filtered_values.displacement, filtered_values.velocity, state);
    portEXIT_CRITICAL(&mutex);

    // if we reach ground state we can start reading gpsData
    if (state == 4)
    {
        gpsReadings = get_gps_readings();
    }

    ld = formart_data(readings, filtered_values, gpsReadings);
    ld.state = state;
    ld.timeStamp = micros();

    return ld;
}

void GetDataTask(void *parameter)
{

    struct LogData ld;
    struct SendValues sv;
    for (;;)
    {
        debugln("getdata task" );

        ld = readData();
        sv = formart_send_data(ld);
        if (xQueueSend(telemetry_queue, (void *)&sv, 0) != pdTRUE)
        {
            debug("Telemetry Queue Full!");
        }
        if (xQueueSend(sdwrite_queue, (void *)&ld, 0) != pdTRUE)
        {
            debug("SD card Queue Full!");
        }
    }
}

void LoRaTelemetryTask(void *parameter)
{
    

    struct SendValues sv;
    struct SendValues svRecords[5];

    for (;;)
    {
        debugln("lora task");
        for (int i = 0; i < 5; i++)
        {
            xQueueReceive(telemetry_queue, (void *)&sv, 10);
            svRecords[i] = sv;
        }
        portENTER_CRITICAL(&mutex);
        state = handleLora(state, svRecords);
        portEXIT_CRITICAL(&mutex);
        // yield to another task
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void SDWriteTask(void *parameter)
{

    struct LogData ld;
    struct LogData ldRecords[5];

    for (;;)
    {
        debugln("SD task");
        for (int i = 0; i < 5; i++)
        {
            xQueueReceive(sdwrite_queue, (void *)&ld, 10);
            ldRecords[i] = ld;
        }
        appendToFile(ldRecords);
        // yield to another task
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void setup()
{

    Serial.begin(115200);

    // set up slave select pins as outputs as the Arduino API
    pinMode(LORA_CS_PIN, OUTPUT);   // HSPI SS for use by LORA
    pinMode(SDCARD_CS_PIN, OUTPUT); // VSPI SS for use by SDCARD

    // set up parachute pin
    pinMode(EJECTION_PIN, OUTPUT);

    hspi = new SPIClass(HSPI);
    hspi->begin();

    SPIClass &spi = *hspi;

    init_components(spi);

    // get the base_altitude
    BASE_ALTITUDE = get_base_altitude();

    telemetry_queue = xQueueCreate(queue_length, sizeof(SendValues));
    sdwrite_queue = xQueueCreate(queue_length, sizeof(LogData));

    // initialize core tasks
    // TODO: optimize the stackdepth
    xTaskCreatePinnedToCore(LoRaTelemetryTask, "LoRaTelemetryTask", 10000, NULL, 1, &LoRaTelemetryTaskHandle, pro_cpu);
    xTaskCreatePinnedToCore(SDWriteTask, "SDWriteTask", 10000, NULL, 1, &SDWriteTaskHandle, pro_cpu);
    xTaskCreatePinnedToCore(GetDataTask, "GetDataTask", 10000, NULL, 1, &GetDataTaskHandle, app_cpu);

    reactionWheelParams rollTaskParam = {&mpu, &dmpReady};
    xTaskCreate(TaskRollControl, "ReactionWheel", 10000, (void *)&rollTaskParam, 1, &TaskRollControl_Handler);

    // Delete "setup and loop" task
    vTaskDelete(NULL);
}
void loop() {}
