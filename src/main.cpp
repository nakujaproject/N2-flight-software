#include <Arduino.h>
/*
 * Check brownout issues to prevent ESP32 from re-booting unexpectedly
 */

#include "../include/kalmanfilter.h"
#include "../include/checkState.h"
#include "../include/logdata.h"
#include "../include/readsensors.h"
#include "../include/transmitlora.h"
#include "../include/transmitwifi.h"
#include "../include/defs.h"
#include <SPI.h>
#include <reaction-wheel.h>
#include <AsyncMqttClient.h>

AsyncMqttClient mqttClient;

static const BaseType_t pro_cpu = 0;

static const BaseType_t app_cpu = 1;

TimerHandle_t ejectionTimerHandle = NULL;

portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;

TaskHandle_t LoRaTelemetryTaskHandle;
TaskHandle_t WiFiTelemetryTaskHandle;

TaskHandle_t GetDataTaskHandle;

TaskHandle_t SDWriteTaskHandle;

TaskHandle_t TaskRollControl_Handler;

TaskHandle_t GPSTaskHandle;

// if 1 chute has been deployed
uint8_t isChuteDeployed = 0;

float BASE_ALTITUDE = 0;

volatile int state = 0;

static uint8_t lora_queue_length = 100;
static uint8_t wifi_queue_length = 100;
static uint8_t sd_queue_length = 150;
static uint8_t gps_queue_length = 100;

static QueueHandle_t lora_telemetry_queue;
static QueueHandle_t wifi_telemetry_queue;
static QueueHandle_t sdwrite_queue;
static QueueHandle_t gps_queue;

// uninitalised pointers to SPI objects
SPIClass *hspi = NULL;

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

void connectToMqtt() {
  debugln("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  debugln("Connected to MQTT.");
  debug("Session present: ");
  debugln(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  debugln("Disconnected from MQTT.");
}

void onMqttPublish(uint16_t packetId) {
  debug("Publish acknowledged.");
  debug("  packetId: ");
  debugln(packetId);
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
        // debugf("lora task core %d\n", xPortGetCoreID());
        for (int i = 0; i < 5; i++)
        {
            if (xQueueReceive(wifi_telemetry_queue, (void *)&sv, 10) == pdTRUE)
            {
                svRecords[i] = sv;
                svRecords[i].latitude = latitude;
                svRecords[i].longitude = longitude;
            }
            if (xQueueReceive(gps_queue, (void *)&gpsReadings, 10) == pdTRUE)
            {
                latitude = gpsReadings.latitude;
                longitude = gpsReadings.longitude;
            }
        }
        handleWiFi(svRecords, mqttClient);

        // yield to other task such as IDLE task
        vTaskDelay(36 / portTICK_PERIOD_MS);
    }
}

void LoRaTelemetryTask(void *parameter)
{

    struct SendValues sv = {0};
    struct SendValues svRecords[5];
    struct GPSReadings gpsReadings = {0};
    float latitude = 0;
    float longitude = 0;

    for (;;)
    {
        // debugf("lora task core %d\n", xPortGetCoreID());
        for (int i = 0; i < 5; i++)
        {
            if (xQueueReceive(lora_telemetry_queue, (void *)&sv, 10) == pdTRUE)
            {
                svRecords[i] = sv;
                svRecords[i].latitude = latitude;
                svRecords[i].longitude = longitude;
            }
            if (xQueueReceive(gps_queue, (void *)&gpsReadings, 10) == pdTRUE)
            {
                latitude = gpsReadings.latitude;
                longitude = gpsReadings.longitude;
            }
        }

        handleLora(svRecords);

        // to let the idle task run
        vTaskDelay(10 / portTICK_PERIOD_MS);
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
        debugf("sd task core %d\n", xPortGetCoreID());
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
    pinMode(LORA_CS_PIN, OUTPUT);   // HSPI SS for use by LORA
    pinMode(SDCARD_CS_PIN, OUTPUT); // VSPI SS for use by SDCARD

    // set up parachute pin
    pinMode(EJECTION_PIN, OUTPUT);

    createAccessPoint();

    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onPublish(onMqttPublish);
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);
    mqttClient.connect();

    hspi = new SPIClass(HSPI);
    hspi->begin();

    SPIClass &spi = *hspi;

    init_components(spi);


    // get the base_altitude
    BASE_ALTITUDE = get_base_altitude();

    // debugln(sizeof(SendValues)); //24 bytes
    // debugln(sizeof(LogData)); //64 bytes
    // debugln(sizeof(GPSReadings));//20 bytes

    // lora_telemetry_queue = xQueueCreate(lora_queue_length, sizeof(SendValues));
    wifi_telemetry_queue = xQueueCreate(wifi_queue_length, sizeof(SendValues));
    sdwrite_queue = xQueueCreate(sd_queue_length, sizeof(LogData));
    gps_queue = xQueueCreate(gps_queue_length, sizeof(GPSReadings));

    // initialize core tasks
    // TODO: optimize the stackdepth
    // xTaskCreatePinnedToCore(LoRaTelemetryTask, "LoRaTelemetryTask", 3000, NULL, 1, &LoRaTelemetryTaskHandle, 0);
    xTaskCreatePinnedToCore(GetDataTask, "GetDataTask", 3000, NULL, 1, &GetDataTaskHandle, 0);
    xTaskCreatePinnedToCore(WiFiTelemetryTask, "WiFiTelemetryTask", 4000, NULL, 1, &WiFiTelemetryTaskHandle, 0);
    xTaskCreatePinnedToCore(readGPSTask, "ReadGPSTask", 3000, NULL, 1, &GPSTaskHandle, 1);
    xTaskCreatePinnedToCore(SDWriteTask, "SDWriteTask", 4000, NULL, 1, &SDWriteTaskHandle, 1);

    reactionWheelParams rollTaskParam = {&mpu, &dmpReady};
    xTaskCreate(TaskRollControl, "ReactionWheel", 10000, (void *)&rollTaskParam, 1, &TaskRollControl_Handler);

    // Delete setup and loop tasks
    vTaskDelete(NULL);
}
void loop()
{
}