#include <Arduino.h>
<<<<<<< HEAD
=======

>>>>>>> mqtt
/*
 * Check brownout issues to prevent ESP32 from re-booting unexpectedly
 */

<<<<<<< HEAD
#include "../include/kalmanfilter.h"
#include "../include/checkState.h"
#include "../include/logdata.h"
#include "../include/readsensors.h"
#include "../include/transmitlora.h"
#include "../include/transmitwifi.h"
#include "../include/defs.h"
#include <SPI.h>
#include <AsyncMqttClient.h>

AsyncMqttClient mqttClient;

static const BaseType_t pro_cpu = 0;

static const BaseType_t app_cpu = 1;
=======
#include "../include/checkState.h"
#include "../include/logdata.h"
#include "../include/readsensors.h"
#include "../include/transmitwifi.h"
#include "../include/defs.h"
#include "../include/kalmanfilter.h"

>>>>>>> mqtt

TimerHandle_t ejectionTimerHandle = NULL;

portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;

<<<<<<< HEAD
TaskHandle_t LoRaTelemetryTaskHandle;
=======
>>>>>>> mqtt
TaskHandle_t WiFiTelemetryTaskHandle;

TaskHandle_t GetDataTaskHandle;

TaskHandle_t SDWriteTaskHandle;

TaskHandle_t GPSTaskHandle;

// if 1 chute has been deployed
uint8_t isChuteDeployed = 0;

float BASE_ALTITUDE = 0;

volatile int state = 0;

<<<<<<< HEAD
static uint8_t lora_queue_length = 100;
static uint8_t wifi_queue_length = 100;
static uint8_t sd_queue_length = 150;
static uint8_t gps_queue_length = 100;

static QueueHandle_t lora_telemetry_queue;
=======
static uint16_t wifi_queue_length = 100;
static uint16_t sd_queue_length = 100;
static uint16_t gps_queue_length = 100;

>>>>>>> mqtt
static QueueHandle_t wifi_telemetry_queue;
static QueueHandle_t sdwrite_queue;
static QueueHandle_t gps_queue;

<<<<<<< HEAD
// uninitalised pointers to SPI objects
SPIClass *hspi = NULL;

=======
>>>>>>> mqtt
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

<<<<<<< HEAD
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

=======
>>>>>>> mqtt
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

<<<<<<< HEAD
=======

/*
**********Time Taken for each Task******************
        Get Data Task  - 36ms
        WiFiTelemetryTask -74ms
        GPS Task - 1000ms
        SD Write Task - 60ms
*/



>>>>>>> mqtt
void GetDataTask(void *parameter)
{

    struct LogData ld = {0};
    struct SendValues sv = {0};
<<<<<<< HEAD
    for (;;)
    {
        // debugf("data task core %d\n", xPortGetCoreID());
=======

    static int droppedWiFiPackets = 0;
    static int droppedSDPackets = 0;

    for (;;)
    {
>>>>>>> mqtt

        ld = readData();
        sv = formart_send_data(ld);

        if (xQueueSend(wifi_telemetry_queue, (void *)&sv, 0) != pdTRUE)
        {
<<<<<<< HEAD
            debugln("Telemetry Queue Full!");
        }
        if (xQueueSend(sdwrite_queue, (void *)&ld, 0) != pdTRUE)
        {
            debugln("SD card Queue Full!");
        }
        // yield to other task such as IDLE task
=======
            droppedWiFiPackets++;
        }
        if (xQueueSend(sdwrite_queue, (void *)&ld, 0) != pdTRUE)
        {
            droppedSDPackets++;
        }

        debugf("Dropped WiFi Packets : %d\n", droppedWiFiPackets);
        debugf("Dropped SD Packets : %d\n", droppedSDPackets);

       

        // yield to WiFi Telemetry task
>>>>>>> mqtt
        vTaskDelay(74 / portTICK_PERIOD_MS);
    }
}
void readGPSTask(void *parameter)
{
<<<<<<< HEAD
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

=======

    struct GPSReadings gpsReadings = {0};

    static int droppedGPSPackets = 0;

    for (;;)
    {
        gpsReadings = get_gps_readings();

        if (xQueueSend(gps_queue, (void *)&gpsReadings, 0) != pdTRUE)
        {
            droppedGPSPackets++;
        }

        debugf("Dropped GPS Packets : %d\n", droppedGPSPackets);

        

        // yield SD Write task
        //TODO: increase this up to 1000 in steps of 60 to improve queue performance
        vTaskDelay(60 / portTICK_PERIOD_MS);
    }
}

void WiFiTelemetryTask(void *parameter)
{
>>>>>>> mqtt
    struct SendValues sv = {0};
    struct SendValues svRecords[5];
    struct GPSReadings gpsReadings = {0};
    float latitude = 0;
    float longitude = 0;

    for (;;)
    {
<<<<<<< HEAD
        // debugf("lora task core %d\n", xPortGetCoreID());
        for (int i = 0; i < 5; i++)
        {
            if (xQueueReceive(lora_telemetry_queue, (void *)&sv, 10) == pdTRUE)
            {
                svRecords[i] = sv;
                svRecords[i].latitude = latitude;
                svRecords[i].longitude = longitude;
            }
=======

        for (int i = 0; i < 5; i++)
        {
            xQueueReceive(wifi_telemetry_queue, (void *)&sv, 10);
            svRecords[i] = sv;
            svRecords[i].latitude = latitude;
            svRecords[i].longitude = longitude;

>>>>>>> mqtt
            if (xQueueReceive(gps_queue, (void *)&gpsReadings, 10) == pdTRUE)
            {
                latitude = gpsReadings.latitude;
                longitude = gpsReadings.longitude;
            }
        }

<<<<<<< HEAD
        handleLora(svRecords);

        // to let the idle task run
        vTaskDelay(10 / portTICK_PERIOD_MS);
=======
        handleWiFi(svRecords);

        // yield to Get Data task
        vTaskDelay(36 / portTICK_PERIOD_MS);
>>>>>>> mqtt
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
<<<<<<< HEAD
        debugf("sd task core %d\n", xPortGetCoreID());
        for (int i = 0; i < 5; i++)
        {
            if (xQueueReceive(sdwrite_queue, (void *)&ld, 10) == pdTRUE)
            {
                ldRecords[i] = ld;
                ldRecords[i].latitude = latitude;
                ldRecords[i].longitude = longitude;
            }
=======

        for (int i = 0; i < 5; i++)
        {
            xQueueReceive(sdwrite_queue, (void *)&ld, 10);

            ldRecords[i] = ld;
            ldRecords[i].latitude = latitude;
            ldRecords[i].longitude = longitude;

>>>>>>> mqtt
            if (xQueueReceive(gps_queue, (void *)&gps, 10) == pdTRUE)
            {
                latitude = gps.latitude;
                longitude = gps.longitude;
            }
        }
        appendToFile(ldRecords);

<<<<<<< HEAD
        // yield to other task such as IDLE task
=======
        // yield to GPS Task
>>>>>>> mqtt
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void setup()
{

    Serial.begin(BAUD_RATE);

<<<<<<< HEAD
    // set up slave select pins as outputs as the Arduino API
    pinMode(LORA_CS_PIN, OUTPUT);   // HSPI SS for use by LORA
    pinMode(SDCARD_CS_PIN, OUTPUT); // VSPI SS for use by SDCARD
=======
    // VSPI SS for use by SDCARD
    pinMode(SD_CS_PIN, OUTPUT);
>>>>>>> mqtt

    // set up parachute pin
    pinMode(EJECTION_PIN, OUTPUT);

<<<<<<< HEAD
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

=======
    init_components();
>>>>>>> mqtt

    // get the base_altitude
    BASE_ALTITUDE = get_base_altitude();

<<<<<<< HEAD
    // debugln(sizeof(SendValues)); //24 bytes
    // debugln(sizeof(LogData)); //64 bytes
    // debugln(sizeof(GPSReadings));//20 bytes

    // lora_telemetry_queue = xQueueCreate(lora_queue_length, sizeof(SendValues));
=======
>>>>>>> mqtt
    wifi_telemetry_queue = xQueueCreate(wifi_queue_length, sizeof(SendValues));
    sdwrite_queue = xQueueCreate(sd_queue_length, sizeof(LogData));
    gps_queue = xQueueCreate(gps_queue_length, sizeof(GPSReadings));

    // initialize core tasks
    // TODO: optimize the stackdepth
<<<<<<< HEAD
    // xTaskCreatePinnedToCore(LoRaTelemetryTask, "LoRaTelemetryTask", 3000, NULL, 1, &LoRaTelemetryTaskHandle, 0);
    xTaskCreatePinnedToCore(GetDataTask, "GetDataTask", 3000, NULL, 1, &GetDataTaskHandle, 0);
    xTaskCreatePinnedToCore(WiFiTelemetryTask, "WiFiTelemetryTask", 4000, NULL, 1, &WiFiTelemetryTaskHandle, 0);
    xTaskCreatePinnedToCore(readGPSTask, "ReadGPSTask", 3000, NULL, 1, &GPSTaskHandle, 1);
    xTaskCreatePinnedToCore(SDWriteTask, "SDWriteTask", 4000, NULL, 1, &SDWriteTaskHandle, 1);
=======
    xTaskCreatePinnedToCore(GetDataTask, "GetDataTask", 3000, NULL, 1, &GetDataTaskHandle, pro_cpu);
    xTaskCreatePinnedToCore(WiFiTelemetryTask, "WiFiTelemetryTask", 4000, NULL, 1, &WiFiTelemetryTaskHandle, pro_cpu);
    xTaskCreatePinnedToCore(readGPSTask, "ReadGPSTask", 3000, NULL, 1, &GPSTaskHandle, app_cpu);
    xTaskCreatePinnedToCore(SDWriteTask, "SDWriteTask", 4000, NULL, 1, &SDWriteTaskHandle, app_cpu);
>>>>>>> mqtt

    // Delete setup and loop tasks
    vTaskDelete(NULL);
}
void loop()
{
}