
// #include <Arduino.h>
// /*
//  * Check brownout issues to prevent ESP32 from re-booting unexpectedly
//  */

// #include "../include/functions.h"
// #include "../include/kalmanfilter.h"
// #include "../include/checkState.h"
// #include "../include/logdata.h"
// #include "../include/readsensors.h"
// #include "../include/transmitlora.h"
// #include "../include/defs.h"
// #include <SPI.h>

// // uninitalised pointers to SPI objects
// SPIClass *hspi = NULL;

// int counter = 0;
// struct LogData ld;

// struct LogData readData()
// {
//     struct SensorReadings readings;
//     struct FilteredValues filtered_values;
//     struct GPSReadings gpsReadings;
//     struct LogData ld;

//     readings = get_readings();

//     // TODO: get rid of GPS readings totally since it is blocking
//     gpsReadings = get_gps_readings();

//     filtered_values = kalmanUpdate(readings.altitude, readings.ay);

//     state = checkState(filtered_values.displacement, filtered_values.velocity, counter, state);

//     ld = formart_data(readings, filtered_values, gpsReadings);
//     ld.state = state;

//     return ld;
// }

// void setup()
// {
//     Serial.begin(BAUD_RATE);
//     hspi = new SPIClass(HSPI);
//     hspi->begin();

//     // set up slave select pins as outputs as the Arduino API
//     pinMode(LORA_CS_PIN, OUTPUT);   // HSPI SS for use by LORA
//     pinMode(SDCARD_CS_PIN, OUTPUT); // VSPI SS for use by SDCARD

//     init_components();

//     // for oscilloscope test
//     pinMode(33, OUTPUT);

//     SPIClass &spi = *hspi;
//     // setUpLoraOnBoard(spi);
// }

// void loop()
// {
//     digitalWrite(33, !digitalRead(33));
//     // char message = 'B';
//     // char *messagepointer = &message;
//     // appendFile(messagepointer);
//     readSD();

//     // ld = readData();
//     //  ld.counter = counter;
//     //  sendTelemetryLora(ld);
//     //  counter++;
// }
