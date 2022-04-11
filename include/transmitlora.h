#ifndef TRANSMITLORA_H
#define TRANSMITLORA_H

#include <SPI.h>
#include <LoRa.h>
#include "defs.h"
#include "readsensors.h"

const long freq = 868E6; // frequechy 868 MHz
const int SF = 9;        // spread factor
const long bw = 125E3;   // bandwidth 125 kHz

void setUpLoraOnBoard(SPIClass &spi)
{
    Serial.print("Setting up LoRa Sender...");
    LoRa.setPins(LORA_CS_PIN, RESET_LORA_PIN, IRQ_LORA_PIN); // set CS, reset, IRQ pin
    LoRa.setSPI(spi);
    Serial.print("Setting up LoRa Sender...");

    while (!LoRa.begin(freq))
    {
        Serial.print(".");
    }

    Serial.println();
    Serial.println("Successfully set up LoRa");

    LoRa.setSpreadingFactor(SF);
    // LoRa.setSignalBandwidth(bw);
    LoRa.setSyncWord(0xF3);
    Serial.print("Frequency ");
    Serial.print(freq);
    Serial.print(" Bandwidth ");
    Serial.print(bw);
    Serial.print(" SF ");
    Serial.println(SF);
}

void sendMessageLora(LogData ld)
{
    char message[256];
    sprintf(message, "{\"counter\":%d,\"sensor altitude\":%.3f,\"gps altitude\":%.3f,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f,\"filtered s\":%.3f,\"filtered v\":%.3f,\"filtered a\":%.3f,\"state\":%d,\"longitude\":%.5f,\"latitude\":%.5f,\"gps v\":%.3f,\"satellites\":%d}", ld.counter, ld.sensorAltitude, ld.gpsAltitude, ld.ax, ld.ay, ld.az, ld.gx, ld.gy, ld.gz, ld.filtered_s, ld.filtered_v, ld.filtered_a, ld.state, ld.longitude, ld.latitude, ld.gpsSpeed, ld.gpsSatellites);

    Serial.println(message);
    // send packet
    LoRa.beginPacket();
    LoRa.print(message);
    LoRa.endPacket();
}

#endif
