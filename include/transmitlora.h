#ifndef TRANSMITLORA_H
#define TRANSMITLORA_H

#include <SPI.h>
#include <LoRa.h>
#include "defs.h"
#include "functions.h"
#include "readsensors.h"

char *printTransmitMessageLoRa(SendValues sv)
{
    // The assigned size is calculated to fit the string
    char *message = (char *)pvPortMalloc(60);

    if (!message)
        return NULL;

    snprintf(message, 60, "{\"timestamp\":%lld,\"state\":%d,\"altitude\":%.3f}\n", sv.timeStamp, sv.state, sv.altitude);
    return message;
}

void sendTelemetryLora(SendValues sv[5])
{
    LoRa.beginPacket();

    // TODO: optimize size
    char combinedMessage[300];
    strcpy(combinedMessage, "");
    for (int i = 0; i < 5; i++)
    {
        char *message = printTransmitMessageLoRa(sv[i]);
        strcat(combinedMessage, message);
        vPortFree(message);
    }
    // debugln(combinedMessage);
    LoRa.print(combinedMessage);
    // send packet
    LoRa.endPacket();
    // debugln("Sent Lora Done");
}

void processInputCommandLoRa()
{
    // received a packet
    if (LoRa.available() > 0)
    {
        char command = LoRa.read();
        switch (command)
        {
        case '0':
            portENTER_CRITICAL(&mutex);
            state = 0;
            portEXIT_CRITICAL(&mutex);
            break;
        case '1':
            portENTER_CRITICAL(&mutex);
            state = 1;
            portEXIT_CRITICAL(&mutex);
            break;
        case '2':
            portENTER_CRITICAL(&mutex);
            state = 2;
            portEXIT_CRITICAL(&mutex);
            break;
        case '3':
            portENTER_CRITICAL(&mutex);
            state = 3;
            portEXIT_CRITICAL(&mutex);
            break;
        case '4':
            portENTER_CRITICAL(&mutex);
            state = 4;
            portEXIT_CRITICAL(&mutex);
            break;
        default:
            // eject parachute
            ejection();
        }
    }
}

void handleLora(SendValues sv[5])
{

    // check for incoming data stream
    int packetSize = LoRa.parsePacket();
    if (packetSize)
    {
        processInputCommandLoRa();
    }
    else
    {
        sendTelemetryLora(sv);
    }
}

#endif
