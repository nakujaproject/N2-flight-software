#ifndef TRANSMITWIFI_H
#define TRANSMITWIFI_H

#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <AsyncMqttClient.h>
#include <WiFi.h>
#include "defs.h"
#include "functions.h"

char *printTransmitMessageWiFi(SendValues sv)
{
  // The assigned size is calculated to fit the string
  char *message = (char *)pvPortMalloc(100);

  if (!message)
    return NULL;

  snprintf(message, 100, "{\"timestamp\":%lld,\"state\":%d,\"altitude\":%.3f,\"latitude\":%.8f,\"longitude\":%.8f}\n", sv.timeStamp, sv.state, sv.altitude,sv.latitude,sv.longitude);
  //snprintf(message, 60, "{\"timestamp\":%lld,\"state\":%d,\"altitude\":%.3f}\n", sv.timeStamp, sv.state, sv.altitude);
  return message;
}

//************METHOD I using UDP no server***********************

void sendTelemetryWiFi(SendValues sv[5], AsyncMqttClient mqttClient)
{
  // send a broadcast to all clients in the local network
  Udp.beginPacket({192, 168, 4, 255}, UDP_PORT);

  // TODO: optimize size
  char combinedMessage[500];
  strcpy(combinedMessage, "");
  for (int i = 0; i < 5; i++)
  {

    char *message = printTransmitMessageWiFi(sv[i]);
    strcat(combinedMessage, message);
    vPortFree(message);
  }
  debugln(combinedMessage);

  // Publish an MQTT message on topic esp/bme680/temperature
  uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TOPIC, 1, true, String(combinedMessage).c_str());
  debugf("Publishing packetId: %i", packetIdPub1);


  Udp.print(combinedMessage);
  if (Udp.endPacket())
  {
    debugln("Sent packet");
  }
}

void processInputCommandWiFi()
{
  char command = Udp.read();
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

  case '5':
    // eject parachute
    ejection();
    break;

  default:; // pass
  }
}

void handleWiFi(SendValues sv[5], AsyncMqttClient mqttClient)
{

  // check for incoming data stream
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    processInputCommandWiFi();
  }
  else
  {
    sendTelemetryWiFi(sv, mqttClient);
  }
}

//************METHOD II using SERVER***********************

void serveData()
{
  struct SendValues sv = {0};
  // TODO fetch send values
  DynamicJsonDocument doc(96);
  doc["timestamp"] = sv.timeStamp;
  doc["altitude"] = sv.altitude;
  doc["state"] = sv.state;
  doc["longitude"] = sv.longitude;
  doc["latitude"] = sv.latitude;

  String json;
  serializeJson(doc, json);

  server.send(200, "application/json", json);
}

void setupServer()
{

  // handle HTTP REQUESTS
  server.on("/data", serveData);
  // start server
  server.begin();
  debugln("\nHTTP Server started...");
}

#endif