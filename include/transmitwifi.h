#ifndef TRANSMITWIFI_H
#define TRANSMITWIFI_H

#include <ArduinoJson.h>
#include <WiFi.h>
#include "defs.h"
#include "functions.h"

char *printTransmitMessageWiFi(SendValues sv)
{
  // The assigned size is calculated to fit the string
  char *message = (char *)pvPortMalloc(100);

  if (!message)
    return NULL;

  snprintf(message, 100, "{\"timestamp\":%lld,\"state\":%d,\"altitude\":%.3f,\"latitude\":%.8f,\"longitude\":%.8f}\n", sv.timeStamp, sv.state, sv.altitude, sv.latitude, sv.longitude);
  // snprintf(message, 60, "{\"timestamp\":%lld,\"state\":%d,\"altitude\":%.3f}\n", sv.timeStamp, sv.state, sv.altitude);
  return message;
}
void callback(char *topic, byte *message, unsigned int length)
{
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++)
  {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off".
  // Changes the output state according to the message
  if (String(topic) == "esp32/ejection")
  {
    Serial.print("Changing output to ");
    if (messageTemp == "on")
    {
      debugln("on");
      digitalWrite(EJECTION_PIN, HIGH);
    }
    else if (messageTemp == "off")
    {
      debugln("off");
      digitalWrite(EJECTION_PIN, LOW);
    }
  }
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client"))
    {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/ejection");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 50 milliseconds");
      // Wait 5 seconds before retrying
      delay(50);
    }
  }
}

void sendTelemetryWiFi(SendValues sv[5])
{
  float altitude = 0;
  int timestamp = 0;
  int state = 0;
  float longitude = 0;
  float latitude = 0;

  // TODO: optimize size
  char combinedMessage[500];
  strcpy(combinedMessage, "");
  for (int i = 0; i < 5; i++)
  {
    // publish altitude
    altitude = sv[i].altitude;
    char altstring[8];
    dtostrf(altitude, 1, 2, altstring);
    client.publish("esp32/altitude", altstring);

    // publish timestamp
    timestamp = sv[i].timeStamp;
    char alttimestamp[8];
    dtostrf(timestamp, 1, 2, alttimestamp);
    client.publish("esp32/timestamp", alttimestamp);
    //debugln(timestamp);

    // publish state
    state = sv[i].state;
    char altstate[8];
    dtostrf(state, 1, 2, altstate);
    client.publish("esp32/state", altstate);

    // publish longitude
    longitude = sv[i].longitude;
    char altlongitude[8];
    dtostrf(longitude, 1, 8, altlongitude);
    client.publish("esp32/longitude", altlongitude);

    // publish latitude
    latitude = sv[i].latitude;
    char altlatitude[8];
    dtostrf(latitude, 1, 8, altlatitude);
    client.publish("esp32/latitude", altlatitude);

    // debugln(latitude);
    // debugln(longitude);

  }
}

void handleWiFi(SendValues sv[5])
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();
  sendTelemetryWiFi(sv);
}

// //************METHOD I using UDP no server***********************

// void sendTelemetryWiFi(SendValues sv[5], AsyncMqttClient mqttClient)
// {
//   // send a broadcast to all clients in the local network
//   Udp.beginPacket({192, 168, 4, 255}, UDP_PORT);

//   // TODO: optimize size
//   char combinedMessage[500];
//   strcpy(combinedMessage, "");
//   for (int i = 0; i < 5; i++)
//   {

//     char *message = printTransmitMessageWiFi(sv[i]);
//     strcat(combinedMessage, message);
//     vPortFree(message);
//   }
//   debugln(combinedMessage);

//   // Publish an MQTT message on topic esp/bme680/temperature
//   uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TOPIC, 1, true, String(combinedMessage).c_str());
//   debugf("Publishing packetId: %i", packetIdPub1);

//   Udp.print(combinedMessage);
//   if (Udp.endPacket())
//   {
//     debugln("Sent packet");
//   }
// }

// void processInputCommandWiFi()
// {
//   char command = Udp.read();
//   switch (command)
//   {
//   case '0':
//     portENTER_CRITICAL(&mutex);
//     state = 0;
//     portEXIT_CRITICAL(&mutex);
//     break;
//   case '1':
//     portENTER_CRITICAL(&mutex);
//     state = 1;
//     portEXIT_CRITICAL(&mutex);
//     break;
//   case '2':
//     portENTER_CRITICAL(&mutex);
//     state = 2;
//     portEXIT_CRITICAL(&mutex);
//     break;
//   case '3':
//     portENTER_CRITICAL(&mutex);
//     state = 3;
//     portEXIT_CRITICAL(&mutex);
//     break;
//   case '4':
//     portENTER_CRITICAL(&mutex);
//     state = 4;
//     portEXIT_CRITICAL(&mutex);
//     break;

//   case '5':
//     // eject parachute
//     ejection();
//     break;

//   default:; // pass
//   }
// }

// void handleWiFi(SendValues sv[5], AsyncMqttClient mqttClient)
// {

//   // check for incoming data stream
//   int packetSize = Udp.parsePacket();
//   if (packetSize)
//   {
//     processInputCommandWiFi();
//   }
//   else
//   {
//     sendTelemetryWiFi(sv, mqttClient);
//   }
// }

// //************METHOD II using SERVER***********************

// void serveData()
// {
//   struct SendValues sv = {0};
//   // TODO fetch send values
//   DynamicJsonDocument doc(96);
//   doc["timestamp"] = sv.timeStamp;
//   doc["altitude"] = sv.altitude;
//   doc["state"] = sv.state;
//   doc["longitude"] = sv.longitude;
//   doc["latitude"] = sv.latitude;

//   String json;
//   serializeJson(doc, json);

//   server.send(200, "application/json", json);
// }

// void setupServer()
// {

//   // handle HTTP REQUESTS
//   server.on("/data", serveData);
//   // start server
//   server.begin();
//   debugln("\nHTTP Server started...");
// }

#endif