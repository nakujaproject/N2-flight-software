#ifndef TRANSMITWIFI_H
#define TRANSMITWIFI_H

#include <WiFi.h>
#include "defs.h"
#include "functions.h"

void mqttCallback(char *topic, byte *message, unsigned int length)
{
  debug("Message arrived on topic: ");
  debug(topic);
  debug(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++)
  {
    debug((char)message[i]);
    messageTemp += (char)message[i];
  }
  debugln();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off".
  // Changes the output state according to the message
  if (String(topic) == "esp32/ejection")
  {
    debug("Changing output to ");
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

void setup_wifi()
{
  // Connect to a WiFi network
  debugln();
  debug("Connecting to ");
  debugln(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    debug(".");
  }

  debugln("");
  debugln("WiFi connected");
  debugln("IP address: ");
  debugln(WiFi.localIP());

  client.setServer(mqtt_server, MQQT_PORT);
  client.setCallback(mqttCallback);
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    debugln("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client"))
    {
      debugln("connected");
      // Subscribe
      client.subscribe("esp32/ejection");
    }
    else
    {
      debug("failed, rc=");
      debug(client.state());
      debugln(" try again in 50 milliseconds");
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

  for (int i = 0; i < 5; i++)
  { 

    //TODO: publish whole message
    // char mqttMessage [200];
    // sprintf(mqttMessage, "{\"timestamp\":%lld,\"altitude\":%.3f,\"state\":%d,\"longitude\":%.8f,\"latitude\":%.8f}", sv[i].timeStamp, sv[i].altitude, sv[i].state, sv[i].longitude, sv[i].latitude);
    // client.publish("esp32/message", mqttMessage);

    // publish altitude
    altitude = sv[i].altitude;
    char mqttAltitude[12];
    sprintf(mqttAltitude,"%.3f",altitude);
    client.publish("esp32/altitude", mqttAltitude);

    // publish timestamp
    timestamp = sv[i].timeStamp;
    char mqttTimestamp[12];
    sprintf(mqttTimestamp, "%d", timestamp);
    client.publish("esp32/timestamp", mqttTimestamp);

    // publish state
    state = sv[i].state;
    char mqttState[3];
    sprintf(mqttState, "%d", state);
    client.publish("esp32/state", mqttState);

    // publish longitude
    longitude = sv[i].longitude;
    char mqttLongitude[12];
    sprintf(mqttLongitude, "%.8f",longitude);
    client.publish("esp32/longitude", mqttLongitude);

    // publish latitude
    latitude = sv[i].latitude;
    char mqttLatitude[12];
    sprintf(mqttLatitude, "%.8f", latitude);
    client.publish("esp32/latitude", mqttLatitude);
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

#endif