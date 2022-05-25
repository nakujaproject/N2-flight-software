#ifndef TRANSMITWIFI_H
#define TRANSMITWIFI_H

#include <WiFi.h>
#include "defs.h"
#include "functions.h"

<<<<<<< HEAD
char *printTransmitMessageWiFi(SendValues sv)
{
  // The assigned size is calculated to fit the string
  char *message = (char *)pvPortMalloc(100);


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


>>>>>>> mqtt

#endif