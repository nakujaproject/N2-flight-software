#ifndef GLOBALVARIABLES_H
#define GLOBALVARIABLES_H




/*
* =================== POSITION VARIABLES ====================
*/
float altitude, velocity, acceleration, kalmanAltitude;
float s, v, a;

int16_t ax, ay, az;
int16_t gx, gy, gz;

  
/*
* Wi-Fi connection variables
*/

static const char* ssid = "ground-station"; // change to credentials of the ground station WiFi
static const char* key = "password";

String sensor_data; // convert char to string
String data_message;

// set up data fetch interval variables
unsigned long previousMillis = 0;
const long interval = 2000;


AsyncWebServer server(80);


/*
* liftoff detection variables
*/
bool is_lift_off = false;
uint previous_altitude;
uint current_altitude;

/*
* apogee variables
*/
bool is_apogee = false;

// circular buffer creation
CircularBuffer<float, 5> altitude_buffer;

unsigned long tm;

#endif