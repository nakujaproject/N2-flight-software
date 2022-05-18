#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "defs.h"
#include "readsensors.h"

void ejection();
void ejectionTimerCallback(TimerHandle_t ejectionTimerHandle);

Adafruit_BMP085 barometer; 
Adafruit_MPU6050 accelerometer;
WiFiUDP Udp;

using namespace BLA;
/* 
* ==================== FUNCTION DECLARATION ==================== 
*/
void initializeComponents();
String getSensorReadings();
void printToSerial();
void connectToWifi();
void appendToFile(fs::FS, const char*, const char*);
void logToSD(String);
void detectLiftoff();
void detectApogee();
void deployParachute();
void createAcessPoint();
void serveData(int counter, float altitude, float ax, float ay, float az, float gx, float gy, float gz, float s, float v, float a, int currentState, float longitude, float latitude);
String request_server_data(const char* server_name);


/*
 * ==================== FUNCTION DEFINITIONS ====================
 */
void initializeComponents(){
  /*
   * This function checks and initializes all the sensors and components 
   * Params: none
   * Return: void
   */

  Serial.begin(BAUD_RATE);
 
  // Initialize BMP pressure sensor
  Serial.println("Barometer Check...");
  if(!barometer.begin()){
    Serial.println("Barometer not found...");
    while (1){
      delay(SHORT_DELAY);
    }
  }
  Serial.println("Barometer OK..."); // todo : log

  // Initialize MPU6050 accelerometer
  Serial.println("Accelerometer Check...");
  if(!accelerometer.begin()){
    Serial.println("Accelerometer not found...");
    while (1) {
      delay(SHORT_DELAY);
    }
  }
  
  Serial.println("Accelerometer OK...");

  // Initialize SD card
  Serial.println("SD card Check...");
  if(!SD.begin()){
    Serial.println("SD card not found...");
    return;
  } 

  // check mounted card type
  uint8_t card_type = SD.cardType();
  if (card_type == CARD_NONE){
    Serial.println("No SD card attached..."); // todo: log
    return;
  }
  
  // SD card exists
  Serial.println("SD card OK...");
  
}
 
String getSensorReadings(){
  sensors_event_t a, g, temp;
  accelerometer.getEvent(&a, &g, &temp);

  // read altitude
  altitude = barometer.readAltitude(SEA_LEVEL_PRESSURE_HPA * 100);

  // convert altitude to unsigned integer
  // altitude = (int)altitude;

  // read x axis acceleration
  ax = a.acceleration.x;

  // read y axis acceleration
  ay = a.acceleration.y;

  // read z axis acceleration
  az = a.acceleration.z;

  // append data to sata_message
  data_message = String("Altitude: ") + String(altitude) + ", " + String("Acc-x: ") + String(ax) + ", " + String("Acc-y: ") + String(ay) + ", " + String("Acc-z: ") + String(az) + "\n";
  return data_message;
}

void kalmanUpdate(){
      //Measurement matrix
    BLA::Matrix<2, 1> Z = {altitude,
                           ay};
    //Predicted state estimate
    BLA::Matrix<3, 1> x_hat_minus = A * x_hat;
    //Predicted estimate covariance
    BLA::Matrix<3, 3> P_minus = A * P * (~A) + Q;
    //Kalman gain
    BLA::Matrix<3, 2> K = P_minus * (~H) * ((H * P_minus * (~H) + R)).Inverse();
    //Measurement residual
    Y = Z - (H * x_hat_minus);
    //Updated state estimate
    x_hat = x_hat_minus + K * Y;
    //Updated estimate covariance
    P = (I - K * H) * P_minus;
    Y = Z - (H * x_hat_minus);

    s = x_hat(0);
    v = x_hat(1);
    a = x_hat(2);
}

void printToSerial(){
  //Data = String(altitude) + "," + String(ax) + "," + String(ay) + "," + String(az);
  Serial.print("Altitude:"); Serial.print(altitude); Serial.println();
  Serial.print("ax: "); Serial.print(ax); Serial.println();
  Serial.print("ay: "); Serial.print(ay); Serial.println();
  Serial.print("az: "); Serial.print(az); Serial.println();
  Serial.println();

  delay(100);
}

// void connectToWifi(){
//   WiFi.begin(ssid, key);

//   // check if connection okay
//   while(WiFi.status() != WL_CONNECTED){
//     Serial.println("Establishing connection to WiFi...");
//     delay(SHORT_DELAY);
//   }

//   // connection okay
//   Serial.print("Connection established to "); Serial.print(ssid);
//   Serial.println(WiFi.localIP()); 
  
// }

void logToSD(String data){
	/*
	* ==================== Write flight data to SD card =====================
  MICROSD MODULE PINOUT
  3V3 -> 3v3
  CS -> GPIO5
  MOSI -> GPIO23
  CLK -> GPIO18
  MISO -> GPIO19
  GND -> GND
	*/

  // convert data message from string to c-type string for use with appendToFile function
  const char* data_message_char = data_message.c_str();

  // todo: figure out which pins are on the actual PCB computer
  appendToFile(SD, "/flight_data.txt", data_message_char);

}

void appendToFile(fs::FS &fs, const char* path, const char* data){
  // the file to write to must first be created in the SD card using a PC
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for reading.."); // todo: log
    return;
  } else if(file.print(data)){
    Serial.println("Message appended"); // todo: log
  }else{
    Serial.println("Failed writing to file");  // todo: log
  }
}

void detectLiftoff(){
  /*
	* ==================== Detect when the rocket has launched =====================
  * If the current altitude is greater than the previous by 50cm
  * report lift-off
	*/
  current_altitude = barometer.readAltitude(SEA_LEVEL_PRESSURE_HPA * 100);

  if(is_lift_off == false){
    Serial.println("0: Waiting for lift-off..."); // todo: transmit to ground
    if((current_altitude - CURRENT_ALTITUDE) > LIFT_OFF_DEVIATION){   
      

      Serial.println(current_altitude);

      // lift off detected
      Serial.print("Lift Off!"); // log this to some file
      is_lift_off = true;
  }
  }
}

void detectApogee(){
  /*
  * Detect point where velocity is at 0
  * v = u + at
  */
  
  Serial.println("(1) Waiting for apogee...");

  using index_t = decltype(altitude_buffer)::index_t; // ensure right type for the index variable

  // check for apogee every 500 ms (600ms is an arbitrary value. can be changed) 
  if(millis() - tm > 500){
    // insert the most recent value into the circular buffer
    altitude_buffer.push(altitude);
    
    // check for apogee
    if(altitude_buffer[altitude_buffer.size()] < altitude_buffer[altitude_buffer.size() - 1]){
      // if the most recent altitude value is less than the previous value, 
      // the rocket is descending. Apogee reached
      // call the function to deploy parachute
      
    }else{
      // rocket still in ascent
      // apogee not reached yet
      Serial.println("Waiting for apogee...");
      // todo: transmit this state to ground station
    }    
  }
}

// get_base_altitude Finds the average of the current altitude from 1000 readings
float get_base_altitude()
{
  float altitude = 0;
  SensorReadings readings;
  for (int i = 0; i < 10; i++)
  {
    readings = get_readings();
    debugln(readings.altitude);
    altitude = altitude + readings.altitude;

    //TODO: why must we delay here?
  }
  altitude = altitude / 10.0;
  debugln(altitude);
  delay(5000);
  return altitude;
}
#endif