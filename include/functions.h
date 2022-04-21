#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
#include <BasicLinearAlgebra.h>
#include <WiFi.h>
#include <SD.h>
#include<CircularBuffer.h>
#include <HTTPClient.h>
#include <WiFiUdp.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>

#include "global_variables.h" // header file containing variables
#include "defs.h" // header file containing constants

#define SD_CS 5

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

Adafruit_BMP085 barometer; 
MPU6050 mpu;
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



int16_t accData[3], gyrData[3];

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2 

#define SDA 21
#define SCL 22

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in  
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

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
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin(SDA, SCL, 400000);
        //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT_PULLUP); 

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        //.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        //Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
    }
  
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
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // read altitude
  altitude = barometer.readAltitude(SEA_LEVEL_PRESSURE_HPA * 100);

  // convert altitude to unsigned integer
  // altitude = (int)altitude;

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

void createAccessPoint(){
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Setting Ground Station WiFi...");
  delay(1000);

  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(ssid, key);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("ssid:"); Serial.print(ssid); Serial.println();
  Serial.print("key:"); Serial.print(key); Serial.println();
  Serial.print("Rocket IP address: "); Serial.print(IP); Serial.println();
  
  server.begin();
}
//check state functions start
int checkPrelaunch(int s){
  // determines that the rocket is in prelaunch and hasn't taken off
  if(s==0){
  return 0;
  }
}
int checkInflight(int s,int t){
  //detects that the rocket is in flight
  if(s>5 && t>0 && t<3){
  return 1;
  }
}
int checkCoasting(int t,int v){
  //detects that burn out has occured and the rocket is coasting
  if (t>3 && v>1){
    return 2;
  }
  
}
int checkApogee(int v){
  //detects that apogee has been achieved and ejection of parachute should take place
  if(v>-1 && v<1){
    return 3;
  }
  
}
int checkDescent(int v,int s){
  //detects descent of the rocket after parachute ejection
  if(v<-1 && s>5){
    return 4;
  }
  
}
int checkGround(int v,int s){
  //detects landing of the rocket
  if(v==0 && s==0){
     return 5; 
  }
void serveData(
    int counter,
    float altitude,
    float ax,
    float ay,
    float az,
    float gx,
    float gy,
    float gz,
    float s,
    float v,
    float a,
    int currentState,
    float longitude,
    float latitude)
{
  Udp.beginPacket({192, 168, 4, 255}, PORT);

  // payload

  Udp.printf("Counter : %d \n Altitude : %.3f \n ax : %.3f \n ay : %.3f \n az  : %.3f \n gx  : %.3f \n gy  : %.3f \n gz  : %.3f \n s : %.3f \n v : %.3f \n a : %.3f \n Current State  : %d \n Longitude  : %.3f \n Latitude  : %.3f \n", counter, altitude, ax, ay, az, gx, gy, gz, s, v, a, currentState, longitude, latitude);

  if (!Udp.endPacket())
  {
    Serial.println("NOT SENT!");
  }
  else
  {
    Serial.println("SENT!!");
  }
}
}
// check state functions end

#endif