/*

  FYP Robot Main Controller Code

  Description:
  This code is for the main low-level controller of FYP Robot, which is a 4-wheel skid-steering drive robot.
  The microcontroller used is ESP32-WROOM-32U.

  NOTE:
  1) Please use ESP32 board version 2.0.17 instead of 3.0.0 as the latest version appears to have a missing file, causing CAN bus code to not compile successfully

  Last updated: 30 May 2024
*/

// ||==============================||
// ||          LIBRARIES           ||
// ||==============================||
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <ESP32-TWAI-CAN.hpp>
#include <FastLED.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// ||==============================||
// ||          DEFINITIONS         ||
// ||==============================||
// CAN bus pins
#define RX 23  // Connect to RX Pin of the CAN module
#define TX 32  // connect to TX Pin of the CAN module

// Addressable LEDs pin and quantity
#define LED_NUM 32
#define LED_PIN 27

#define flLEDPin 12  // for status
#define frLEDPin 2  // for status
#define blLEDPin 13  // for status
#define brLEDPin 15  // for status

// GPS pins
#define RXPin 18
#define TXPin 19
#define GPSBaud 9600

// Time constants
#define RESET_TIMEFRAME 1000
#define DC_TIMEFRAME 1000
#define SERIAL_TIMEFRAME 100
#define IMU_TIMEFRAME 100
#define LED_TIMEFRAME 100
#define CAN_TIMEFRAME 50 // 50 ms
#define LED_BLINK_DELAY_SLOW 100  // 100 ms

// DC motor constants
#define noOfMotors 4
#define motorMaxPWM 255
#define motorMinPWM -motorMaxPWM
#define wheelSeparation 0.35  // Measured from left side of wheels to right side of wheels in meters

// ||==============================||
// ||           VARIABLES          ||
// ||==============================||
// ESP-NOW
enum deviceID { BASE_TELEOP, BASE_TELEM }; // Enumeration (to differentiate between which controller board sends out what data)
uint8_t chan = 5; // <======= NOTE! Check if the selected ESP-NOW channel is quiet or not


// CAN bus
// CAN message ID, used for differentiating the type of driving mode in this case
// 1 - Open loop control
// 2 - Closed loop control
int msgID = 0x0001;  


// Addressable LEDs
enum LED_STATUS {OFFLINE, MANUAL, AUTO, WAIT, RUNNING, STOP, PERMIT, AUX2, AUX3}; // Show different status of robot through the LEDs
int ledStatusCode = 0;


// Serial parsing
// Character arrays to hold 5 arguments from the laptop
char argv1[16];
char argv2[16];
char argv3[16];
char argv4[16];
char argv5[16];
char argv6[16];

// The arguments converted to floats and long integers
float arg1;
float arg2;
float arg3;
int arg4;
long arg5;
long arg6;


// Flags to indicate certain operations
bool runThis = false;      // Allow for immediate switching to new serial commands (when new commands are entered)
bool runOnce = false;      // Allow for certain parts of serial code to execute once only
bool ledState = false;     // default is off
bool autoOrManual = true;  // default is auto mode
bool resetOdom = false;
bool wasSwitchedToManual = false;
bool waitForKey = false;

// Motor speed variables
// "P" stands for PWM
// "D" stands for DIR
int FLW = 0;
int FRW = 0;
int BLW = 0;
int BRW = 0;
unsigned int PFLW = 0;
unsigned int PFRW = 0;
unsigned int PBLW = 0;
unsigned int PBRW = 0;
unsigned int DFLW = 0;
unsigned int DFRW = 0;
unsigned int DBLW = 0;
unsigned int DBRW = 0;

float measuredRPM[noOfMotors] = { 0, 0, 0, 0 };

// translation and rotation variables
float x = 0;
float y = 0;
float w = 0;

// IMU data
float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

// gps coordinates
float gpsLat = 8888.0;
float gpsLong = 8888.0;


// variables used for serial data parsing
int indexCmd = 0;  // index variable for array
int arg = 1;       // temporary storage of arguments
char chr;          // store incoming serial data

// for non-blocking delay
unsigned long previousResetMillis = 0;
unsigned long dcTimeoutMillis = 0;
unsigned long prevContinueMillis = 0;
unsigned long prevSerialMillis = 0;
unsigned long prevIMUMillis = 0;
unsigned long prevLEDStatusMillis = 0;
unsigned long prevCANMillis = 0;

// Structure example to send data
// Must match the receiver structure
typedef struct ESP32TELE {
  
  // which ESP32 board are we receiving data from?
  int deviceID;

  // ==================
  //   TELEOPERATION
  // ==================

  // base
  float xPos, yPos, wPos;
  bool autoOrManual, resetOdom, waitKey;

  // ==============
  //   TELEMETRY
  // ==============

  // base
  float targetRPM[noOfMotors];
  float measuredRPM[noOfMotors];

  // ===================
  //   LIVE PID TUNING
  // ===================

  float Kp[noOfMotors];
  float Ki[noOfMotors];
  float Kd[noOfMotors];

} ESP32TELE;


ESP32TELE base; // ESP-NOW message structure object
esp_now_peer_info_t peerInfo;
CRGB leds[LED_NUM];
TinyGPSPlus gps;  // The TinyGPSPlus object
SoftwareSerial gpsSerial(RXPin, TXPin);  // The serial connection to the GPS device
CanFrame txFrame = { 0 };
CanFrame rxFrame;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

// ||==============================||
// ||           FUNCTIONS          ||
// ||==============================||

//====================== CALLBACK FUNCTIONS ==========================

// callback function that will be executed when data is received, passes all values from the controller to the respective variables
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  dcTimeoutMillis = millis();
  memcpy(&base, incomingData, sizeof(base));

  if (base.deviceID == BASE_TELEOP){
    x = base.xPos;
    y = base.yPos;
    w = base.wPos;
    autoOrManual = base.autoOrManual;
    resetOdom = base.resetOdom;
    waitForKey = base.waitKey;
  }
  else if (base.deviceID == BASE_TELEM){
    measuredRPM[0] = base.measuredRPM[0];
    measuredRPM[1] = base.measuredRPM[1];
    measuredRPM[2] = base.measuredRPM[2];
    measuredRPM[3] = base.measuredRPM[3];
  }

}

//====================== TASK FUNCTIONS ==========================

// Reset the variables related to command parameters parsing and execution when no serial data has been received for a set period
void resetCommand() {
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  memset(argv3, 0, sizeof(argv3));
  memset(argv4, 0, sizeof(argv4));
  memset(argv5, 0, sizeof(argv5));
  memset(argv6, 0, sizeof(argv6));
  
  ledStatusCode = OFFLINE;
  arg1 = 0;
  arg2 = 0;
  arg3 = 0;
  arg4 = 0;
  arg5 = 0;
  arg6 = 0;
  arg = 1;
  indexCmd = 0;
  runOnce = false;
  runThis = false;
  FLW = 0;
  FRW = 0;
  BLW = 0;
  BRW = 0;
  PFLW = 0;
  PFRW = 0;
  PBLW = 0;
  PBRW = 0;
  DFLW = 0;
  DFRW = 0;
  DBLW = 0;
  DBRW = 0;
  x = 0;
  y = 0;
  w = 0;
  msgID = 0;
}


// Send data through serial from ESP32 microcontroller to laptop
void sendDataToPC() {
  Serial.print(autoOrManual);
  Serial.print(":");
  Serial.print(resetOdom);
  Serial.print(":");
  Serial.print(waitForKey);
  Serial.print(":");
  Serial.print(gpsLat);
  Serial.print("/");
  Serial.print(gpsLong);
  Serial.print(":");
  Serial.print(roll);
  Serial.print("/");
  Serial.print(pitch);
  Serial.print("/");
  Serial.println(yaw);
  Serial.print(":");
  Serial.print(measuredRPM[0]);
  Serial.print("/");
  Serial.print(measuredRPM[1]);
  Serial.print("/");
  Serial.print(measuredRPM[2]);
  Serial.print("/");
  Serial.println(measuredRPM[3]);
}

// Mecanum robot kinematics
void skidSteeringKinematics(float yInput, float wInput, int* FLWOut, int* FRWOut, int* BLWOut, int* BRWOut) {
  // Mecanum wheel robot kinematics code
  // Take note that with the current wiring
  // the formula may be different from what you have seen online
  *FLWOut = constrain((yInput - wheelSeparation * wInput) * motorMaxPWM, motorMinPWM, motorMaxPWM);
  *FRWOut = constrain((-yInput - wheelSeparation * wInput) * motorMaxPWM, motorMinPWM, motorMaxPWM);
  *BLWOut = constrain((yInput - wheelSeparation * wInput) * motorMaxPWM, motorMinPWM, motorMaxPWM);
  *BRWOut = constrain((-yInput - wheelSeparation * wInput) * motorMaxPWM, motorMinPWM, motorMaxPWM);

  analogWrite(flLEDPin, abs(*FLWOut));
  analogWrite(frLEDPin, abs(*FRWOut));
  analogWrite(blLEDPin, abs(*BLWOut));
  analogWrite(brLEDPin, abs(*BRWOut));

}


// Show the robot status through the RGB LEDs
void ledStatus (unsigned int status){
  unsigned long currMillis = millis();
  static unsigned int lastStatus = 0;
  static int ledNo = 0;
  static int sequence = 0;

  switch(status){
    case OFFLINE:
      if (currMillis - prevLEDStatusMillis >= LED_BLINK_DELAY_SLOW && sequence == 0){
        if (ledNo < LED_NUM){
          leds[ledNo] = CRGB(70, 0, 0);
          ledNo++;
        }
        else{
          ledNo = 0;
          sequence = 1;
          prevLEDStatusMillis = currMillis;      
        }
      }
      else if (currMillis - prevLEDStatusMillis >= LED_BLINK_DELAY_SLOW && sequence == 1){
        if (ledNo < LED_NUM){
          leds[ledNo] = CRGB(0, 0, 0);
          ledNo++;
        }
        else{
          ledNo = 0;
          sequence = 0;
          prevLEDStatusMillis = currMillis;      
        }
      }
      break;
    case MANUAL:
      if (ledNo < LED_NUM){
        leds[ledNo] = CRGB(50, 25, 0);
        ledNo++;
      }
      else{
        ledNo = 0;
      }
      break;
    case AUTO:
      if (ledNo < LED_NUM){
        leds[ledNo] = CRGB(50, 50, 0);
        ledNo++;
      }
      else{
        ledNo = 0;
      }
      break;
    case WAIT:
      if (ledNo < LED_NUM){
        leds[ledNo] = CRGB(50, 10, 0);
        ledNo++;
      }
      else{
        ledNo = 0;
      }
      break;
    case RUNNING:
      if (currMillis - prevLEDStatusMillis >= LED_BLINK_DELAY_SLOW && sequence == 0){
        if (ledNo < LED_NUM){
          leds[ledNo] = CRGB(0, 50, 0);
          ledNo++;
        }
        else{
          ledNo = 0;
          sequence = 1;
          prevLEDStatusMillis = currMillis;      
        }
      }
      else if (currMillis - prevLEDStatusMillis >= LED_BLINK_DELAY_SLOW && sequence == 1){
        if (ledNo < LED_NUM){
          leds[ledNo] = CRGB(0, 0, 0);
          ledNo++;
        }
        else{
          ledNo = 0;
          sequence = 0;
          prevLEDStatusMillis = currMillis;      
        }
      }
      break;
    case STOP:
      if (ledNo < LED_NUM){
        leds[ledNo] = CRGB(50, 0, 0);
        ledNo++;
      }
      else{
        ledNo = 0;
      }
      break;
    case PERMIT:
      if (ledNo < LED_NUM){
        leds[ledNo] = CRGB(0, 50, 0);
        ledNo++;
      }
      else{
        ledNo = 0;
      }
      break;
    default:
      if (ledNo < LED_NUM){
        leds[ledNo] = CRGB(50, 50, 50);
        ledNo++;
      }
      else{
        ledNo = 0;
      }  
      break;    
  }    

  if (ledNo == 0){
    FastLED.show();    
  }
}

void getIMUData(){
  sensors_event_t event; 
  bno.getEvent(&event);

  roll = event.orientation.z;
  pitch = event.orientation.y;
  yaw = event.orientation.x;

}

// execute the serial input command along with the specified argument values
void runAuto() {
  unsigned long currentMillis = millis();

  arg1 = atof(argv1);  // x
  arg2 = atof(argv2);  // y
  arg3 = atof(argv3);  // w
  arg4 = atoi(argv4);  // RGB LED Status
  arg5 = atoi(argv5);  // blank for other uses
  arg6 = atoi(argv6);  // blank for other uses

  // Closed-loop motion
  // left and right motors, accept 2 arguments (left speed & right speed, respectively,
  // with -ve sign indicating opposite direction)
  if (!runOnce) {
    //msgID = 1;  // uncomment this to choose open-loop motion
    msgID = 2;  // uncomment this to choose closed-loop motion
    previousResetMillis = currentMillis;
    runOnce = true;
  }

  // pass the motion values
  x = arg1;
  y = arg2;
  w = arg3;

  // calculate velocities and directions of each wheel based on the input
  skidSteeringKinematics(y, w, &FLW, &FRW, &BLW, &BRW);

  // preparing the CAN message payload
  PFLW = abs(FLW);
  PFRW = abs(FRW);
  PBLW = abs(BLW);
  PBRW = abs(BRW);
  DFLW = ((FLW < 0) ? 0 : 1);
  DFRW = ((FRW < 0) ? 0 : 1);
  DBLW = ((BLW < 0) ? 0 : 1);
  DBRW = ((BRW < 0) ? 0 : 1);

  ledStatusCode = arg4;

  // reset the parameters and commands after a preset timeout 
  if (currentMillis - previousResetMillis >= RESET_TIMEFRAME) {
    resetCommand();
  }
}


// |=======================================|
// |                                       |
// |            SETUP FUNCTION             |
// |                                       |
// |=======================================|
void setup() {

  // ==========
  //    GPIO 
  // ==========

  pinMode(flLEDPin, OUTPUT);
  pinMode(frLEDPin, OUTPUT);
  pinMode(blLEDPin, OUTPUT);
  pinMode(brLEDPin, OUTPUT);

  digitalWrite(flLEDPin, LOW);
  digitalWrite(frLEDPin, LOW);
  digitalWrite(blLEDPin, LOW);
  digitalWrite(brLEDPin, LOW);

  FastLED.addLeds<WS2812,LED_PIN,GRB>(leds,LED_NUM);

  // ============================
  //     SERIAL COMMUNICATION
  // ============================
  Serial.begin(115200);  // Begin the serial communication at 115200 baud rate
  gpsSerial.begin(GPSBaud);

  // ==============
  //    CAN Bus 
  // ==============
  // or override everything in one command;
  // It is also safe to use .begin() without .end() as it calls it internally
  if(ESP32Can.begin(ESP32Can.convertSpeed(500), TX, RX, 10, 10)) {
      Serial.println("CAN bus started!");
  } else {
      Serial.println("CAN bus failed!");
  }

  // ================
  //    IMU SENSOR
  // ================

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
    
  bno.setExtCrystalUse(true);


  // ==============
  //    ESP-NOW 
  // ==============
  WiFi.mode(WIFI_STA);  // Set device as a Wi-Fi Station

  // Set the WiFi channel to channel 11
  // Available channels are 0 to 14
  // Channel 0 will auto select the channel the station or softAP is on
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(chan, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);



  // indicate start of operation
  digitalWrite(flLEDPin, HIGH);
  delay(100);
  digitalWrite(flLEDPin, LOW);
  digitalWrite(frLEDPin, HIGH);
  delay(100);
  digitalWrite(frLEDPin, LOW);
  digitalWrite(blLEDPin, HIGH);
  delay(100);
  digitalWrite(blLEDPin, LOW);
  digitalWrite(brLEDPin, HIGH);
  delay(100);
  digitalWrite(blLEDPin, LOW);

}



// |=======================================|
// |                                       |
// |             LOOP FUNCTION             |
// |                                       |
// |=======================================|
void loop() {
  unsigned long currMillis = millis();

  // Safety feature where if we do not receive any data from controller
  // Reset all command velocities to zero so that the robot ceases to move
  if (currMillis - dcTimeoutMillis >= DC_TIMEFRAME) {
    x = 0;
    y = 0;
    w = 0;
  }

  // ========================================
  //   AUTO CONTROL - Serial commands only
  // ========================================
  if (autoOrManual) {
    if (wasSwitchedToManual){
      wasSwitchedToManual = false;
      ledStatusCode = OFFLINE;
    }
    //Serial.println("running auto");
    while (Serial.available() > 0) {
      // Read the next character
      chr = Serial.read();

      // If a new command is being input, stop running current command, clear
      // the current commands so that new ones can be written into them
      if (runThis) {
        runThis = false;
        resetCommand();
      }

      // Terminate a command with a CR (Carriage Return)
      if (chr == 13) {
        if (arg == 1) argv1[indexCmd] = NULL;
        else if (arg == 2) argv2[indexCmd] = NULL;
        else if (arg == 3) argv3[indexCmd] = NULL;
        else if (arg == 4) argv4[indexCmd] = NULL;
        else if (arg == 5) argv5[indexCmd] = NULL;
        else if (arg == 6) argv6[indexCmd] = NULL;
        runThis = true;
      }

      // Use spaces to delimit parts of the command
      else if (chr == ' ') {
        // Step through the arguments
        if (arg == 1) {
          argv1[indexCmd] = NULL;
          arg = 2;
          indexCmd = 0;
        } else if (arg == 2) {
          argv2[indexCmd] = NULL;
          arg = 3;
          indexCmd = 0;
        } else if (arg == 3) {
          argv3[indexCmd] = NULL;
          arg = 4;
          indexCmd = 0;
        } else if (arg == 4) {
          argv4[indexCmd] = NULL;
          arg = 5;
          indexCmd = 0;
        } else if (arg == 5) {
          argv5[indexCmd] = NULL;
          arg = 6;
          indexCmd = 0;
        }
        continue;
      }

      // record the serial command
      else {
        if (arg == 1) {
          // Subsequent arguments can be more than one character
          argv1[indexCmd] = chr;
          indexCmd++;
        } else if (arg == 2) {
          argv2[indexCmd] = chr;
          indexCmd++;
        } else if (arg == 3) {
          argv3[indexCmd] = chr;
          indexCmd++;
        } else if (arg == 4) {
          argv4[indexCmd] = chr;
          indexCmd++;
        } else if (arg == 5) {
          argv5[indexCmd] = chr;
          indexCmd++;
        } else if (arg == 6) {
          argv6[indexCmd] = chr;
          indexCmd++;
        }
      }
    }

    // Run the command
    if (runThis) {
      runAuto();
    }
    // if there are no serial commands received, then it will keep sending this to prevent
    // the motor from running right away by ensuring there is always valid CAN message
    // being transmitted
    else{
      msgID = 2;
      PFLW = 0;         // Left wheel speed
      PFRW = 0;         // Right wheel speed
      PBLW = 0;         // Left wheel direction
      PBRW = 0;         // Left wheel speed
      DFLW = 0;         // Left wheel speed
      DFRW = 0;         // Right wheel speed
      DBLW = 0;         // Left wheel direction
      DBRW = 0;         // Left wheel speed
    }
  }

  // ===========================================
  //    MANUAL CONTROL - Remote commands only
  // ===========================================
  else {
    ledStatusCode = MANUAL;
    wasSwitchedToManual = true;
    skidSteeringKinematics(y, w, &FLW, &FRW, &BLW, &BRW);

    msgID = 0x0001;
    PFLW = abs(FLW);
    PFRW = abs(FRW);
    PBLW = abs(BLW);
    PBRW = abs(BRW);

    DFLW = ((FLW < 0) ? 0 : 1);
    DFRW = ((FRW < 0) ? 0 : 1);
    DBLW = ((BLW < 0) ? 0 : 1);
    DBRW = ((BRW < 0) ? 0 : 1);

    // uncomment this to get command velocity output
    // Serial.print(PFLW);
    // Serial.print(" ");
    // Serial.print(PFRW);
    // Serial.print(" ");
    // Serial.print(PBLW);
    // Serial.print(" ");
    // Serial.print(PBRW);
    // Serial.print(" | ");
    // Serial.print(DFLW);
    // Serial.print(" ");
    // Serial.print(DFRW);
    // Serial.print(" ");
    // Serial.print(DBLW);
    // Serial.print(" ");
    // Serial.println(DBRW);
    
  }

  // DO NOT PUT NON-BLOCKING DELAY FOR THIS, LET IT RUN AS FAST AS IT CAN
  txFrame.identifier = (msgID);
  txFrame.extd = 0;
  txFrame.data_length_code = 8;
  txFrame.data[0] = (PFLW);         // Front left wheel speed
  txFrame.data[1] = (PFRW);         // Front right wheel speed
  txFrame.data[2] = (PBLW);         // Back left wheel speed
  txFrame.data[3] = (PBRW);         // Back right wheel speed
  txFrame.data[4] = (DFLW);         // Front left wheel direction
  txFrame.data[5] = (DFRW);         // Front right wheel direction
  txFrame.data[6] = (DBLW);         // Back left wheel direction
  txFrame.data[7] = (DBRW);         // Back right wheel direction

  ESP32Can.writeFrame(txFrame);  // timeout defaults to 1 ms


  // Send odometry and other data to PC via serial
  if (currMillis - prevSerialMillis >= SERIAL_TIMEFRAME){
    sendDataToPC();    
    prevSerialMillis = currMillis;
  }

    // Send odometry and other data to PC via serial
  if (currMillis - prevIMUMillis >= IMU_TIMEFRAME){
    getIMUData();    
    prevIMUMillis = currMillis;
  }

  // Change the led colour of the robot depending on the modes
  ledStatus(ledStatusCode);    


}