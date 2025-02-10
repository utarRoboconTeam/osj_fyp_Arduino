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
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>

// ||==============================||
// ||          DEFINITIONS         ||
// ||==============================||

// Touchscreen pins
#define XPT2046_IRQ 36   // T_IRQ
#define XPT2046_MOSI 32  // T_DIN
#define XPT2046_MISO 39  // T_OUT
#define XPT2046_CLK 25   // T_CLK
#define XPT2046_CS 33    // T_CS

// LED pins
#define redLEDPin 16
#define greenLEDPin 2

// button pins
#define upBtn 19
#define downBtn 18
#define leftBtn 5
#define rightBtn 17
#define enterBtn 22
#define escBtn 23


// Screen constants
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define FONT_SIZE 2

// Time constants
#define DC_TIMEFRAME 1000
#define LED_TIMEFRAME 100
#define LED_BLINK_DELAY_SLOW 100  // 100 ms

// ||==============================||
// ||           VARIABLES          ||
// ||==============================||
// ESP-NOW
enum deviceID { BASE_TELEOP, BASE_TELEM }; // Enumeration (to differentiate between which controller board sends out what data)
uint8_t chan = 3; // <======= NOTE! Check if the selected ESP-NOW channel is quiet or not


// Addressable LEDs
enum LED_STATUS {OFFLINE, MANUAL, AUTO, WAIT, RUNNING, STOP, PERMIT, AUX2, AUX3}; // Show different status of robot through the LEDs
int ledStatusCode = 0;


// Flags to indicate certain operations
bool ledState = false;     // default is off
bool autoOrManual = true;  // default is auto mode
bool resetOdom = false;
bool wasSwitchedToManual = false;
bool waitForKey = false;

float measuredRPM[noOfMotors] = { 0, 0, 0, 0 };

// translation and rotation variables
float x = 0;
float y = 0;
float w = 0;


// gps coordinates
float gpsLat = 8888.0;
float gpsLong = 8888.0;


// for non-blocking delay
unsigned long previousResetMillis = 0;
unsigned long dcTimeoutMillis = 0;
unsigned long prevContinueMillis = 0;
unsigned long prevLEDStatusMillis = 0;


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
SPIClass touchscreenSPI = SPIClass(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);


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

// Show the robot status through the RGB LEDs
void ledStatus (unsigned int status){
  unsigned long currMillis = millis();
  static unsigned int lastStatus = 0;
  static int ledNo = 0;
  static int sequence = 0;

  switch(status){
    case OFFLINE:
      if (currMillis - prevLEDStatusMillis >= LED_BLINK_DELAY_SLOW && sequence == 0){

      }
      else if (currMillis - prevLEDStatusMillis >= LED_BLINK_DELAY_SLOW && sequence == 1){

      }
      break;
    case MANUAL:

      break;
    case AUTO:

      break;
    case WAIT:

      break;
    case RUNNING:
      if (currMillis - prevLEDStatusMillis >= LED_BLINK_DELAY_SLOW && sequence == 0){

      }
      else if (currMillis - prevLEDStatusMillis >= LED_BLINK_DELAY_SLOW && sequence == 1){

      }
      break;
    case STOP:

      break;
    case PERMIT:

      break;
    default:

      break;    
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
  
  pinMode(redLEDPin, OUTPUT);
  pinMode(greenLEDPin, OUTPUT);
  pinMode(upBtn, INPUT);
  pinMode(downBtn, INPUT);
  pinMode(leftBtn, INPUT);
  pinMode(rightBtn, INPUT);
  pinMode(enterBtn, INPUT);
  pinMode(escBtn, INPUT);

  // ============================
  //     SERIAL COMMUNICATION
  // ============================
  Serial.begin(115200);  // Begin the serial communication at 115200 baud rate

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
  digitalWrite(redLEDPin, LOW);
  digitalWrite(greenLEDPin, LOW);
  delay(100);
  digitalWrite(redLEDPin, HIGH);
  digitalWrite(greenLEDPin, LOW);
  delay(100);
  digitalWrite(redLEDPin, LOW);
  digitalWrite(greenLEDPin, HIGH);
  delay(100);
  digitalWrite(redLEDPin, HIGH);
  digitalWrite(greenLEDPin, HIGH);
  delay(100);
  digitalWrite(redLEDPin, LOW);
  digitalWrite(greenLEDPin, LOW);

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
    ledStatusCode = OFFLINE;
  }

  



  // Change the led colour of the robot depending on the modes
  ledStatus(ledStatusCode);    


}