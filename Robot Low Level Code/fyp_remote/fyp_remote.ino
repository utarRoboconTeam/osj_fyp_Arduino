/*

Controller using ESP32-S3 Microcontroller (Mk2 Variant 1)

Last Updated: 7th November 2024

*/

// ||==============================||
// ||          LIBRARIES           ||
// ||==============================||
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_NeoPixel.h>
#include "bitmaps.h"

// ||==============================||
// ||          DEFINITIONS         ||
// ||==============================||
#define x_pin 5   // left joystick x axis pin
#define y_pin 4   // left joystick y axis pin
#define z_pin 7  // right joystick x axis pin
#define d_pin 6  // right joystick y axis pin

#define SW1 42  // joystick left button
#define SW2 41 // joystick right button

#define B1 11   // Up
#define B2 13  // Down
#define B3 14  // Left
#define B4 12  // Right

#define B5 15   // Triangle
#define B6 16  // Square
#define B7 17  // Circle
#define B8 18  // X

#define L1 40  // L1  Shoot
#define R1 39  // L1  Shoot

#define LU 35
#define LD 36
#define RU 37
#define RD 38

#define buzzerPin 8
#define motorPin 20

#define pot1 9 // top potentiometer
#define pot2 10 // bottom potentiometer
#define pot3 1 // top potentiometer
#define pot4 2 // bottom potentiometer

#define led_pin        48 // On Trinket or Gemma, suggest changing this to 1
#define no_of_pixels 1 // Popular NeoPixel ring size

#define i2c_Address 0x3c //initialize with the I2C addr 0x3C Typically eBay OLED's
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   //   QT-PY / XIAO

// DC motor constants
#define noOfMotors 4

// Controller constants
#define rotateMultiplier 1.18   // amplify the yaw movement of the robot by increasing the right joystick x axis sensitivity
#define MaxReadings 50          //For smoothening left joystick; Adjustable; larger number = more delay but smoother.
#define ADC_bit 12              // ADC measurement

// Battery constants
#define LOW_BATTERY_LEVEL 25
#define DANGEROUS_BATTERY_LEVEL 10


// Time constants
#define ESPNOW_TIMEFRAME 50  // 50 ms
#define DISPLAY_TIMEFRAME 200
#define CLICK_SOUND_TIMEFRAME 100
#define CHANGE_TONE_TIMEFRAME 200
#define debounce 200UL


// ||==============================||
// ||           VARIABLES          ||
// ||==============================||
// Features to turn on or off for the controller
bool useZPower = false; // true to use amplified z axis data, false to use normal z axis data
bool pidTune = false;  // true to enable live PID tuning, false to disable it

// ESP-NOW
enum deviceID { MAIN_TELEOP, BASE_TELEOP, MAIN_TELEM, BASE_TELEM }; // Enumeration (to differentiate between which controller board sends out what data)
uint8_t chan = 5;   // To prevent interference-induced lagging, might want to switch to another channel (0 to 11);
uint8_t broadcastAddress1[] = { 0xA0, 0xA3, 0xB3, 0x8A, 0x7D, 0x88 };  // Main Controller
uint8_t broadcastAddress2[] = { 0xD8, 0xBC, 0x38, 0xFC, 0x2D, 0xF8 };  // Telemetry Viewer
uint8_t broadcastAddress3[] = { 0xA0, 0xA3, 0xB3, 0x8A, 0x67, 0xD8 };  // Base Controller

// Addressable LED parameters
enum LED_STATUS {OFFLINE, MANUAL, AUTO, WAIT, RUNNING, STOP, PERMIT, AUX2, AUX3}; // Show different status of robot through the LEDs
unsigned int red = 0, green = 0, blue = 0;  // onboard LED RGB values

// Joystick
float x, y, z, d, zPower;                   // joystick parameters
float xBuf = 0, yBuf = 0, zBuf = 0, dBuf = 0;
float xPos, yPos, wPos;
int joystickx, joysticky, joystickz, joystickd;

// Buttons
// for mode changing
int enableReading, prevEnableReading;
int telemReading, prevTelemReading;

// DC motors telemetry data
float targetRPM[noOfMotors] = {0.0, 0.0, 0.0, 0.0};
float measuredRPM[noOfMotors] = {0.0, 0.0, 0.0, 0.0};

// Main controller telemetry data
float internalTemp = 8888.0;
float internalHumid = 8888.0;
float gpsLat = 8888.0;
float gpsLong = 8888.0;
float batteryLevel = 0.0;
int robotStatus = OFFLINE;

// Flags to indicate certain operations
bool autoOrManual = false; // true to make it auto only
bool resetOdom = false;
bool updateDisplay = true; 
bool waitForKey = false;
bool restartDriver = false;
bool telemToView = false;

// for non-blocking delay
unsigned long prevESPNOWMillis = 0;
unsigned long prevButtonMillis = 0;
unsigned long prevDisplayMillis = 0;

// to make sure the values are within the 12 bit range
int xcenter, ycenter, zcenter, dcenter;
int xMin = 0, xMax = 4095;
int yMin = 0, yMax = 4095;
int zMin = 0, zMax = 4095;
int dMin = 0, dMax = 4095;

// left joystick x-axis averaging filter
int Xreadings[MaxReadings];
int XreadIndex = 0;
int Xtotal = 0;
int X_Pos = 0;

// left joystick y-axis averaging filter
int Yreadings[MaxReadings];
int YreadIndex = 0;
int Ytotal = 0;
int Y_Pos = 0;

// right joystick x-axis averaging filter
int Zreadings[MaxReadings];
int ZreadIndex = 0;
int Ztotal = 0;
int Z_Pos = 0;

// right joystick y-axis averaging filter
int Dreadings[MaxReadings];
int DreadIndex = 0;
int Dtotal = 0;
int D_Pos = 0;

// outermost potentiometer averaging filter
int POT1readings[MaxReadings];
int POT1readIndex = 0;
int POT1total = 0;
int POT1_Pos = 0;

// innermost potentiometer averaging filter
int POT2readings[MaxReadings];
int POT2readIndex = 0;
int POT2total = 0;
int POT2_Pos = 0;

// outermost potentiometer averaging filter
int POT3readings[MaxReadings];
int POT3readIndex = 0;
int POT3total = 0;
int POT3_Pos = 0;

// innermost potentiometer averaging filter
int POT4readings[MaxReadings];
int POT4readIndex = 0;
int POT4total = 0;
int POT4_Pos = 0;

// ------------------
// OBJECTS
// ------------------
// Structure example to send data
// Must match the receiver structure
typedef struct ESP32TELE {
  
  // which ESP32 board are we receiving data from?
  int deviceID;

  // ==================
  //   TELEOPERATION
  // ==================

  // main
  float xPos, yPos, wPos;
  bool autoOrManual, resetOdom, restartDriver, waitKey;
  float headlightIntensity;

  // ==============
  //   TELEMETRY
  // ==============

  // main
  float internalTemp, internalHumid;
  float longitude, latitude;
  float batteryLevel;
  int robotStatus;

  // base
  float targetRPM[noOfMotors];
  float measuredRPM[noOfMotors];

} ESP32TELE;

ESP32TELE tele; // ESP-NOW message structure object
esp_now_peer_info_t peerInfo; // Declare structure object

Adafruit_NeoPixel pixels(no_of_pixels, led_pin, NEO_GRB + NEO_KHZ800);

Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// send callback
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS){
    green = 100;
    red = 0;
  }
  else{
    green = 0;
    red = 100; 
  }
}

// callback function that will be executed when data is received, passes all values from the controller to the respective variables
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  memcpy(&tele, incomingData, sizeof(tele));
  if (tele.deviceID == BASE_TELEM){
    targetRPM[0] = tele.targetRPM[0];
    targetRPM[1] = tele.targetRPM[1];
    targetRPM[2] = tele.targetRPM[2];
    targetRPM[3] = tele.targetRPM[3];
    measuredRPM[0] = tele.measuredRPM[0];
    measuredRPM[1] = tele.measuredRPM[1];
    measuredRPM[2] = tele.measuredRPM[2];
    measuredRPM[3] = tele.measuredRPM[3];
  }
  else if (tele.deviceID == MAIN_TELEM){
    internalTemp = tele.internalTemp;
    internalHumid = tele.internalHumid;
    batteryLevel = tele.batteryLevel;
    robotStatus = tele.robotStatus;
  }
}

void normalDisplay(){
  display.clearDisplay();

  display.setTextSize(1);

  // -------------------------------------
  //            SCREEN HEADER
  // -------------------------------------

  // Control mode
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.print("CTRL:");
  display.setTextColor(SH110X_BLACK, SH110X_WHITE); // 'inverted' text
  if (autoOrManual){
    display.print("AUTO");
  }
  else{
    display.print("MANU");
  }

  // Robot status
  display.setTextColor(SH110X_WHITE);
  display.setCursor(70, 0);
  display.print("STAT:");
  display.setTextColor(SH110X_BLACK, SH110X_WHITE); // 'inverted' text
  switch(robotStatus){
    case OFFLINE:
      display.print("OFF");
      break;
    case MANUAL:
      display.print("MANU");
      break;
    case AUTO:
      display.print("AUTO");
      break;
    case WAIT:
      display.print("WAIT");
      break;
    case RUNNING:
      display.print("RUN");
      break;
    case STOP:
      display.print("STOP");
      break;
    case PERMIT:
      display.print("PERM");
      break;
    default:
      display.print("UNK");
      break;
  }

  display.drawLine(0, 10, 128, 10, SH110X_WHITE);


  // -------------------------------------
  //             DATA DISPLAY
  // -------------------------------------

  // Input controls display and input pop-ups
  if (!resetOdom && !restartDriver){
    display.drawLine(96, 15, 96 + map(POT1_Pos, 0, 255, 0, 28), 15, SH110X_WHITE);
    display.drawCircle(110, 40, 16, SH110X_WHITE);
    display.drawCircle(110 + (int)(xPos * 10), 40 - (int)(yPos * 10), 1, SH110X_WHITE);
    display.drawCircle(110 - (int)(wPos * 15), 20, 1, SH110X_WHITE);
  }
  // reset odom pop up
  else if (resetOdom){
    display.setTextColor(SH110X_WHITE);
    display.setCursor(80, 20);
    display.print("ODOM");
    display.setCursor(80, 40);
    display.print("RESET!");
  }
  // restart driver pop up
  else if (restartDriver){
    display.setTextColor(SH110X_WHITE);
    display.setCursor(80, 20);
    display.print("DRIVER");
    display.setCursor(80, 40);
    display.print("RESET!");
  }


  // switch between which telemetry data to view
  if (!telemToView){
    // Front Left Wheel speed
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 13);
    display.print("FLW:");
    display.setTextColor(SH110X_BLACK, SH110X_WHITE); // 'inverted' text
    display.print((int)measuredRPM[0]);
    display.print(" RPM");   

    // Front Right Wheel speed
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 23);
    display.print("FRW:");
    display.setTextColor(SH110X_BLACK, SH110X_WHITE); // 'inverted' text
    display.print((int)measuredRPM[1]);
    display.print(" RPM");   

    // Back Left Wheel speed
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 33);
    display.print("BLW:");
    display.setTextColor(SH110X_BLACK, SH110X_WHITE); // 'inverted' text
    display.print((int)measuredRPM[2]);
    display.print(" RPM"); 

    // Back Right Wheel speed
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 43);
    display.print("BRW:");
    display.setTextColor(SH110X_BLACK, SH110X_WHITE); // 'inverted' text
    display.print((int)measuredRPM[3]);
    display.print(" RPM");    

  }

  // Display robot condition
  else{

    // Temperature
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 13);
    display.print("T:");
    display.setTextColor(SH110X_BLACK, SH110X_WHITE); // 'inverted' text
    display.print((String)internalTemp);
    display.print(" C");

    // Humidity
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 23);
    display.print("H:");
    display.setTextColor(SH110X_BLACK, SH110X_WHITE); // 'inverted' text
    display.print((String)internalHumid);
    display.print(" RH");

    // Longitude
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 33);
    display.print("Long:");
    display.setTextColor(SH110X_BLACK, SH110X_WHITE); // 'inverted' text
    display.print((String)gpsLong);

    // Latitude
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 43);
    display.print("Lat:");
    display.setTextColor(SH110X_BLACK, SH110X_WHITE); // 'inverted' text
    display.print((String)gpsLat);

    // Battery Level
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 53);
    display.print("B:");
    display.setTextColor(SH110X_BLACK, SH110X_WHITE); // 'inverted' text
    display.print((String)map(batteryLevel, 30, 42, 0, 100));
    display.print(" %");
  }


  display.display();
}

void RGBLEDStatus(unsigned int red, unsigned int green, unsigned int blue){
  pixels.setPixelColor(0, pixels.Color(red, green, blue));
  pixels.show();
}

void buzzerStatus(){
  unsigned long currMillis = millis();
  static unsigned long prevBuzzerMillis = 0;
  static unsigned long prevBuzzer2Millis = 0;
  static int buzzerSequence = 0;
  static bool playOnce = false;

  // make a short beep sound to emulate button press sound whenever any of the buttons are pressed
  if (enableReading || telemReading || resetOdom || waitForKey || restartDriver){
    ledcWriteNote(0, NOTE_C, 4);
    if (currMillis - prevBuzzer2Millis >= CLICK_SOUND_TIMEFRAME){
      ledcWrite(0, 0);
    }
  }

  else{
    prevBuzzer2Millis = currMillis;
    if (batteryLevel < DANGEROUS_BATTERY_LEVEL){
      if (currMillis - prevBuzzerMillis >= CHANGE_TONE_TIMEFRAME && buzzerSequence == 0){
        ledcWriteNote(0, NOTE_G, 5);
        buzzerSequence = 1;
        prevBuzzerMillis = currMillis;
      }
      else if (currMillis - prevBuzzerMillis >= CHANGE_TONE_TIMEFRAME && buzzerSequence == 1){
        ledcWriteNote(0, NOTE_E, 5);
        buzzerSequence = 0;
        prevBuzzerMillis = currMillis;
      }   
    }

    else if (batteryLevel < LOW_BATTERY_LEVEL && !playOnce){
      if (buzzerSequence == 0){
        ledcWriteNote(0, NOTE_G, 5);
        buzzerSequence = 1;
      }
      else if (currMillis - prevBuzzerMillis >= CHANGE_TONE_TIMEFRAME && buzzerSequence == 1){
        ledcWriteNote(0, NOTE_E, 5);
        buzzerSequence = 2;
        prevBuzzerMillis = currMillis;
      }
      else if (currMillis - prevBuzzerMillis >= CHANGE_TONE_TIMEFRAME && buzzerSequence == 2){
        playOnce = true;
        buzzerSequence = 0;
      }
    }

    else if (batteryLevel > LOW_BATTERY_LEVEL){
      prevBuzzerMillis = currMillis;
      playOnce = false;
      ledcWrite(0, 0);
    }

    else{
      prevBuzzerMillis = currMillis;
      ledcWrite(0, 0);
    }
  }
}


// check for initial position of both joysticks
void calibrate() {
  Serial.println("\n---calibrating joystick---\n");
  Serial.println("place the joystick in the center position");
  delay(1000);
  joystickx = joysticky = joystickz = joystickd = 0;
  blue = 100;
  RGBLEDStatus(red, green, blue);
  for (int i = 0; i < 100; i++) {
    Serial.print(".");
    joystickx += analogRead(x_pin);
    delay(5);
    joysticky += analogRead(y_pin);
    delay(5);
    joystickz += analogRead(z_pin);
    delay(5);
    joystickd += analogRead(d_pin);
  }
  xcenter = joystickx / 100;
  ycenter = joysticky / 100;
  zcenter = joystickz / 100;
  dcenter = joystickd / 100;
  delay(1000);
  blue = 0;
  RGBLEDStatus(red, green, blue);
}

void setup() {
  Serial.begin(115200);

  // ==============
  //    ESP-NOW
  // ==============
  WiFi.mode(WIFI_STA);

  // Set the WiFi channel to channel 11
  // Available channels are 0 to 14
  // Channel 0 will auto select the channel the station or softAP is on
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(chan, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  //Initialise ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error");
    return;
  }

  //Register callback
  esp_now_register_send_cb(OnDataSent);

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  //Register Peer
  esp_now_peer_info_t peerInfo;
  peerInfo.channel = chan;
  peerInfo.ifidx = WIFI_IF_STA;                         // In case ESP-Now refuses to connect to other devices, can add this in
  peerInfo.encrypt = false;

  //Add peer
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  //Add peer
  memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  //Add peer
  memcpy(peerInfo.peer_addr, broadcastAddress3, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // ==========
  //    GPIO
  // ==========
  pinMode(B1, INPUT);
  pinMode(B2, INPUT);
  pinMode(B3, INPUT);
  pinMode(B4, INPUT);
  pinMode(B5, INPUT);
  pinMode(B6, INPUT);
  pinMode(B7, INPUT);
  pinMode(B8, INPUT);
  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
  pinMode(L1, INPUT);
  pinMode(R1, INPUT);
  pinMode(LU, INPUT);
  pinMode(LD, INPUT);
  pinMode(RU, INPUT);
  pinMode(RD, INPUT);

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear(); // Set all pixel colors to 'off'

  // ==============
  //    DISPLAY
  // ==============
  Wire.setPins(47, 21); // 47 - SDA pin, 21 - SCL pin
  display.begin(i2c_Address, true); // Address 0x3C default

  // Clear the buffer.
  display.clearDisplay();
  display.display();

  display.drawBitmap(0, 0, robocon_logo, 128, 64, 1);
  display.display();

  // =============
  //     SOUND
  // =============
  ledcSetup(0, 2000, 8);
  ledcAttachPin(buzzerPin, 0);
  //ledcWriteTone(0, 523.25);
  ledcWriteNote(0, NOTE_C, 5);
  delay(350);
  //ledcWriteTone(0, 587.33);
  ledcWriteNote(0, NOTE_D, 5);
  delay(350);
  //ledcWriteTone(0, 783.99);
  ledcWriteNote(0, NOTE_G, 5);
  delay(350);
  ledcWrite(0, 0);
  delay(300);

  // Calibrate joystick center
  calibrate();
  updateDisplay = true;
}

void loop() {
  unsigned long currMillis = millis();       // count the time the ESP32 has stayed powered on

  // averaging filter for joysticks and potentiometers
  Xtotal -= Xreadings[XreadIndex];
  Ytotal -= Yreadings[YreadIndex];
  Ztotal -= Zreadings[ZreadIndex];
  Dtotal -= Dreadings[DreadIndex];
  POT1total -= POT1readings[POT1readIndex];
  POT2total -= POT2readings[POT2readIndex];
  POT3total -= POT3readings[POT3readIndex];
  POT4total -= POT4readings[POT4readIndex];
  Xreadings[XreadIndex] = analogRead(x_pin);
  Yreadings[YreadIndex] = analogRead(y_pin);
  Zreadings[ZreadIndex] = analogRead(z_pin);
  Dreadings[DreadIndex] = analogRead(d_pin);
  POT1readings[POT1readIndex] = analogRead(pot1);
  POT2readings[POT2readIndex] = analogRead(pot2);
  POT3readings[POT3readIndex] = analogRead(pot3);
  POT4readings[POT4readIndex] = analogRead(pot4);
  Xtotal = Xtotal + Xreadings[XreadIndex];
  Ytotal = Ytotal + Yreadings[YreadIndex];
  Ztotal = Ztotal + Zreadings[ZreadIndex];
  Dtotal = Dtotal + Dreadings[DreadIndex];
  POT1total = POT1total + POT1readings[POT1readIndex];
  POT2total = POT2total + POT2readings[POT2readIndex];
  POT3total = POT3total + POT3readings[POT3readIndex];
  POT4total = POT4total + POT4readings[POT4readIndex];
  XreadIndex = XreadIndex + 1;
  YreadIndex = YreadIndex + 1;
  ZreadIndex = ZreadIndex + 1;
  DreadIndex = DreadIndex + 1;
  POT1readIndex = POT1readIndex + 1;
  POT2readIndex = POT2readIndex + 1;
  POT3readIndex = POT3readIndex + 1;
  POT4readIndex = POT4readIndex + 1;
  if (XreadIndex >= MaxReadings) XreadIndex = 0;
  if (YreadIndex >= MaxReadings) YreadIndex = 0;
  if (ZreadIndex >= MaxReadings) ZreadIndex = 0;
  if (DreadIndex >= MaxReadings) DreadIndex = 0;
  if (POT1readIndex >= MaxReadings) POT1readIndex = 0;
  if (POT2readIndex >= MaxReadings) POT2readIndex = 0;
  if (POT3readIndex >= MaxReadings) POT3readIndex = 0;
  if (POT4readIndex >= MaxReadings) POT4readIndex = 0;
  X_Pos = Xtotal / MaxReadings;
  Y_Pos = Ytotal / MaxReadings;
  Z_Pos = Ztotal / MaxReadings;
  D_Pos = Dtotal / MaxReadings;
  POT1_Pos = map((POT1total / MaxReadings), 0, 4095, 0, 255);
  POT2_Pos = map((POT2total / MaxReadings), 0, 4095, 0, 255);
  POT3_Pos = map((POT3total / MaxReadings), 0, 4095, 0, 255);
  POT4_Pos = map((POT4total / MaxReadings), 0, 4095, 0, 255);

  //Remapping joysticks
  if (X_Pos > xcenter + 100) {  // Left Joy Stick X-axis
    x = map(X_Pos, xcenter, xMax, 0, 1000);
    x = x / 1000;
  } else if (X_Pos < xcenter - 100) {
    x = map(X_Pos, xcenter, xMin, 0, 1000);
    x = x / -1000;
  } else {
    x = 0;
  }
  if (Y_Pos > ycenter + 100) {  // Left Joy Stick Y-axis
    y = map(Y_Pos, ycenter, yMax, 0, 1000);
    y = y / -1000;
  } else if (Y_Pos < ycenter - 100) {
    y = map(Y_Pos, ycenter, yMin, 0, 1000);
    y = y / 1000;
  } else {
    y = 0;
  }
  if (Z_Pos > zcenter + 100) {  // Right Joy Stick X-axis
    z = map(Z_Pos, zcenter, zMax, 0, 1000);
    z = z / -1000;
  } else if (Z_Pos < zcenter - 100) {
    z = map(Z_Pos, zcenter, zMin, 0, 1000);
    z = z / 1000;
  } else {
    z = 0;
  }
  if (D_Pos > dcenter + 500) {  // Right Joy Stick Y-axis
    d = map(D_Pos, dcenter, dMax, 0, -255);
  } else if (D_Pos < dcenter - 800) {
    d = map(D_Pos, dcenter, dMin, 0, 255);
  } else {
    d = 0;
  }

  // if this zPower is not done and z value is used instead, the robot's yaw motion will be slow
  zPower = pow(z * rotateMultiplier, 5);

  // Obtain the mapped joysticks values
  xBuf = x * (POT1_Pos / 255.0);
  yBuf = y * (POT1_Pos / 255.0);
  zBuf = (useZPower ? zPower : z) * (POT1_Pos / 255.0);
  dBuf = d * (POT1_Pos / 255.0);


  // ========================================================
  //                  NORMAL OPERATION MODE
  // ========================================================
  // switch between auto or manual control of robot 
  enableReading = digitalRead(B6);
  if (enableReading == 1 && prevEnableReading == 0 && currMillis - prevButtonMillis >= debounce) {
    autoOrManual = !autoOrManual;
    prevButtonMillis = currMillis;
  }
  prevEnableReading = enableReading;

  // switch between the telemetry readings to check
  telemReading = digitalRead(B3);
  if (telemReading == 1 && prevTelemReading == 0 && currMillis - prevButtonMillis >= debounce) {
    telemToView = !telemToView;
    prevButtonMillis = currMillis;
  }
  prevTelemReading = telemReading;

  resetOdom = digitalRead(B5);
  waitForKey = digitalRead(B8);
  restartDriver = digitalRead(B7);
  xPos = xBuf;
  yPos = yBuf;        
  wPos = zBuf;
  tele.xPos = xPos;
  tele.yPos = yPos;
  tele.wPos = wPos;    
  tele.resetOdom = resetOdom;    
  tele.autoOrManual = autoOrManual;
  tele.waitKey = waitForKey;

  buzzerStatus();

  // send the data
  if (currMillis - prevESPNOWMillis >= ESPNOW_TIMEFRAME){
    tele.deviceID = MAIN_TELEOP;
    esp_err_t toMain = esp_now_send(broadcastAddress1, (uint8_t *)&tele, sizeof(tele));
    esp_err_t toTelem = esp_now_send(broadcastAddress2, (uint8_t *)&tele, sizeof(tele));
    
    // If the robot suddenly behaves weirdly in its motion or it ceases to move,
    // most likely there was a power reset to the motor drivers
    // need to restart it manually    
    tele.deviceID = BASE_TELEOP;
    tele.restartDriver = restartDriver;
    esp_err_t toBase = esp_now_send(broadcastAddress3, (uint8_t *)&tele, sizeof(tele));
    prevESPNOWMillis = currMillis;
  }

  if (currMillis - prevDisplayMillis >= DISPLAY_TIMEFRAME){
    prevDisplayMillis = currMillis;
    normalDisplay();    
  }
  RGBLEDStatus(red, green, blue);

  // Uncomment this to check which buttons correspond to which variables
  // Serial.print(xBuf);
  // Serial.print(" ");
  // Serial.print(yBuf);
  // Serial.print(" ");
  // Serial.print(zBuf);
  // Serial.print(" ");
  // Serial.print(dBuf);
  // Serial.print(" ");
  // Serial.print(POT1_Pos);
  // Serial.print(" ");
  // Serial.print(POT2_Pos);
  // Serial.print(" ");
  // Serial.print(POT3_Pos);
  // Serial.print(" ");
  // Serial.print(POT4_Pos);
  // Serial.print(" ");
  // Serial.print(digitalRead(B1));
  // Serial.print(" ");
  // Serial.print(digitalRead(B2));
  // Serial.print(" ");
  // Serial.print(digitalRead(B3));
  // Serial.print(" ");
  // Serial.print(digitalRead(B4));
  // Serial.print(" ");
  // Serial.print(digitalRead(B5));
  // Serial.print(" ");
  // Serial.print(digitalRead(B6));
  // Serial.print(" ");
  // Serial.print(digitalRead(B7));
  // Serial.print(" ");
  // Serial.print(digitalRead(B8));
  // Serial.print(" ");
  // Serial.print(digitalRead(L1));
  // Serial.print(" ");
  // Serial.print(digitalRead(R1));
  // Serial.print(" ");
  // Serial.print(digitalRead(LU));
  // Serial.print(" ");
  // Serial.print(digitalRead(LD));
  // Serial.print(" ");
  // Serial.print(digitalRead(RU));
  // Serial.print(" ");
  // Serial.print(digitalRead(RD));
  // Serial.print(" ");
  // Serial.print(digitalRead(SW1));
  // Serial.print(" ");
  // Serial.println(digitalRead(SW2));

}


