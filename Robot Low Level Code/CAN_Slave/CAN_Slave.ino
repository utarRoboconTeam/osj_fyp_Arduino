/*

  ROBOCUP Robot Base Controller Code

  NOTE:
  1) Please use ESP32 board version 2.0.17 instead of 3.0.0 as the latest version appears to have a missing file, causing CAN bus code to not compile successfully

  Last Updated: 10th Jan 2025

*/

// ||==============================||
// ||          LIBRARIES           ||
// ||==============================||
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <ESP32-TWAI-CAN.hpp>

// ||==============================||
// ||          DEFINITIONS         ||
// ||==============================||
// CAN bus pins
#define RX 23  // Connect to RX Pin of the CAN module
#define TX 32  // connect to TX Pin of the CAN module

#define TX_ID_1 0x601
#define TX_ID_2 0x602
#define RX_ID_1 0x581
#define RX_ID_2 0x582


// Time constants
#define DC_TIMEFRAME 1000  // 1000 ms
#define PID_TIMEFRAME 50000 // 50 ms

// DC Motor constants
#define motorMaxRPM 300  // RPM
#define motorMinPWM 0
#define noOfMotors 4


// ||==============================||
// ||           VARIABLES          ||
// ||==============================||

// Features to turn on or off for the controller
bool pidTune = false; // Live PID tuning


// ESP-NOW
enum deviceID { BASE_TELEOP, BASE_TELEM }; // Enumeration (to differentiate between which controller board sends out what data)
uint8_t chan = 5; // To prevent interference-induced lagging, might want to switch to another channel (0 to 11);
uint8_t broadcastAddress[] = { 0xF4, 0x12, 0xFA, 0xE7, 0xA0, 0x8C };  // Remote controller MAC Address


// CAN bus
// data array to store the parameters
uint8_t canID = 0;


// DC Motor
// PID control
// Speed and direction input and output
int target[noOfMotors] = { 0, 0, 0, 0 };          // target speed of the motor in PWM
float mappedTarget[noOfMotors] = { 0, 0, 0, 0 };  // target speed mapped from PWM to RPM
int direction[noOfMotors] = { 0, 0, 0, 0 };       // target direction of the motor
float output[noOfMotors] = { 0, 0, 0, 0 };        // output speed of the motor in RPM
float mappedOutput[noOfMotors] = { 0, 0, 0, 0 };  // output speed mapped from RPM to PWM

// Velocity calculations
float velRaw[noOfMotors] = { 0, 0, 0, 0 };
float velGet[noOfMotors] = { 0, 0, 0, 0 };
float velCalc[noOfMotors] = { 0, 0, 0, 0 };
float velError[noOfMotors] = { 0, 0, 0, 0 };
float velLastError[noOfMotors] = { 0, 0, 0, 0 };

// PID calculations
float proportional[noOfMotors] = { 0, 0, 0, 0 };  
float integral[noOfMotors] = { 0, 0, 0, 0 };
float derivative[noOfMotors] = { 0, 0, 0, 0 };
float clCtrlTimeElapsed[noOfMotors] = { 0, 0, 0, 0 };  

// PID constants (CHANGE THESE TO MATCH YOUR ROBOT'S BASE PERFORMANCE)
// |
// |
// V
float Kp[noOfMotors] = { 1.8, 1.8, 1.65, 1.65 };
float Ki[noOfMotors] = { 0.0, 0.0, 0.0, 0.0 };
float Kd[noOfMotors] = { 0.002, 0.002, 0.002, 0.002 };

// Flags to determine whether operations are done or not
bool allMotorsDone = false; // check for whether all motor velocities have been calculated
bool gotCmdVelData = false; // check for presence of cmd vel data

// For non-blocking delays
unsigned long prevDCTimeoutMillis = 0;
unsigned long currMillis[noOfMotors] = { 0, 0, 0, 0 };
unsigned long prevPIDMillis[noOfMotors] = { 0, 0, 0, 0 };
unsigned long prevCLCtrlTime[noOfMotors] = { 0, 0, 0, 0 };


// ESP-NOW structure to send data
// Must match the receiver structure
typedef struct ESP32TELE {
  
  // which ESP32 board are we receiving data from?
  int deviceID;

  // ==================
  //   TELEOPERATION
  // ==================

  // base
  float xPos, yPos, wPos;
  bool autoOrManual, clearOdom;

  // arm
  float xArm, yArm, zArm;
  unsigned short int rollArm, pitchArm, gripArm;
  bool homeAllAxes;

  // ==============
  //   TELEMETRY
  // ==============

  // base
  float targetRPM[noOfMotors];
  float measuredRPM[noOfMotors];

  // arm
  long int xArmNow, yArmNow, zArmNow;
  unsigned short int rollArmNow, pitchArmNow, gripArmNow, forceReading;

  // ===================
  //   LIVE PID TUNING
  // ===================

  float Kp[noOfMotors];
  float Ki[noOfMotors];
  float Kd[noOfMotors];


} ESP32TELE;

ESP32TELE base; // ESP-NOW message structure object
CanFrame txFrame = { 0 };
CanFrame rxFrame;

// ||==============================||
// ||           FUNCTIONS          ||
// ||==============================||


//====================== CALLBACK FUNCTIONS ==========================

// ESP-NOW callback function that will be executed when data is received, passes all values from the controller to the respective variables
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  memcpy(&base, incomingData, sizeof(base));

  if (base.deviceID == BASE_TELEOP){
    Kp[0] = base.Kp[0];
    Kp[1] = base.Kp[1];
    Kp[2] = base.Kp[2];
    Kp[3] = base.Kp[3];  

    Ki[0] = base.Ki[0];
    Ki[1] = base.Ki[1];
    Ki[2] = base.Ki[2];
    Ki[3] = base.Ki[3];

    Kd[0] = base.Kd[0];
    Kd[1] = base.Kd[1];
    Kd[2] = base.Kd[2];
    Kd[3] = base.Kd[3];
  }    
}

// ESP-NOW send callback
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {

}

//====================== TASK FUNCTIONS ==========================

void canSendSDO(uint16_t nodeID, int byteLength, uint16_t index, uint8_t subIndex, int32_t value){
  // DO NOT PUT NON-BLOCKING DELAY FOR THIS, LET IT RUN AS FAST AS IT CAN

  // we are writing to the driver, so set the identifier to the transmit id
  txFrame.identifier = nodeID;

  txFrame.extd = 0;
  txFrame.data_length_code = 8;
  //Serial.print("Beginning packet at: ");
  //Serial.println(TX_ID, HEX);
  
  // Command word, here we only specify the length of data to be sent
  switch(byteLength){
    case -1:  // arbitrary command to perform read request
      txFrame.data[0] = (0x40);
      break;
    case 1: // 1 byte long of data
      txFrame.data[0] = (0x2F);      
      break;
    case 2: // 2 bytes long of data
      txFrame.data[0] = (0x2B);
      break;
    case 3: // 3 bytes long of data
      txFrame.data[0] = (0x27);
      break;
    case 4: // 4 bytes long of data
      txFrame.data[0] = (0x23);
      break;
    default:  // out of range from datasheet
      txFrame.data[0] = (0x23);  // default it back to 4 bytes
      break;
  }

  // the index to send the data to
  txFrame.data[1] = (uint8_t)(index & 0xFF); // low byte
  // Serial.print("Writing lower byte of index");
  // Serial.println(index & 0xFF, HEX);

  txFrame.data[2] = (uint8_t)(index >> 8); // high byte
  // Serial.print("Writing higher byte of index");
  // Serial.println(index >> 8, HEX);


  // the subindex to send the data to
  txFrame.data[3] = (subIndex);
  //Serial.println("Selecting subindex");


  // Command word, here we only specify the length of data to be sent
  switch(byteLength){
    case -1:
      txFrame.data[4] = 0; // byte 1
      txFrame.data[5] = 0; // byte 2
      txFrame.data[6] = 0; // byte 3
      txFrame.data[7] = 0; // byte 4
      break;
    case 1: // 1 byte long of data
      // the data
      txFrame.data[4] = ((uint8_t)(value & 0xFF)); // byte 1
      //Serial.println("Writing the data");
      break;
    case 2: // 2 bytes long of data
      // the data
      txFrame.data[4] = ((uint8_t)(value & 0xFF)); // byte 1
      txFrame.data[5] = ((uint8_t)(value >> 8) & 0xFF); // byte 2
      //Serial.println("Writing the data");
      break;
    case 3: // 3 bytes long of data
      // the data
      txFrame.data[4] = ((uint8_t)(value & 0xFF)); // byte 1
      txFrame.data[5] = ((uint8_t)(value >> 8) & 0xFF); // byte 2
      txFrame.data[6] = ((uint8_t)(value >> 16) & 0xFF); // byte 3
      //Serial.println("Writing the data");
      break;
    case 4: // 4 bytes long of data
      // the data
      txFrame.data[4] = ((uint8_t)(value & 0xFF)); // byte 1
      txFrame.data[5] = ((uint8_t)(value >> 8) & 0xFF); // byte 2
      txFrame.data[6] = ((uint8_t)(value >> 16) & 0xFF); // byte 3
      txFrame.data[7] = ((uint8_t)(value >> 24) & 0xFF); // byte 4
      //Serial.println("Writing the data");
      break;
    default:  // out of range from datasheet
      // the data
      txFrame.data[4] = ((uint8_t)(value & 0xFF)); // byte 1
      txFrame.data[5] = ((uint8_t)(value >> 8) & 0xFF); // byte 2
      txFrame.data[6] = ((uint8_t)(value >> 16) & 0xFF); // byte 3
      txFrame.data[7] = ((uint8_t)(value >> 24) & 0xFF); // byte 4
      //Serial.println("Writing the data");
      break;
  }


  ESP32Can.writeFrame(txFrame);  // timeout defaults to 1 ms
  //Serial.println("Done. Sending packet...");


}

void canReceiveSDO(){
  // You can set custom timeout, default is 1000
  if(ESP32Can.readFrame(rxFrame, 1000)) {
      // Comment out if too many frames
      Serial.printf("Received frame: %03X  \r\n", rxFrame.identifier);
      Serial.print(rxFrame.data[0], HEX);
      Serial.print(" ");
      Serial.print(rxFrame.data[1], HEX);
      Serial.print(" ");
      Serial.print(rxFrame.data[2], HEX);
      Serial.print(" ");
      Serial.print(rxFrame.data[3], HEX);
      Serial.print(" ");
      Serial.print(rxFrame.data[4], HEX);
      Serial.print(" ");
      Serial.print(rxFrame.data[5], HEX);
      Serial.print(" ");
      Serial.print(rxFrame.data[6], HEX);
      Serial.print(" ");
      Serial.print(rxFrame.data[7], HEX);
      Serial.print(" ");
  }
}

// Get command velocities from main controller via CAN bus
void readCmdVel(){

  // check for incoming CAN data. If there is, parse the data packet
  if(ESP32Can.readFrame(rxFrame, 0)) {
    if (rxFrame.identifier == 0x0001){
      for (int i = 0; i < noOfMotors; i++){
        // if you are using those bldc motors purchased on Taobao
        // They have a deadzone where the input within that deadzone will not move the motor
        // so we need to offset it
        target[i] = map(rxFrame.data[i], 0, 255, motorMinPWM, 255); 
        mappedTarget[i] =  map(abs(target[i]), motorMinPWM, 255, 0, motorMaxRPM);
        direction[i] = ((rxFrame.data[i + 4] == 0) ? -1 : 1);      
      }
      gotCmdVelData = true;
    }
  }

  // No CAN data  
  else{
    gotCmdVelData = false;
  }

}

void sendCmdVel(){
  Serial.print(target[0] * direction[0]);
  Serial.print(" ");
  Serial.println(target[1] * direction[1]);
  canSendSDO(TX_ID_1, 4, 0x60FF, 0x01, target[0] * direction[0]);
  canSendSDO(TX_ID_1, 4, 0x60FF, 0x02, target[1] * direction[1]);
  //canSendSDO(TX_ID_2, 4, 0x60FF, 0x01, target[2] * direction[2]);
  //canSendSDO(TX_ID_2, 4, 0x60FF, 0x02, target[3] * direction[3]);
}


void readWheelSpeeds(){
  canSendSDO(TX_ID_1, -1, 0x60FF, 0x01, 0x00);
  canReceiveSDO();
  canSendSDO(TX_ID_1, -1, 0x60FF, 0x02, 0x00);
  canReceiveSDO();
  canSendSDO(TX_ID_2, -1, 0x60FF, 0x01, 0x00);
  canReceiveSDO();
  canSendSDO(TX_ID_2, -1, 0x60FF, 0x02, 0x00);
  canReceiveSDO();
}

void driverStartup(){

  canSendSDO(TX_ID_1, 1, 0x2001, 0x00, 0x01); // Enable only the left motor
  //delay(10);

  canSendSDO(TX_ID_1, 2, 0x200F, 0x00, 0x00); // set the driver to asynchronous movement
  //delay(10);

  canSendSDO(TX_ID_1, 1, 0x6060, 0x00, 0x03); // select Speed mode operation 
  //delay(10);

  canSendSDO(TX_ID_1, 4, 0x6083, 0x01, 0x64); // Acceleration time (100 ms) for left wheel
  //delay(10);

  canSendSDO(TX_ID_1, 4, 0x6083, 0x02, 0x64); // Acceleration time (100 ms) for right wheel
  //delay(10);

  canSendSDO(TX_ID_1, 4, 0x6084, 0x01, 100); // Deceleration time (100 ms) for left wheel
  //delay(10);

  canSendSDO(TX_ID_1, 4, 0x6084, 0x02, 100); // Deceleration time (100 ms) for right wheel
  //delay(10);

  // enable the driver
  canSendSDO(TX_ID_1, 2, 0x6040, 0x00, 0x06);
  //delay(10);

  canSendSDO(TX_ID_1, 2, 0x6040, 0x00, 0x07);
  //delay(10);

  canSendSDO(TX_ID_1, 2, 0x6040, 0x00, 0x0F);
  //delay(10);


  Serial.println("Status of motor driver: ");
  canSendSDO(TX_ID_1, -1, 0x6041, 0x00, 0x00); // Request motor state
  canReceiveSDO();
}


// |=====================================|
// |                                     |
// |            SETUP FUNCTION           |
// |                                     |
// |=====================================|

void setup() {
  // ============================
  //     SERIAL COMMUNICATION
  // ============================
  Serial.begin(115200);  // Begin the serial communication at 115200 baud rate


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

  driverStartup();

  // ==============
  //    ESP-NOW 
  // ==============
  WiFi.mode(WIFI_STA);  // Set device as a Wi-Fi Station

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(chan, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  //Register callback
  esp_now_register_send_cb(OnDataSent);

  //Register Peer
  esp_now_peer_info_t peerInfo;
  peerInfo.channel = chan;
  peerInfo.ifidx = WIFI_IF_STA;                         // In case ESP-Now refuses to connect to other devices, can add this in
  peerInfo.encrypt = false;

  //Add peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // ESP-NOW receive callback function
  // Only works if live PID tuning is enabled in the code
  if (pidTune){
    esp_now_register_recv_cb(OnDataRecv);    
  }

}


// |=====================================|
// |                                     |
// |            LOOP FUNCTION            |
// |                                     |
// |=====================================|
void loop() {
  unsigned long currMillis = millis();

  // Get the command velocities from CAN bus
  readCmdVel();

  // If we got command velocities, operate the motors
  if (gotCmdVelData){
    prevDCTimeoutMillis = currMillis;
    sendCmdVel();
  }

  // Safety feature where if there is no CAN data received, turn off the motors
  else {
    if(currMillis - prevDCTimeoutMillis >= DC_TIMEFRAME){
      Serial.println("No CAN data received. Stopping motors!");
      for (int i = 0; i < noOfMotors; i++){
        canSendSDO(TX_ID_1, 4, 0x60FF, 0x01, 0x00);
        canSendSDO(TX_ID_1, 4, 0x60FF, 0x02, 0x00);
      }
    }
  }
}
