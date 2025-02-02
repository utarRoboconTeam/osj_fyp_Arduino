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
#include <CAN.h>


// ||==============================||
// ||          DEFINITIONS         ||
// ||==============================||
// CAN bus pins
#define RX 26  // Connect to RX Pin of the CAN module
#define TX 27  // connect to TX Pin of the CAN module

// Time constants
#define DC_TIMEFRAME 1000  // 1000 ms
#define PID_TIMEFRAME 50000 // 50 ms

// DC Motor constants
#define motorMaxRPM 1400  // RPM
#define motorMinPWM 0
#define noOfMotors 4
#define pulsePerMotorRev 5
#define internalGearRatio 4 // 1 to 4
#define externalGearRatio 1


// ||==============================||
// ||           VARIABLES          ||
// ||==============================||

// Features to turn on or off for the controller
bool pidTune = false; // Live PID tuning


// ESP-NOW
enum deviceID { BASE_TELEOP, ARM_TELEOP, BASE_TELEM, ARM_TELEM }; // Enumeration (to differentiate between which controller board sends out what data)
uint8_t chan = 3; // To prevent interference-induced lagging, might want to switch to another channel (0 to 11);
uint8_t broadcastAddress[] = { 0xF4, 0x12, 0xFA, 0xE7, 0xA0, 0x8C };  // Remote controller MAC Address


// CAN bus
// data array to store the parameters
int data[8];
int CANIndex = 0;
uint8_t canID = 0;


// DC Motor
// GPIO pins
int ENCA[noOfMotors] = { 22, 25, 32, 13 };  // encoder A on the motor
int ENCB[noOfMotors] = { 21, 12, 23, 19 };  // encoder B on the motor
int PWM[noOfMotors] = { 16, 4, 5, 2 };      // PWM data
int DIR[noOfMotors] = { 14, 17, 18, 33 };   // Direction data

volatile int counter[noOfMotors] = { 0, 0, 0, 0 };  // Encoder pulses

// Motor constant calculation
// different motor has different constant of n output shaft / m pulses
// please calculate yours by looking up these parameters in your motor datasheet and calculate using the formula below
// 1) no of pulses in 1 motor shaft
// 2) gearbox gear ratio
// 3) external gear ratio
// [constant] = (p no. of pulses / 1 motor shaft rotation) * (gearbox gear ratio (basically q no. of motor shaft rotations / 1 output shaft rotation)) * ([optional] output gear ratio, only implement if you have external gearing)
float motorConstant = pulsePerMotorRev * internalGearRatio * externalGearRatio;  // in pulses per output shaft rotation

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


// ||==============================||
// ||           FUNCTIONS          ||
// ||==============================||

//====================== INTERRUPT SERVICE ROUTINES ==========================

// Front left wheel encoder
void IRAM_ATTR readEncoderFLW() {
  //Serial.println("ENCA Triggered on encoder 1");
  counter[0]++;
}

// Front right wheel encoder
void IRAM_ATTR readEncoderFRW() {
  //Serial.println("ENCA Triggered on encoder 2");
  counter[1]++;
}

// Back left wheel encoder
void IRAM_ATTR readEncoderBLW() {
  //Serial.println("ENCB Triggered on encoder 3");
  counter[2]++;
}

// Back right wheel encoder
void IRAM_ATTR readEncoderBRW() {
  //Serial.println("ENCB Triggered on encoder 4");
  counter[3]++;
}


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

// Get command velocities from main controller via CAN bus
void readCmdVel(){

  // check for incoming CAN data. If there is, parse the data packet
  CANIndex = 0;
  int packetSize = CAN.parsePacket();

  // If there is CAN data parsed, assign them to the data array accordingly
  if (packetSize) {
    while (CAN.available()) {
      data[CANIndex] = CAN.read();
      canID = CAN.packetId();
      CANIndex++;

      // Uncomment the line below to show the data received

      // for (int i = 0; i < packetSize; i++){
      //   Serial.print(data[i]);
      //   Serial.print(" ");
      // }
      // Serial.println();
    }
  
    for (int i = 0; i < noOfMotors; i++){
      // if you are using those bldc motors purchased on Taobao
      // They have a deadzone where the input within that deadzone will not move the motor
      // so we need to offset it
      target[i] = map(data[i], 0, 255, motorMinPWM, 255); 
      mappedTarget[i] =  map(abs(target[i]), motorMinPWM, 255, 0, motorMaxRPM);
      direction[i] = ((data[i + 4] == 0) ? false : true);      
    }
    gotCmdVelData = true;
  }

  // No CAN data
  else{
    gotCmdVelData = false;
  }
}

// Open loop control of motor
// Good for testing whether your motors are responding to your command velocities correctly or not
// before you proceed to closed loop control
void openLoopMotorCtrl(){
  for (int i = 0; i < noOfMotors; i++) {
    if (!direction[i]) {
      digitalWrite(DIR[i], HIGH);
    } else {
      digitalWrite(DIR[i], LOW);
    }
    ledcWrite(i, abs(target[i]));
  }
}

// Closed loop control of motors
// To ensure the motor speeds are being driven at the right speed
void closedLoopMotorCtrl(){

  int getCounter[noOfMotors] = { 0, 0, 0, 0 };
  static int prevCounter[noOfMotors] = { 0, 0, 0, 0 };
  
  for (int i = 0; i < noOfMotors; i++) {
    unsigned long currMillis = micros();
    // Indicate that we have performed PID control for all motors for this round
    if (i >= 2){
      allMotorsDone = true;
    }

    // Run the PID control within a timeframe
    if (currMillis - prevPIDMillis[i] >= PID_TIMEFRAME){
      noInterrupts();
      getCounter[i] = counter[i];
      interrupts();

      // Calculate how many pulses have been recorded per second
      clCtrlTimeElapsed[i] = ((float) (currMillis - prevCLCtrlTime[i])) / 1.0e6;
      velRaw[i] = (getCounter[i] - prevCounter[i]) / clCtrlTimeElapsed[i];  // get the speed in terms of pulses per sec
      prevCLCtrlTime[i] = currMillis;
      prevCounter[i] = getCounter[i];

      // Compute motor velocity in terms of rpm
      velCalc[i] = velRaw[i] / motorConstant * 60.0; // (raw vel in pulses / s) / ([constant] 1 output shaft / 20 pulses) * 60 seconds

      // Begin PID control
      velError[i] = mappedTarget[i] - velCalc[i]; // find the error

      proportional[i] = Kp[i] * velError[i];
      integral[i] += Ki[i] * (velError[i] * clCtrlTimeElapsed[i]);
      derivative[i] = Kd[i] * ((velError[i] - velLastError[i]) / clCtrlTimeElapsed[i]);

      // Get the output and remap it to RPM
      output[i] = proportional[i] + integral[i] + derivative[i];
      mappedOutput[i] = map(output[i], 0, motorMaxRPM, motorMinPWM, 255);

      // If the input is already zero but the output still has some noise or oscillations
      // force it to zero
      if (target[i] == motorMinPWM && mappedOutput[i] != motorMinPWM){
        integral[i] = 0;
        proportional[i] = 0;
        derivative[i] = 0;
        mappedOutput[i] = 0;
      }

      // Output the calculated speed values to each motor
      // Direction output
      if (!direction[i]) {
        digitalWrite(DIR[i], HIGH);
      }
      else {
        digitalWrite(DIR[i], LOW);
      }

      // Speed output
      ledcWrite(i, mappedOutput[i]);

      // Overwrite the previous error and current error
      velLastError[i] = velError[i];

      // Restart the noon-blocking delay for PID control
      prevPIDMillis[i] = currMillis;


      base.deviceID = BASE_TELEM;
      base.targetRPM[i] = mappedTarget[i];
      base.measuredRPM[i] = velCalc[i];
    }
  }

  if (allMotorsDone){
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&base, sizeof(base));
    allMotorsDone = false;
  }

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
  CAN.setPins(RX, TX);  // Set the CAN pins to communicate with the CAN Tranceiver

  // start the CAN bus at 125 kbps
  if (!CAN.begin(125E3)) {
    Serial.println("Starting CAN failed!");
    while (1)
      ;  // forever loop if fails, you need to reset the microcontroller
  }

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

  // ==========
  //    GPIO
  // ==========
  // setup encoders and motors code
  for (int i = 0; i < noOfMotors; i++) {
    pinMode(ENCA[i], INPUT_PULLUP);
    pinMode(ENCB[i], INPUT_PULLUP);
    pinMode(PWM[i], OUTPUT);
    pinMode(DIR[i], OUTPUT);

    ledcSetup(i, 5000, 8);
    ledcAttachPin(PWM[i], i);
  }

  // Attach interrupts to motor encoder pins
  attachInterrupt(ENCA[0], readEncoderFLW, RISING);
  attachInterrupt(ENCA[1], readEncoderFRW, RISING);
  attachInterrupt(ENCA[2], readEncoderBLW, RISING);
  attachInterrupt(ENCA[3], readEncoderBRW, RISING);

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

    // If the CAN data is meant for open loop control
    if (canID == 1) {
      openLoopMotorCtrl();
    }

    // If the CAN data is meant for closed loop control
    else if (canID == 2){
      closedLoopMotorCtrl();
    }
  }

  // Safety feature where if there is no CAN data received, turn off the motors
  else {
    if(currMillis - prevDCTimeoutMillis >= DC_TIMEFRAME){
      Serial.println("No CAN data received. Stopping motors!");
      for (int i = 0; i < noOfMotors; i++){
        ledcWrite(i, motorMinPWM);
      }
    }
  }
}
