
// ||==============================||
// ||          LIBRARIES           ||
// ||==============================||
#include <ESP32-TWAI-CAN.hpp>


// ||==============================||
// ||          DEFINITIONS         ||
// ||==============================||
// CAN bus pins
#define RX 23  // Connect to RX Pin of the CAN module
#define TX 32  // connect to TX Pin of the CAN module

#define TX_ID 0x601
#define RX_ID 0x581


#define noOfMotors 2

#define MODE_SELECT_WORD 0x6060
#define MODE_DISPLAY_WORD 0x6061
#define CONTROL_WORD 0x6040
#define STATUS_WORD 0x6041

// ||==============================||
// ||           VARIABLES          ||
// ||==============================||
enum whichTest {
  PWM_TEST,
  DIR_TEST,
  PWM_SWEEP,
  DIR_CHANGE_SHARP,  
  DIR_CHANGE_SMOOTH,
  E_BRAKE
};

int selectedTest = PWM_TEST;

uint8_t PWM[noOfMotors] = { 0, 0 };
uint8_t DIR[noOfMotors] = { 0, 0 };

long counter[noOfMotors] = { 0, 0 };

unsigned long prevSeqMillis = 0;

CanFrame txFrame = { 0 };
CanFrame rxFrame;
  
// void canSendCtrlWord(uint16_t uint16_t controlWord){
//   // DO NOT PUT NON-BLOCKING DELAY FOR THIS, LET IT RUN AS FAST AS IT CAN

//   // we are writing to the driver, so set the identifier to the transmit id
//   CAN.beginPacket(TX_ID);  

//   // Command word: write 2 bytes
//   CAN.write(0x2B);

//   // the index to send the data to
//   CAN.write(0x40);  // low byte
//   CAN.write(0x60);  // high byte

//   // the subindex to send the data to
//   CAN.write(0x00);

//   // the data
//   CAN.write((uint8_t)(controlWord & 0xFF)); // low byte
//   CAN.write((uint8_t)(controlWord >> 8)); // high byte
//   CAN.write(0x00);
//   CAN.write(0x00);

//   CAN.endPacket();         // end the data packet writing and send it    

// }


void canSendSDO(int byteLength, uint16_t index, uint8_t subIndex, int32_t value){
  // DO NOT PUT NON-BLOCKING DELAY FOR THIS, LET IT RUN AS FAST AS IT CAN

  // we are writing to the driver, so set the identifier to the transmit id
  txFrame.identifier = (TX_ID);

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


void pwmOnlyTest(){
  Serial.println("Start moving");
  canSendSDO(4, 0x60FF, 0x00, 0x64);
  Serial.println("moving");
  delay(1000);
  // Serial.println("Start stopping");
  // canSendSDO(2, 0x6040, 0x00, 0x00);
  // Serial.println("stop");
  // delay(1000);
}

void dirOnlyTest(){
  canSendSDO(4, 0x60FF, 0x00, -100);
}

void pwmSweepTest(){
  for (int i = 0; i < 100; i++){
    canSendSDO(4, 0x60FF, 0x00, i);
    delay(10);
  }
  for (int i = 100; i > 0; i--){
    canSendSDO(4, 0x60FF, 0x00, i);
    delay(10);
  }
}

void sharpDirectionChangeTest(){
  canSendSDO(4, 0x60FF, 0x00, 100);
  delay(2000);
  canSendSDO(4, 0x60FF, 0x00, -100);
  delay(2000);
}

void smoothDirectionChangeTest(){
  for (int i = 0; i < 100; i++){
    canSendSDO(4, 0x60FF, 0x00, i);
    delay(10);
  }
  for (int i = 100; i > 0; i--){
    canSendSDO(4, 0x60FF, 0x00, i);
    delay(10);
  }
  for (int i = 0; i > -100; i++){
    canSendSDO(4, 0x60FF, 0x00, i);
    delay(10);
  }
  for (int i = -100; i < 0; i--){
    canSendSDO(4, 0x60FF, 0x00, i);
    delay(10);
  }
}

void emergencyBrakeTest(){
  canSendSDO(4, 0x60FF, 0x00, 100);
  delay(1000);
  canSendSDO(2, 0x605A, 0x00, 0x07); // emergency brake
  delay(1000);
}

void driverStartup(){

  // canSendCtrlWord(0x0006); // Shut down the driver and disable control to the motors
  // canSendCtrlWord(0x0007); // Enable the driver and the motors
  // canSendCtrlWord(0x000F); // Set the operation mode to speed control mode
  
  canSendSDO(1, 0x2001, 0x00, 0x01); // Enable only the left motor
  canReceiveSDO();
  delay(10);


  // clear any potential faults
  Serial.println("Clearing any potential faults");
  canSendSDO(2, 0x6040, 0x00, 0x80);
  canReceiveSDO();
  delay(10);

  // check for any potential faults
  Serial.println("potential faults: ");
  canSendSDO(-1, 0x603F, 0x00, 0x00); // Request motor state
  canReceiveSDO();

  // disable quick stop
  Serial.println("disable quickstop");
  canSendSDO(2, 0x6040, 0x00, 0x0F); // Disable quick stop
  canReceiveSDO();
  delay(10);

  // disable quick stop
  Serial.println("disable quickstop againnnnnn");
  canSendSDO(2, 0x605A, 0x00, 0x00); // Disable quick stop
  canReceiveSDO();
  delay(10);

  canSendSDO(2, 0x200F, 0x00, 0x00); // set the driver to asynchronous movement
  canReceiveSDO();
  delay(10);

  canSendSDO(1, 0x6060, 0x00, 0x03); // select Speed mode operation 
  canReceiveSDO();
  delay(10);

  canSendSDO(4, 0x6083, 0x01, 0x64); // Acceleration time (100 ms) for left wheel
  canReceiveSDO();
  delay(10);

  canSendSDO(4, 0x6083, 0x02, 0x64); // Acceleration time (100 ms) for right wheel
  canReceiveSDO();
  delay(10);

  canSendSDO(4, 0x6084, 0x01, 100); // Deceleration time (100 ms) for left wheel
  canReceiveSDO();
  delay(10);

  canSendSDO(4, 0x6084, 0x02, 100); // Deceleration time (100 ms) for right wheel
  canReceiveSDO();
  delay(10);

  Serial.println("First status of motor driver: ");
  canSendSDO(-1, 0x6041, 0x00, 0x00); // Request motor state
  canReceiveSDO();
  delay(10);

  // clear any potential faults
  Serial.println("Clearing any potential faults");
  canSendSDO(2, 0x6040, 0x00, 0x80);
  canReceiveSDO();
  delay(10);

  // check for any potential faults
  Serial.println("potential faults: ");
  canSendSDO(-1, 0x603F, 0x00, 0x00); // Request motor state
  canReceiveSDO();

  // enable the driver
  canSendSDO(2, 0x6040, 0x00, 0x06);
  canReceiveSDO();
  delay(10);

  canSendSDO(2, 0x6040, 0x00, 0x07);
  canReceiveSDO();
  delay(10);

  canSendSDO(2, 0x6040, 0x00, 0x0F);
  canReceiveSDO();
  delay(10);


  Serial.println("Second status of motor driver: ");
  canSendSDO(-1, 0x6041, 0x00, 0x00); // Request motor state
  canReceiveSDO();

    // check for any potential faults
  Serial.println("potential faults: ");
  canSendSDO(-1, 0x603F, 0x00, 0x00); // Request motor state
  canReceiveSDO();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

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
  
  Serial.print("You have selected: ");
  switch(selectedTest){
    case PWM_TEST:
      Serial.println("PWM ONLY TEST ");
      break;
    case DIR_TEST:
      Serial.println("DIRECTION ONLY TEST ");
      break;
    case PWM_SWEEP:
      Serial.println("PWM SWEEP TEST ");
      break;
    case DIR_CHANGE_SHARP:
      Serial.println("SHARP DIRECTION CHANGE TEST ");
      break;
    case DIR_CHANGE_SMOOTH:
      Serial.println("SMOOTH DIRECTION CHANGE TEST ");
      break;
    case E_BRAKE:
      Serial.println("EMERGENCY BRAKE TEST ");
      break;
    default:
      Serial.println("UNKNOWN");
      break;
  }
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  switch(selectedTest){
    case PWM_TEST:
      pwmOnlyTest();
      break;
    case DIR_TEST:
      dirOnlyTest();
      break;
    case PWM_SWEEP:
      pwmSweepTest();
      break;
    case DIR_CHANGE_SHARP:
      sharpDirectionChangeTest();
      break;
    case DIR_CHANGE_SMOOTH:
      smoothDirectionChangeTest();
      break;
    case E_BRAKE:
      emergencyBrakeTest();
      break;
    default:
      Serial.println("Dafuq is this");
      break;
  }

  canReceiveSDO();
  Serial.println("a cycle done!");
  
}
