
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

#define TX_ID 0x602
#define RX_ID 0x582

// ||==============================||
// ||           VARIABLES          ||
// ||==============================||
CanFrame txFrame = { 0 };
CanFrame rxFrame;
  
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
  static int32_t left_velocity = 0;
  static int32_t right_velocity = 0;
  if(ESP32Can.readFrame(rxFrame, 1000)) {
    // if (rxFrame.data[3] == 1){
    //   left_velocity = (rxFrame.data[7] << 24) | (rxFrame.data[6] << 16) | (rxFrame.data[5] << 8) | (rxFrame.data[4]);
    // }
    // else if (rxFrame.data[3] == 2){
    //   right_velocity = (rxFrame.data[7] << 24) | (rxFrame.data[6] << 16) | (rxFrame.data[5] << 8) | (rxFrame.data[4]);
    // }
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
      Serial.println(" ");
      // Serial.print("Left Wheel Speed: ");
      // Serial.print(left_velocity * 0.1);
      // Serial.print(" Right Wheel Speed: ");
      // Serial.println(right_velocity * 0.1);
  }
}


void pwmOnlyTest(){
  Serial.println("Start moving");
  canSendSDO(4, 0x60FF, 0x01, 0x64);
  canSendSDO(4, 0x60FF, 0x02, 0x64);
  Serial.println("moving");
  delay(1000);
}

void driverStartup(){
  // canSendSDO(2, 0x200A, 0x00, 0x02);
  // canReceiveSDO();
  // delay(10);
  // canSendSDO(2, 0x2010, 0x00, 0x01);
  // canReceiveSDO();
  // delay(10);
  // canSendSDO(-1, 0x6041, 0x00, 0x00); // Request motor state
  // canReceiveSDO();
  // delay(10);
  canSendSDO(-1, 0x200A, 0x00, 0x00); // Request motor state
  canReceiveSDO();
  delay(10);
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
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
