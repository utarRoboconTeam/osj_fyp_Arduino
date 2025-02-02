
// ||==============================||
// ||          LIBRARIES           ||
// ||==============================||
#include <CAN.h>


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


  
void canSendCtrlWord(uint16_t uint16_t controlWord){
  // DO NOT PUT NON-BLOCKING DELAY FOR THIS, LET IT RUN AS FAST AS IT CAN

  // we are writing to the driver, so set the identifier to the transmit id
  CAN.beginPacket(TX_ID);  

  // Command word: write 2 bytes
  CAN.write(0x2B);

  // the index to send the data to
  CAN.write(0x40);  // low byte
  CAN.write(0x60);  // high byte

  // the subindex to send the data to
  CAN.write(0x00);

  // the data
  CAN.write((uint8_t)(controlWord & 0xFF)); // low byte
  CAN.write((uint8_t)(controlWord >> 8)); // high byte
  CAN.write(0x00);
  CAN.write(0x00);

  CAN.endPacket();         // end the data packet writing and send it    

}


void canSendSDO(int byteLength, uint16_t index, uint8_t subIndex, int32_t value){
  // DO NOT PUT NON-BLOCKING DELAY FOR THIS, LET IT RUN AS FAST AS IT CAN

  // we are writing to the driver, so set the identifier to the transmit id
  CAN.beginPacket(TX_ID);  
  //Serial.print("Beginning packet at: ");
  //Serial.println(TX_ID, HEX);
  
  // Command word, here we only specify the length of data to be sent
  switch(byteLength){
    case 1:
      CAN.write(0x2F);      
      break;
    case 2:
      CAN.write(0x2B);
      break;
    case 3:
      CAN.write(0x27);
      break;
    case 4:
      CAN.write(0x23);
      break;
    default:
      CAN.write(0x23);  // default it back to 4 bytes
      break;
  }

  //Serial.println("Indicate 4 bytes will be written: ");

  // the index to send the data to
  CAN.write((uint8_t)(index & 0xFF)); // low byte
  Serial.print("Writing lower byte of index");
  Serial.println(index & 0xFF, HEX);

  CAN.write((uint8_t)(index >> 8)); // high byte
  Serial.print("Writing higher byte of index");
  Serial.println(index >> 8, HEX);

  // the subindex to send the data to
  CAN.write(subIndex);
  //Serial.println("Selecting subindex");

  // the data
  CAN.write((uint8_t)(value & 0xFF)); // byte 1
  CAN.write((uint8_t)(value >> 8) & 0xFF); // byte 2
  CAN.write((uint8_t)(value >> 16) & 0xFF); // byte 3
  CAN.write((uint8_t)(value >> 24) & 0xFF); // byte 4
  //Serial.println("Writing the data");

  CAN.endPacket();         // end the data packet writing and send it    
  //Serial.println("Done. Sending packet...");


}


void pwmOnlyTest(){
  Serial.println("Start moving");
  canSendSDO(0x60FF, 0x01, 0x64);
  Serial.println("moving");
  delay(1000);
  Serial.println("Start stopping");
  canSendSDO(0x60FF, 0x01, 0x00);
  Serial.println("stop");
  delay(1000);
}

void dirOnlyTest(){
  canSendSDO(0x60FF, 0x00, -100);
}

void pwmSweepTest(){
  for (int i = 0; i < 100; i++){
    canSendSDO(0x60FF, 0x00, i);
    delay(10);
  }
  for (int i = 100; i > 0; i--){
    canSendSDO(0x60FF, 0x00, i);
    delay(10);
  }
}

void sharpDirectionChangeTest(){
  canSendSDO(0x60FF, 0x00, 100);
  delay(2000);
  canSendSDO(0x60FF, 0x00, -100);
  delay(2000);
}


void smoothDirectionChangeTest(){
  for (int i = 0; i < 100; i++){
    canSendSDO(0x60FF, 0x00, i);
    delay(10);
  }
  for (int i = 100; i > 0; i--){
    canSendSDO(0x60FF, 0x00, i);
    delay(10);
  }
  for (int i = 0; i > -100; i++){
    canSendSDO(0x60FF, 0x00, i);
    delay(10);
  }
  for (int i = -100; i < 0; i--){
    canSendSDO(0x60FF, 0x00, i);
    delay(10);
  }
}

void emergencyBrakeTest(){
  canSendSDO(0x60FF, 0x00, 100);
  delay(1000);
  canSendSDO(0x605A, 0x00, 0x07); // emergency brake
  delay(1000);
}


void driverStartup(){

  canSendCtrlWord(0x0006); // Shut down the driver and disable control to the motors
  canSendCtrlWord(0x0007); // Enable the driver and the motors
  canSendCtrlWord(0x000F); // Set the operation mode to speed control mode

  canSendSDO(0x6060, 0x00, 0x03); // Speed mode operation is selected

  canSendSDO(0x6083, 0x00, 100); // Acceleration time (100 ms)
  canSendSDO(0x6084, 0x00, 100); // Deceleration time (100 ms)






}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // ==============
  //    CAN Bus 
  // ==============
  CAN.setPins(RX, TX);  // Set the CAN pins to communicate with the CAN Tranceiver

  // start the CAN bus at 500 kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    // forever loop if fails, you need to reset the microcontroller
    // the onboard LED will repeatedly blink every second
    while (1){
    }  
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
  Serial.println("a cycle done!");
  
}
