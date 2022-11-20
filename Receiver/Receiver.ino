#include <SPI.h>          // include SPI library since CAN and RF modules use SPI
#include <Enrf24.h>       // Include library for RF module
#include <nRF24L01.h>     // Include library for RF module
#include <mcp_can.h>      // Include library for RF module
#include <string.h>       // Include string library for "//Serial.print" command for testing

#define CAN0_INT  P1_4
#define CAN_CSn   P2_0
#define RF_CE     P2_3
#define RF_CSn    P2_4
#define RF_INT    P2_5

#define SAFETY_MODE        1     // how long before the reciever package goes into safety mode (seconds)
#define MESSAGE_FREQUENCY  50   // the rate at which the CAN messages are sent to the falcon controller (milliseconds)

MCP_CAN CAN0(CAN_CSn);                // Instantiate RF module and declare (CSn)
Enrf24 radio(RF_CE, RF_CSn, RF_INT);  // Instantiate RF module and declare pins (CE, CSn, INT)

byte data[8] = {0x00, 0x08, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00};          // Create variable for CAN message to be sent to the falcon controller
byte sleepData[8] = {0x00, 0x08, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00};     // Create variable for CAN message to be sent to the falcon controller when in sleep mode
uint32_t message = 0;                                                     // Create 64 bit variable for received RF module message

const uint8_t rxaddr[] = "0";     // Define "address" for wireless transmission
const long ADDR = 0x00EF0010;     // Define 29-bit CAN extended ID

unsigned long timeStamp = 0;
unsigned long temp = millis();
unsigned long missedTransmissions = 0;

#define LED_R P1_3
#define LED_G P1_2

void setup() {
  SPI.begin();                // Set SPI pins (MISO = P1_6, MOSI = P1_7, SCLK = P1_5)                                 //
  SPI.setDataMode(SPI_MODE0); // Set SPI mode to MODE0                    //These 2 lines may not be necessary,       // ARE THESE NECESSARY?
  SPI.setBitOrder(MSBFIRST);  // Set SPI mode to transmit MSB first       //more testing required.                    //

  CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_10MHZ);
  CAN0.setMode(MCP_NORMAL);      // Change to normal mode to allow messages to be transmitted/received

  radio.begin();                      // Setup RF module to defaults of 1Mbps, channel 0, max TX power
  radio.setRXaddress((void*)rxaddr);  // Set receive address to previously defined
  radio.setChannel(0);                // Set transmission channel to channel 0 (2400 MHZ?)
  radio.setSpeed(250000);             // Set RF transmission to 250kbps (increases range, but can be increased if necessary)

  radio.enableRX();  // Start listening from RF module for transmissions
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
}


void loop() {
  char inbuf[10];     // define variable for rf message reception
  if (radio.read(inbuf, 10)) {   // read 10 bytes from rf module and store in inbuf array
    missedTransmissions = 0;

    message = atol(inbuf);    // convert received char array (ASCII codes) into 64bit variable

    //removed the part of code that converts the long type to Can
    

  }


  temp = millis();
  if ((temp - timeStamp) > (MESSAGE_FREQUENCY-1)) { // code in here will run every MESSAGE_FREQUENCY milliseconds
    timeStamp = millis();
    missedTransmissions++;

    if (missedTransmissions > (SAFETY_MODE * 20)) {
      // Expected messages from TX but didn't get any
      digitalWrite(LED_R, HIGH);
      byte sndStat = CAN0.sendMsgBuf(ADDR, 1, 8, sleepData); // Send predetermined message from Elmers.h file
      delay(10);
      digitalWrite(LED_R, LOW);

    } else {
      // Normal operation
      digitalWrite(LED_G, HIGH);
      digitalWrite(LED_R, LOW);    //make sure the red LED is off
      byte sndStat = CAN0.sendMsgBuf(ADDR, 1, 8, data);     // create CAN message send to controller (Address, eID = 1, 8 data bytes, data)
      delay(10);
      digitalWrite(LED_G, LOW);
    }
  }
}
