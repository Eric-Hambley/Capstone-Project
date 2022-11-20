#include <SPI.h>                  // include SPI library for CAN and RF modules
#include <Enrf24.h>               // Include library for RF module
#include <nRF24L01.h>             // Include library for RF module
#include <mcp_can.h>              // Include library for CAN module
#include <I2cMaster.h>            // Include library for Battery Charger module

#define PB        P1_0            // Define Pushbutton as Pin 1_0
#define AUX_PWR   P1_1            // Define Auxiliary PWR to Mosfet On/Off switch for the joystick as Pin 1_1
#define GR_LED    P1_2            // Define Green LED Power as Pin 1_2
#define R_LED     P1_3            // Define Red LED Power as Pin 1_3
#define CAN0_INT  P1_4            // Define Can Interrrupt as Pin 1_4
#define CAN_CSn   P2_0            // Define CAN Chip Select as Pin 2_0
#define SDA_PIN   P2_1            // Define SDA_Pin (battery babysitter) as P2_1
#define SCL_PIN   P2_2            // Define SCL_Pin (battery babysitter) as P2_2
#define RF_CE     P2_3            // Define RF Chip Enable as P2_3
#define RF_CSn    P2_4            // Define RF SPI Chip select as P2_4
#define RF_INT    P2_5            // Define RF module interrrupt as P 2_5

#define FUEL_GAUGE_SoC      0x1C  // Define memory in Battery Babysitter as SOC variable
#define FUEL_GAUGE_CURRENT  0x10  // Define memory in Battery Babysitter as current variable

int timeout_mins = 5;             // Timeout variable, enters sleep after set minute value
int pb_seconds = 5;               // Timeout variable, enters sleep after pushbutton held for set number of seconds
int soc_seconds = 1;              // Check state of charge after set time in seconds
int brightness = 5;               // Declare how bright the fading LED starts at while charging
int fadeAmount = 1;               // Set how many points to fade the LED by per cycle while charging

int current;                      // Stores value of current entering the battery
int soc;                          // Stores value of charge of battery from 0-100%
long unsigned int rxId;           // Received eID from  CAN module (not used)
unsigned char len = 0;            // Received data length from CAN module (not used)
unsigned char rxBuf[8];           // Received data from CAN module
unsigned int spout_x = 0;         // Variable to store current value of joystick X
unsigned int spout_z = 0;         // Variable to store current value of joystick Z

const int idleTimeout = (timeout_mins*60) / (50E-3); // Number of cycles required for timeout based on timeout_mins
const int pbTimeout = (pb_seconds) / (50E-3);        // Number of cycles required for push button timeout based on pb_seconds
const int readSOC = (soc_seconds) / (50E-3);         // Number of cycles required to check state of charge based on soc_seconds
const int battHigh = 66;                             // High battery threshold, above this value the LED displays green when checking SOC
const int battLow = 33;                              // Low Battery threshold, if below, LED displays RED and if less than 66 and greater than 33 LED displays yellow
int buttonState = 0;                                 // Variable to store state of pushbutton, when the button is pressed, buttonState is high (1)
int minSOC = 10;                                     // Variable to put modules to sleep if battery has low charge.
int greenvalue = 0;                                  // PWM value to decrease intensity of green LED
int ledState = LOW;                  // State of LED Off=LOW, ON = HIGH
unsigned long previousMillis = 0;    // previous value of milliseconds on previous loop
unsigned long interval;              // Millisecond value of interval to hold LED on/off while checking state of charge
unsigned long interval_LO = 2000;    // Millisecond value of interval to hold LED ON while LED flashing for normal operation
unsigned long interval_HI = 200;     // Millisecond value of interval to hold LED ON while LED flashing for normal operation
unsigned long interval_hold = 3000;  // Millisecond value of interval to hold LED ON while checking battery state of charge

// Transmitter variable declarations
const uint8_t txaddr[] = "0"; // Define "address" for wireless transmission
const uint8_t rxaddr[] = "0"; // Define "address" for wireless transmission

SoftI2cMaster wire(SDA_PIN, SCL_PIN);// Pin declaration of I2cMaster function to check battery current and state of charge
MCP_CAN CAN0(CAN_CSn);               // Instantiate RF module and declare (CSn)
Enrf24 radio(RF_CE, RF_CSn, RF_INT); // Instantiate RF module and declare pins (CE, CSn, INT)

void setup()
{
  pinMode(CAN0_INT, INPUT);                       // Configuring pin for CAN Interrupt input
  pinMode(AUX_PWR, OUTPUT);                       // Configure MOSFET switch driver as ouput
  pinMode(GR_LED, OUTPUT);                        // Configure green LED as output
  pinMode(R_LED, OUTPUT);                         // Configure green LED as output
  pinMode(PB, INPUT_PULLUP);                      // Make push button input
  CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_10MHZ);    // Setup CAN module for our specific bit rate and oscillator frequency
  CAN0.setMode(MCP_NORMAL);                       // Change to normal mode to allow messages to be transmitted/received
  CAN0.setSleepWakeup(1);                         // Enable Can module sleep mode
  radio.begin();                                  // Setup RF module to defaults of 1Mbps, channel 0, max TX power
  radio.setTXaddress((void*)txaddr);              // Set transmit address for data
  radio.setChannel(0);                            // Set transmission channel to channel 0 (2400 MHZ)
  radio.setSpeed(250000);                         // Set RF transmission to 250kbps (increases range, but can be increased if necessary)
  radio.setTXpower(-12);                          // Set transmission power to -12 dBm

}

void loop()
{
  int timeout = 0;                // Loop value for idle timeout counter
  int btnPush = 0;                // Counter of subsequent button presses for module timeout
  int btnSoc = 0;                 // Counter of subsequent button presses to check battery state of charge
  int zero_counter = 0;           // Counts subsequent zeros to confirm that the battery is disconnected
  int soc_flag = 0;               // Flag is set high once btnPush exceeds the readSOC variable.
  int soc_counter = 0;            // Checks how many consequtive cycles that soc < minSOC.
  interval = 0;                   // Chosen interval in milliseconds of millis function
  soc=minSOC+10;                  // Initialize soc to greater than minSOC to be able to enter operational while loop
  digitalWrite(AUX_PWR, HIGH);    // Power mosfet, which supplies power to the joystick


  //****************************************************************************
  //*****The functions elmersCANToLong and noJoystickInput were removed*****
  //as this is proprietary info.
  //
  //elmersCANToLong - Converts the CAN message into a transmittable format,
  //                  shifts a few bits for security, these bits are reshifted
  //                  in the opposite direction in the wireless receiver code.
  //noJoystickInput - Checks for valid user input. Has a threshold that accounts
  //                  for random vibrations as non valid input (through testing)
  //****************************************************************************


  while (timeout < idleTimeout && btnPush < pbTimeout && zero_counter <= 50 && soc_counter < 10)
  {
    //If any of these arguments are invalid, module goes to sleep. While in this loop, module is transmitting.
    //1st arg: Do not enter loop if joystick is idle for set time (timeout)
    //2nd arg: Do not enter loop if button is pushed for the sleep threshold time
    //3rd arg: zero_counter checks if current is zero for 50 loops (batt is disconnected in this case)
    //4th arg: Checks if state of charge is less than minSOC for 10 loops.

    if (!digitalRead(CAN0_INT))   // If CAN0_INT pin is low, read receive buffer of CAN module
    {
      CAN0.readMsgBuf(&rxId, &len, rxBuf);            // Read data from CAN module and store data in rxBuf array
      unsigned long message = elmersCANToLong(rxBuf); //bit shift operation for a wireless security protocol.
      radio.println(message);     // Add message to RF transmit buffer
      radio.flush();              // Force transmission of message to receiver
      if (noJoystickInput(rxBuf)) timeout++; else timeout = 0; // Check if joystick had buttons pushed, function accounts for random vibrations
      buttonState = digitalRead(PB);
      if (buttonState == LOW)
      {
        btnPush++;                // Increment button push counter for sleep threshold counter
        btnSoc++;                 // Increment button push counter for state of charge counter
        if (btnSoc >= readSOC){   // Soc_flag is used to toggle the soc check.
          soc_flag = 1;
        }
      }
      else{                       // If no subsequent button press, set both counters to zero
        btnPush=0;
        btnSoc=0;
      }
    }
    soc = wire.fuelGaugeRead(FUEL_GAUGE_SoC);           // i2cmaster function that retreives battery state of charge 0-100%
    current = wire.fuelGaugeRead(FUEL_GAUGE_CURRENT);   // i2cmaster function that retreives current entering the battery
    if (current == 0){                                  //Checks how many times current is zero subsequently, this is to assure our battery is disconnected (current = 0)
      zero_counter++;
    }
    else{
      zero_counter = 0;
    }
    if (soc < minSOC){
      soc_counter++;
    }
    else{
     soc_counter = 0;
    }
    if (current > 0) {                      // LED is charging in this state, LED fades.
      analogWrite(GR_LED, brightness);      // Declare analog input "brightness" to allow fading
      analogWrite(R_LED, LOW);              // Turn red LED off, not needed in this state
      brightness = brightness + fadeAmount; //increment LED brightness by fade value during each loop.
                                            // reverse the direction of the fading at the ends of the fade:
      if (brightness == 0 || brightness == 150) {
        fadeAmount = -fadeAmount ;
      }
      timeout = 0;                          // reset timeout so module never goes to sleep while idle and charging.
    }
    else { //LED not charging in this state, this checks the battery state of charge,
      if (soc_flag == 1 && current < 0)
      {
        //1st arg: Enters if flag is high, (previously set when pushbutton is pressed down for appropriate number of cycles)
        //2nd arg: Enters if battery is not being charged, which means current is less than zero.
        interval = interval_hold; // switch interval so that LED stays on for 5 secs to check battery charge.
        unsigned long currentMillis = millis(); //declare most recent millisecond value of most recent loop
        if (currentMillis - previousMillis > interval)//if the subtraction is greater than "interval" we enter the if statement, this is our LED on/off time.
        {
          previousMillis = currentMillis;
          if (ledState == LOW) {
            ledState = HIGH;
            greenvalue = 30;
          }
          else {
            ledState = LOW;
            soc_flag = 0; //set flag to zero after checking state of charge
            greenvalue = 0;
          }

          if (soc < battLow) { //LED is red for 5 secs if state of charge is below our battery low threshold.
            digitalWrite(R_LED, ledState);
          }
          else if (soc > battHigh) { //LED is green for 5 secs if state of charge is above our battery high threshold.
            digitalWrite(GR_LED, ledState);
          }
          else { //if state of charge is either the above, LED is yellow for 5 seconds to display mid battery life.
            digitalWrite(R_LED, ledState);
            analogWrite(GR_LED, greenvalue);
          }
        }
      }
      else { //LED not charging in this state "Normal operation state" LED flashes on for 200ms and off for 2000ms
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillis > interval)
        {
          previousMillis = currentMillis;
          if (ledState == LOW) {
            ledState = HIGH;        // toggle state variable for LED high
            interval = interval_HI; // swap interval to small value so LED is on only briefly
          }
          else {
            ledState = LOW;         // toggle state variable for LED Low
            interval = interval_LO; // swap interval to large value so LED is off for a large period of time
          }
          digitalWrite(R_LED, LOW); // Ensure red LED is off
          digitalWrite(GR_LED, ledState); // turn LED off or on depending on state variable set previously.
        }
      }
    }
  }
  digitalWrite(GR_LED, LOW);                              // Turn Red and Green LED off
  digitalWrite(R_LED, LOW);
  CAN0.setMode(MCP_SLEEP);                                // Put CAN module to sleep
  Serial.flush();
  radio.deepsleep();                                      // Put RF module to sleep
  digitalWrite(AUX_PWR, LOW);                             // Cut power from joystick
  zero_counter = 0;                                       // Reset zero counter
  soc_counter = 0;
  delay(3000);                                            // To debounce
  attachInterrupt(PB, interrupt, FALLING);                // Allow module to be woken up by pushbutton press
  suspend();                                              // Enter sleep mode                                           // Enter sleep mode
}

void interrupt()
{
  wakeup();                                               // Wake up from sleep mode
  digitalWrite(AUX_PWR, HIGH);                            // Apply constant power to joystick
  radio.enableRX();                                       // Wake up RF module  - NEEDS TO BE TESTED
  CAN0.MCPwakeup();                                       // Wake up CAN Module
  CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_10MHZ);
  CAN0.setMode(MCP_NORMAL);
  detachInterrupt(PB);                                    // Remove pusbutton wake up interrupt
}
