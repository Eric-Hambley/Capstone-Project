// Warning only use this for hardware debug!
// See which addresses respond to a start condition.

#include <I2cMaster.h>

#define SCL_PIN 14

SoftI2cMaster rtc(SDA_PIN, SCL_PIN);

//------------------------------------------------------------------------------
void setup(void) {
  Serial.begin(9600);

  uint8_t add = 0;

  // try read
  do {
    if (rtc.start(add | I2C_READ)) {
      Serial.print("Add read: ");
      Serial.println(add, HEX);
      rtc.read(true);
    }
    rtc.stop();
    add += 2;
  } while (add);

  // try write
  add = 0;
  do {
    if (rtc.start(add | I2C_WRITE)) {
      Serial.print("Add write: ");
      Serial.println(add, HEX);
    }
    rtc.stop();
    add += 2;
  } while (add);

  Serial.println("Done");
}
void loop(void){}
