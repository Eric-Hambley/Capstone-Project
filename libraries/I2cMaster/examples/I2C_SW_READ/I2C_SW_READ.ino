// Warning only use this for hardware debug!
// See which addresses respond to a start condition.

#include <I2cMaster.h>

#define SCL_PIN P2_2
#define SDA_PIN P2_1

#define FUEL_GAUGE_SoC      0x1C
#define FUEL_GAUGE_VOLTAGE  0x04
#define FUEL_GAUGE_CURRENT  0x10

SoftI2cMaster wire(SDA_PIN, SCL_PIN);

//------------------------------------------------------------------------------
void setup(void) {
  Serial.begin(9600);

}
void loop(void) {
  int SoC = 0;
  int current = 0;
  int voltage = 0;

  SoC = wire.fuelGaugeRead(FUEL_GAUGE_SoC);
  current = wire.fuelGaugeRead(FUEL_GAUGE_CURRENT);
  voltage = wire.fuelGaugeRead(FUEL_GAUGE_VOLTAGE);

  Serial.print("State of Charge (%): ");
  Serial.println(SoC);
  Serial.print("Average current (mA): ");
  Serial.println(current);
  Serial.print("Voltage (mV): ");
  Serial.println(voltage);
  
  delay(5000);
}
