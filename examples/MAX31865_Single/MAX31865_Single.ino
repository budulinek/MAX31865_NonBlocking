/***************************************************
  This is a library for the Adafruit PT100/P1000 RTD Sensor w/MAX31865

  Designed specifically to work with the Adafruit RTD Sensor
  ----> https://www.adafruit.com/products/3328

  This sensor uses SPI to communicate, 4 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  Modified by budulinek for everyone.

  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "MAX31865_NonBlocking.h"

// uses hardware SPI, just pass in the CS pin (mandatory) and SPI bus (optional)
// MAX31865 rtd(5, &SPI);
MAX31865 rtd(6);

// Resistance of the reference resistor. Check Rref resistor value on your module,
// should be 430 for PT100 and 4300 for PT1000
#define RREF 4300
// Nominal resistance of the RTD sensor at 0°C,
// use 100 for PT100, 1000 for PT1000
#define RNOMINAL 1000

void setup() {
  Serial.begin(115200);
  Serial.println("MAX31865 PT1000 Sensor Test!");

  /*!
    Default settings are:
     rtd.begin(MAX31865::RTD_2WIRE, MAX31865::FILTER_50HZ, MAX31865::CONV_MODE_SINGLE);
    You can set your configuration through begin, such as:
     rtd.begin(MAX31865::RTD_3WIRE)
    or later through setWires(), setFilter() or setConvMode() functions
  */
  rtd.begin();
}

void loop() {
  if (rtd.isConversionComplete() == true) {
    readSensor();

    delay(1000);
  }
}

// Read and print temperature and faults (non-blocking)
void readSensor() {
  Serial.print("Resistance = ");
  Serial.print(rtd.getResistance(RREF));
  Serial.println(" Ω");
  Serial.print("Temperature = ");
  Serial.print(rtd.getTemperature(RNOMINAL, RREF));
  Serial.println("°C");

  // Check and print any faults
  uint8_t fault = rtd.getFault();
  if (fault) {
    Serial.print("Fault 0x");
    Serial.println(fault, HEX);
    if (fault & MAX31865::FAULT_HIGHTHRESH_BIT) {
      Serial.println("RTD High Threshold");
    }
    if (fault & MAX31865::FAULT_LOWTHRESH_BIT) {
      Serial.println("RTD Low Threshold");
    }
    if (fault & MAX31865::FAULT_REFINLOW_BIT) {
      Serial.println("REFIN- > 0.85 x Bias");
    }
    if (fault & MAX31865::FAULT_REFINHIGH_BIT) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865::FAULT_RTDINLOW_BIT) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865::FAULT_OVUV_BIT) {
      Serial.println("Under/Over voltage");
    }
    rtd.clearFault();
  }
  Serial.println();
}
