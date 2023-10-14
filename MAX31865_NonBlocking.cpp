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
#ifdef __AVR
#include <avr/pgmspace.h>
#elif defined(ESP8266)
#include <pgmspace.h>
#endif

#include <stdlib.h>

void MAX31865::begin(RtdWire wires, FilterFreq filter) {
  _spi->begin();
  pinMode(_cs, OUTPUT);

  setThresholds(0, 0xFFFF);
  clearFault();
  setConfig(0x00);

  setWires(wires);
  setFilter(filter);


  // Serial.print("config: ");
  // Serial.println(readRegister8(CONFIG_ADDR), HEX);
}

void MAX31865::setConfig(uint8_t cfg_reg) {
  writeRegister8(CONFIG_ADDR, cfg_reg);
}

uint8_t MAX31865::getConfig() {
  return readRegister8(CONFIG_ADDR);
}

void MAX31865::setWires(RtdWire wires) {
  uint8_t t = getConfig();
  if (wires == RTD_3WIRE) {
    t |= CONFIG_3WIRE_RTD_BIT;
  } else {
    // 2 or 4 wire
    t &= ~CONFIG_3WIRE_RTD_BIT;
  }
  setConfig(t);
}

MAX31865::RtdWire MAX31865::getWires(void) {
  return (MAX31865::RtdWire(getConfig() & CONFIG_3WIRE_RTD_BIT));
}

void MAX31865::setFilter(const FilterFreq filter) {
  uint8_t t = getConfig();
  if (t & CONFIG_CONVERSION_MODE_BIT) return;
  if (filter == FILTER_50HZ) {
    t |= CONFIG_FILTER_BIT;
  } else {
    t &= ~CONFIG_FILTER_BIT;
  }
  setConfig(t);
}

MAX31865::FilterFreq MAX31865::getFilter(void) {
  return (MAX31865::FilterFreq(getConfig() & CONFIG_FILTER_BIT));
}

void MAX31865::autoConvert(bool b) {
  _autoConvert = b;
  uint8_t t = getConfig();
  if (b) {
    t |= CONFIG_CONVERSION_MODE_BIT;  // enable continuous conversion
  } else {
    t &= ~CONFIG_CONVERSION_MODE_BIT;  // disable continuous conversion
  }
  setConfig(t);
}

bool MAX31865::isConversionComplete(void) {
  if (_autoConvert == true) return true;
  switch (_state) {
    case 0:
      {
        clearFault();
        uint8_t t = getConfig();
        t |= CONFIG_VBIAS_BIT;  // enable bias
        setConfig(t);
        _chrono = millis();
        _state++;
      }
      break;
    case 1:
      {
        if ((uint32_t)(millis() - _chrono) > TIMEOUT_VBIAS) {
          setFaultCycle(MAX31865::FAULT_AUTO);
          _state++;
        }
      }
      break;
    case 2:
      {
        if (getFaultCycle() == MAX31865::FAULT_STATUS_FINISHED) {
          uint8_t t = getConfig();
          t |= CONFIG_1SHOT_BIT;
          setConfig(t);
          _state++;
        }
      }
      break;
    case 3:
      {
        // The CONFIG_1SHOT_BIT is a self-clearing bit. But it clears to 0 only
        // when a new conversion result is available in the RTD Data Registers.
        if ((getConfig() & CONFIG_1SHOT_BIT) == false) {
          _rtd = readRegister16(RTD_MSB_ADDR);
          _state++;
        }
      }
      break;
    case 4:
      {
        uint8_t t = getConfig();
        t &= ~CONFIG_VBIAS_BIT;  // disable bias
        setConfig(t);
        _state = 0;
        return true;
      }
      break;
    default:
      break;
  }
  return false;
}

void MAX31865::setThresholds(uint16_t lower, uint16_t upper) {
  writeRegister8(LOW_FAULT_THRESH_LSB_ADDR, lower & 0xFF);
  writeRegister8(LOW_FAULT_THRESH_MSB_ADDR, lower >> 8);
  writeRegister8(HIGH_FAULT_THRESH_LSB_ADDR, upper & 0xFF);
  writeRegister8(HIGH_FAULT_THRESH_MSB_ADDR, upper >> 8);
}

uint16_t MAX31865::getLowerThreshold(void) {
  return readRegister16(LOW_FAULT_THRESH_MSB_ADDR);
}

uint16_t MAX31865::getUpperThreshold(void) {
  return readRegister16(HIGH_FAULT_THRESH_MSB_ADDR);
}

void MAX31865::setFaultCycle(FaultCycle fault_cycle) {
  if (fault_cycle) {
    uint8_t cfg_reg = getConfig();
    cfg_reg &= 0x11;  // mask out wire and filter bits
    switch (fault_cycle) {
      case FAULT_AUTO:
        setConfig(cfg_reg | 0b10000100);
        break;
      case FAULT_MANUAL_RUN:
        setConfig(cfg_reg | 0b10001000);
        break;
      case FAULT_MANUAL_FINISH:
        setConfig(cfg_reg | 0b10001100);
        break;
      case FAULT_NONE:
      default:
        break;
    }
  }
}

MAX31865::FaultCycleStatus MAX31865::getFaultCycle(void) {
  uint8_t t = getConfig();
  t &= CONFIG_FAULT_CYCLE_MASK;
  t >>= 2;
  return MAX31865::FaultCycleStatus(t);
}

uint8_t MAX31865::getFault(void) {
  return readRegister8(FAULT_STATUS_ADDR);
}

void MAX31865::clearFault(void) {
  uint8_t t = getConfig();
  t &= ~CONFIG_1SHOT_BIT;         // Write a 0
  t &= ~CONFIG_FAULT_CYCLE_MASK;  // Write a 0
  t |= CONFIG_FAULT_CLEAR_BIT;    // Write a 1
  setConfig(t);
}

float MAX31865::getResistance(uint16_t rReference) {
  uint16_t rtdRaw;
  if (_autoConvert == true) {
    _rtd = readRegister16(RTD_MSB_ADDR);
  }
  rtdRaw = _rtd;

  // Remove fault bit. This bit is set to 1 if there is an error in FAULTSTAT,
  // so it does not provide any additional information.
  // Use getFault() to read FAULTSTAT.
  rtdRaw >>= 1;

  return static_cast<float>(rReference) * static_cast<float>(rtdRaw) / RTD_MAX_VAL;
}

float MAX31865::getTemperature(uint16_t rNominal, uint16_t rReference) {

  float Z1, Z2, Z3, Z4, Rt, temp;

  Z1 = -RTD_A;
  Z2 = RTD_A * RTD_A - (4 * RTD_B);
  Z3 = (4 * RTD_B) / static_cast<float>(rNominal);
  Z4 = 2 * RTD_B;

  Rt = getResistance(rReference);

  /* Convert resistance to temperature */
  temp = (Z1 + sqrtf(Z2 + (Z3 * Rt))) / Z4;
  // if (temp < -12.5f) {
  //   Rt /= static_cast<float>(rNominal);
  //   Rt *= 100.0f;
  //   temp = -242.97f + 2.2838f * Rt + 1.4727e-3f * Rt * Rt;
  // }

  return temp;
}

/**********************************************/

uint8_t MAX31865::readRegister8(const uint8_t addr) {
  uint8_t ret = 0;
  readRegisterN(addr, &ret, 1);
  return ret;
}

uint16_t MAX31865::readRegister16(const uint8_t addr) {
  uint8_t buffer[2] = { 0, 0 };
  readRegisterN(addr, buffer, 2);
  uint16_t ret = buffer[0];
  ret <<= 8;
  ret |= buffer[1];
  return ret;
}

void MAX31865::readRegisterN(uint8_t addr, uint8_t *const data,
                             const uint8_t count) {
  addr &= 0x7F;  // make sure top bit is not set
  _spi->beginTransaction(SPI_MAX31865_SETTINGS);
#if defined(TEENSYDUINO)
  digitalWriteFast(_cs, LOW);
#else
  digitalWrite(_cs, LOW);
#endif
  _spi->transfer(addr);
  _spi->transfer(data, count);
#if defined(TEENSYDUINO)
  digitalWriteFast(_cs, HIGH);
#else
  digitalWrite(_cs, HIGH);
#endif
  _spi->endTransaction();
}

void MAX31865::writeRegister8(uint8_t addr, const uint8_t data) {
  addr |= 0x80;  // make sure top bit is set
  _spi->beginTransaction(SPI_MAX31865_SETTINGS);
#if defined(TEENSYDUINO)
  digitalWriteFast(_cs, LOW);
#else
  digitalWrite(_cs, LOW);
#endif
  _spi->transfer(addr);
  _spi->transfer(data);
#if defined(TEENSYDUINO)
  digitalWriteFast(_cs, HIGH);
#else
  digitalWrite(_cs, HIGH);
#endif
  _spi->endTransaction();
}
