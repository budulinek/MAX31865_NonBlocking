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

void MAX31865::begin(RtdWire wires, FilterFreq filter, ConvMode mode) {
  _spi->begin();
  pinMode(_cs, OUTPUT);

  setThresholds(0, 0xFFFF);
  clearFault();
  setConfig(0x00);

  setWires(wires);
  setFilter(filter);
  setConvMode(mode);
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

MAX31865::RtdWire MAX31865::getWires() {
  return MAX31865::RtdWire((getConfig() & CONFIG_3WIRE_RTD_BIT) == true);
}

void MAX31865::setFilter(const FilterFreq filter) {
  uint8_t t = getConfig();
  if (t & CONFIG_CONVERSION_MODE_BIT) return;  // Do not change the notch frequency while in continuous conversion mode.
  if (filter == FILTER_50HZ) {
    t |= CONFIG_FILTER_BIT;
  } else {
    t &= ~CONFIG_FILTER_BIT;
  }
  setConfig(t);
}

MAX31865::FilterFreq MAX31865::getFilter() {
  return MAX31865::FilterFreq((getConfig() & CONFIG_FILTER_BIT) == true);
}

void MAX31865::setConvMode(ConvMode mode) {
  uint8_t t = getConfig();
  if (mode == CONV_MODE_CONTINUOUS) {
    t |= CONFIG_CONVERSION_MODE_BIT;  // enable continuous conversion
    t |= CONFIG_VBIAS_BIT;            // enable bias
  } else {
    t &= ~CONFIG_CONVERSION_MODE_BIT;  // disable continuous conversion
    t &= ~CONFIG_VBIAS_BIT;            // disable bias
  }
  setConfig(t);
}

MAX31865::ConvMode MAX31865::getConvMode() {
  return MAX31865::ConvMode((getConfig() & CONFIG_CONVERSION_MODE_BIT) == true);
}

void MAX31865::setBias(bool b) {
  uint8_t t = getConfig();
  if (b) {
    t |= CONFIG_VBIAS_BIT;  // enable bias
  } else {
    t &= ~CONFIG_VBIAS_BIT;  // disable bias
  }
  setConfig(t);
}

bool MAX31865::getBias() {
  return getConfig() & CONFIG_VBIAS_BIT;
}

void MAX31865::setConfig(uint8_t cfg_reg) {
  writeRegister8(CONFIG_ADDR, cfg_reg);
}

uint8_t MAX31865::getConfig() {
  return readRegister8(CONFIG_ADDR);
}

bool MAX31865::isConversionComplete() {
  if (getConvMode() == MAX31865::CONV_MODE_CONTINUOUS) return true;
  switch (_state) {
    case 0:
      {
        // clear FAULTSTAT register
        clearFault();
        // enable bias
        setBias(true);
        _chrono = millis();
        _state++;
      }
      break;
    case 1:
      {
        if ((uint32_t)(millis() - _chrono) > TIMEOUT_VBIAS) {
          // run fault detection cycle with automatic delay
          setFaultCycle(MAX31865::FAULT_AUTO_RUN);
          _state++;
        }
      }
      break;
    case 2:
      {
        if (getFaultCycle() == MAX31865::FAULT_STATUS_FINISHED) {
          // trigger single shot conversion
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
          // disable bias
          setBias(false);
          // conversion is finished, reset the state and return true
          _state = 0;
          return true;
        }
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

uint16_t MAX31865::getLowerThreshold() {
  return readRegister16(LOW_FAULT_THRESH_MSB_ADDR);
}

uint16_t MAX31865::getUpperThreshold() {
  return readRegister16(HIGH_FAULT_THRESH_MSB_ADDR);
}

void MAX31865::setFaultCycle(FaultCycle fault_cycle) {
  uint8_t cfg_reg = getConfig();
  cfg_reg &= 0x11;  // mask out wire and filter bits
  switch (fault_cycle) {
    case FAULT_AUTO_RUN:
      setConfig(cfg_reg | 0b10000100);
      break;
    case FAULT_MANUAL_RUN:
      setConfig(cfg_reg | 0b10001000);
      break;
    case FAULT_MANUAL_FINISH:
      setConfig(cfg_reg | 0b10001100);
      break;
    default:
      break;
  }
}

MAX31865::FaultCycleStatus MAX31865::getFaultCycle() {
  uint8_t t = getConfig();
  t &= CONFIG_FAULT_CYCLE_MASK;
  t >>= 2;
  return MAX31865::FaultCycleStatus(t);
}

uint8_t MAX31865::getFault() {
  return readRegister8(FAULT_STATUS_ADDR);
}

void MAX31865::clearFault() {
  uint8_t t = getConfig();
  t &= ~CONFIG_1SHOT_BIT;         // Write a 0
  t &= ~CONFIG_FAULT_CYCLE_MASK;  // Write a 0
  t |= CONFIG_FAULT_CLEAR_BIT;    // Write a 1
  setConfig(t);
}

float MAX31865::getResistance(uint16_t rReference) {
  uint16_t rtdRaw = readRegister16(RTD_MSB_ADDR);

  // Remove fault bit. This bit is set to 1 if there is an error in FAULTSTAT,
  // so it does not provide any additional information.
  // Use getFault() to read the FAULTSTAT register.
  rtdRaw >>= 1;
  return static_cast<float>(rReference) * static_cast<float>(rtdRaw) / RTD_MAX_VAL;
}

float MAX31865::getTemperature(uint16_t rNominal, uint16_t rReference, MAX31865::CalcMethod method) {

  float Z1, Z2, Z3, Z4, Rt, temp;

  Rt = getResistance(rReference);

  if (method == MAX31865::CALC_LINEAR) {
    Rt /= static_cast<float>(rNominal);
    Rt *= 100.0f;
    return 2.57559f * Rt - 257.339f;  // optimal coeficient values for -40°C to +85°C
  }

  Z1 = -RTD_A;
  Z2 = RTD_A * RTD_A - (4 * RTD_B);
  Z3 = (4 * RTD_B) / static_cast<float>(rNominal);
  Z4 = 2 * RTD_B;
  temp = (Z1 + sqrtf(Z2 + (Z3 * Rt))) / Z4;

  if (method == MAX31865::CALC_POLY_ADVANCED) {
    if (temp < 0) {
      Rt /= static_cast<float>(rNominal);
      Rt *= 100.0f;
      float rpoly = Rt;
      temp = -242.02f;
      temp += 2.2228f * rpoly;
      rpoly *= Rt;  // square
      temp += 2.5859e-3f * rpoly;
      rpoly *= Rt;  // ^3
      temp -= 4.8260e-6f * rpoly;
      rpoly *= Rt;  // ^4
      temp -= 2.8183e-8f * rpoly;
      rpoly *= Rt;  // ^5
      temp += 1.5243e-10f * rpoly;

      // temp = -242.97f
      //        + 2.2838f * Rt
      //        + 1.4727e-3f * Rt * Rt;
    }
  }

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
