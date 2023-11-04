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

#ifndef MAX31865_NONBLOCKING_H
#define MAX31865_NONBLOCKING_H

#if defined(ARDUINO)
#include <Arduino.h>
#include "SPI.h"
#else
#include <cstddef>
#include <cstdint>
#include "core/core.h"
#endif

#define SPI_MAX31865_SETTINGS SPISettings(5000000, MSBFIRST, SPI_MODE1)

/*! Interface class for the MAX31865 RTD Sensor reader */
class MAX31865 {
public:
  /* Fault Status Masks */

  /** RTD High Threshold */
  static constexpr uint8_t FAULT_HIGHTHRESH_BIT = 0x80;
  /** RTD Low Threshold */
  static constexpr uint8_t FAULT_LOWTHRESH_BIT = 0x40;
  /** REFIN- > 0.85 x Bias */
  static constexpr uint8_t FAULT_REFINLOW_BIT = 0x20;
  /** REFIN- < 0.85 x Bias - FORCE- open */
  static constexpr uint8_t FAULT_REFINHIGH_BIT = 0x10;
  /** RTDIN- < 0.85 x Bias - FORCE- open */
  static constexpr uint8_t FAULT_RTDINLOW_BIT = 0x08;
  /** Under/Over voltage */
  static constexpr uint8_t FAULT_OVUV_BIT = 0x04;

  /**************************************************************************/
  /*!
   Number of wires we have in our RTD setup
  */
  /**************************************************************************/
  typedef enum {
    RTD_2WIRE = 0,
    RTD_3WIRE = 1,
    RTD_4WIRE = 0
  } RtdWire;

  /**************************************************************************/
  /*!
   Notch frequency for the noise rejection filter.
   Choose 50Hz or 60Hz depending on the frequency of your power sources.
  */
  /**************************************************************************/
  typedef enum {
    /** Filter out 60Hz noise and its harmonics */
    FILTER_60HZ = 0,
    /** Filter out 50Hz noise and its harmonics */
    FILTER_50HZ = 1
  } FilterFreq;

  /**************************************************************************/
  /*!
   Conversion Mode.
   Choose continuous conversion mode or single shot mode.
  */
  /**************************************************************************/
  typedef enum {
    /** “Normally Off” mode. 1-shot conversions may be initiated from this mode. */
    CONV_MODE_SINGLE = 0,
    /** Continuous (automatic) conversion mode */
    CONV_MODE_CONTINUOUS = 1
  } ConvMode;

  /**************************************************************************/
  /*!
   Fault Detection Cycle Control
  */
  /**************************************************************************/
  typedef enum {
    /** Run fault detection with automatic delay */
    FAULT_AUTO_RUN,
    /** Run fault detection with manual delay (cycle 1) */
    FAULT_MANUAL_RUN,
    /** Finish fault detection with manual delay (cycle 2) */
    FAULT_MANUAL_FINISH
  } FaultCycle;

  /**************************************************************************/
  /*!
   Fault Detection Cycle Status
  */
  /**************************************************************************/
  typedef enum {
    /** Fault detection finished */
    FAULT_STATUS_FINISHED = 0,
    /** Automatic fault detection still running */
    FAULT_STATUS_AUTO_RUNNING = 1,
    /** Manual cycle 1 still running; waiting for user to write setFaultCycle(MAX31865::FAULT_MANUAL_FINISH) */
    FAULT_STATUS_MANUAL_CYCLE1 = 2,
    /** Manual cycle 2 still running */
    FAULT_STATUS_MANUAL_CYCLE2 = 3
  } FaultCycleStatus;

  /**************************************************************************/
  /*!
   Calculation Method
  */
  /**************************************************************************/
  typedef enum {
    /** Linear equation (good approximation for -40°C to +85°C) */
    CALC_LINEAR,
    /** Callendar-Van Dusen equation (high precision for -70°C to +850°C) */
    CALC_POLY_SIMPLE,
    /** Callendar-Van Dusen equation - advanced resolution (high precision for -200°C to +850°C) */
    CALC_POLY_ADVANCED,
  } CalcMethod;

  MAX31865() {}

  /**************************************************************************/
  /*!
    @brief Create the interface object using hardware SPI
    @param cs the SPI chip select pin to use
    @param spi the SPI device to use, default is SPI
  */
  /**************************************************************************/
  MAX31865(const uint8_t cs, SPIClass *spi = &SPI)
    : _spi(spi), _cs(cs) {}

  /**************************************************************************/
  /*!
    @brief Initialize the SPI interface and set the number of RTD wires used.
    @param wires Number of wires in enum format:
        @c RTD_2WIRE (default)
        @c RTD_3WIRE
        @c RTD_4WIRE
    @param filter Notch frequency for the noise rejection filter:
        @c FILTER_50HZ (default)
        @c FILTER_60HZ
    @param mode Conversion mode:
        @c CONV_MODE_CONTINUOUS
        @c CONV_MODE_SINGLE (default)
  */
  /**************************************************************************/
  void begin(RtdWire wires = RTD_2WIRE, FilterFreq filter = FILTER_50HZ, ConvMode mode = CONV_MODE_SINGLE);

  /**************************************************************************/
  /*!
    @brief How many wires we have in our RTD setup.
    @param wires Number of wires in enum format:
        @c RTD_2WIRE
        @c RTD_3WIRE
        @c RTD_4WIRE
  */
  /**************************************************************************/
  void setWires(RtdWire wires);

  /**************************************************************************/
  /*!
    @brief Reads number of configured wires.
    @return Number of wires in enum format:
        - @c RTD_2WIRE
        - @c RTD_3WIRE
        - @c RTD_4WIRE
  */
  /**************************************************************************/
  RtdWire getWires();

  /**************************************************************************/
  /*!
    @brief Choose notch frequency for the noise rejection filter.
    Do not change the notch frequency while in continuous conversion mode.
    @param filter Notch frequency for the noise rejection filter:
        @c FILTER_50HZ
        @c FILTER_60HZ
  */
  /**************************************************************************/
  void setFilter(FilterFreq filter);

  /**************************************************************************/
  /*!
    @brief Reads notch frequency for the noise rejection filter.
    @return Notch frequency for the noise rejection filter:
        - @c FILTER_50HZ
        - @c FILTER_60HZ
  */
  /**************************************************************************/
  FilterFreq getFilter();

  /**************************************************************************/
  /*!
    @brief Set the conversion mode.
    @param mode Conversion mode:
        @c CONV_MODE_CONTINUOUS: Conversions occur continuously at a 50/60Hz rate.
        Bias voltage remains on continuously, leading to self-heating of the RTD sensor.
        @c CONV_MODE_SINGLE: Conversions must be initiated by @ref isConversionComplete() function.
  */
  /**************************************************************************/
  void setConvMode(ConvMode mode);

  /**************************************************************************/
  /*!
    @brief Reads conversion mode.
    @return Conversion mode:
        @c CONV_MODE_CONTINUOUS
        @c CONV_MODE_SINGLE
  */
  /**************************************************************************/
  ConvMode getConvMode();

  /**************************************************************************/
  /*!
    @brief Enable the bias voltage on the RTD sensor
    @param b If true bias is enabled, else disabled
  */
  /**************************************************************************/
  void setBias(bool b);

  /**************************************************************************/
  /*!
    @brief Get the bias voltage setting
    @return True if bias is enabled
  */
  /**************************************************************************/
  bool getBias();

  /**************************************************************************/
  /*!
    @brief The function runs all stages of single shot resistance conversion
    (enables bias, runs fault cycle, triggers conversion, disables bias).
    Bias voltage is disabled between 1-shot conversions to minimize self-heating.
    @return True if single shot resistance conversion is complete, always true
    if conversion mode is set to CONV_MODE_CONTINUOUS.
  */
  /**************************************************************************/
  bool isConversionComplete();

  /**************************************************************************/
  /*!
    @brief Write the lower and upper values into the threshold fault
    register to values as returned by getRTD()
    @param lower raw lower threshold
    @param upper raw upper threshold
  */
  /**************************************************************************/
  void setThresholds(uint16_t lower, uint16_t upper);

  /**************************************************************************/
  /*!
    @brief Read the raw 16-bit lower threshold value
    @return The raw unsigned 16-bit value, NOT temperature!
  */
  /**************************************************************************/
  uint16_t getLowerThreshold();

  /**************************************************************************/
  /*!
    @brief Read the raw 16-bit lower threshold value
    @return The raw unsigned 16-bit value, NOT temperature!
  */
  /**************************************************************************/
  uint16_t getUpperThreshold();

  /**************************************************************************/
  /*!
    @brief Triggers the fault detection cycle (and also enables bias voltage)
    @param fault_cycle The fault cycle type to run:
        @c FAULT_AUTO_RUN (default): Run fault detection with automatic delay
        @c FAULT_MANUAL_RUN: Run fault detection with manual delay (cycle 1)
        @c FAULT_MANUAL_FINISH: Finish fault detection with manual delay (cycle 2)
  */
  /**************************************************************************/
  void setFaultCycle(FaultCycle fault_cycle = FAULT_AUTO_RUN);

  /**************************************************************************/
  /*!
    @brief Reads the fault detection cycle status
    @return Fault detection cycle status:
        @c FAULT_STATUS_FINISHED: Fault detection finished
        @c FAULT_STATUS_AUTO_RUNNING: Automatic fault detection still running
        @c FAULT_STATUS_MANUAL_CYCLE1: Manual cycle 1 still running
        waiting for user to write setFaultCycle(MAX31865::FAULT_MANUAL_FINISH)
        @c FAULT_STATUS_MANUAL_CYCLE2: Manual cycle 2 still running
  */
  /**************************************************************************/
  FaultCycleStatus getFaultCycle();

  /**************************************************************************/
  /*!
    @brief Read the raw 8-bit FAULTSTAT register
    @return The raw unsigned 8-bit FAULT status register
  */
  /**************************************************************************/
  uint8_t getFault();

  /**************************************************************************/
  /*!
    @brief Clear all faults in FAULTSTAT
  */
  /**************************************************************************/
  void clearFault();

  /**************************************************************************/
  /*!
    @brief Read the RTD sensor's resistance
    @param rReference Resistance of the reference resistor, usually
    430 or 4300
    @return Resistance in Ohm
  */
  /**************************************************************************/
  float getResistance(uint16_t rReference);

  /**************************************************************************/
  /*!
    @brief Read the temperature in °C from the RTD through calculation of the
    resistance. Uses
   http://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf
   technique
    @param rNominal Nominal resistance of the RTD sensor at 0°C, usually 100
    or 1000
    @param rReference Resistance of the reference resistor, usually
    430 or 4300
    @param method Calculation method:
        @c CALC_LINEAR: Linear equation
        (good approximation for -40°C to +85°C)
        @c CALC_POLY_SIMPLE (default): Callendar-Van Dusen equation
        (high precision for -70°C to +850°C)
        @c CALC_POLY_ADVANCED: Callendar-Van Dusen equation - advanced resolution
        (high precision for -200°C to +850°C)
    @return Temperature in °C
  */
  /**************************************************************************/
  float getTemperature(uint16_t rNominal, uint16_t rReference, MAX31865::CalcMethod method = CALC_POLY_SIMPLE);

private:
  /* Communication */
  SPIClass *_spi;
  uint8_t _cs;
  /* State Machine */
  uint8_t _state;
  uint32_t _chrono;
  /* Data */
  static constexpr uint32_t TIMEOUT_VBIAS = 15;  // bias voltage timeout in ms
  static constexpr float RTD_MAX_VAL = 32768.0f;
  static constexpr float RTD_A = 3.9083e-3;
  static constexpr float RTD_B = -5.775e-7;
  /* Registers */
  static constexpr uint8_t CONFIG_ADDR = 0x00;
  static constexpr uint8_t RTD_MSB_ADDR = 0x01;
  static constexpr uint8_t RTD_LSB_ADDR = 0x02;
  static constexpr uint8_t HIGH_FAULT_THRESH_MSB_ADDR = 0x03;
  static constexpr uint8_t HIGH_FAULT_THRESH_LSB_ADDR = 0x04;
  static constexpr uint8_t LOW_FAULT_THRESH_MSB_ADDR = 0x05;
  static constexpr uint8_t LOW_FAULT_THRESH_LSB_ADDR = 0x06;
  static constexpr uint8_t FAULT_STATUS_ADDR = 0x07;
  /* Config Masks */
  static constexpr uint8_t CONFIG_VBIAS_BIT = 0x80;
  static constexpr uint8_t CONFIG_CONVERSION_MODE_BIT = 0x40;
  static constexpr uint8_t CONFIG_1SHOT_BIT = 0x20;
  static constexpr uint8_t CONFIG_3WIRE_RTD_BIT = 0x10;
  static constexpr uint8_t CONFIG_FAULT_CYCLE_MASK = 0x0C;
  static constexpr uint8_t CONFIG_FAULT_CLEAR_BIT = 0x02;
  static constexpr uint8_t CONFIG_FILTER_BIT = 0x01;

  /**************************************************************************/
  /*!
    @brief Set the raw 8-bit CONFIG register
    @param cfg_reg raw 8-bit CONFIG value
  */
  /**************************************************************************/
  void setConfig(uint8_t cfg_reg);

  /**************************************************************************/
  /*!
    @brief Read the raw 8-bit CONFIG register
    @return The raw unsigned 8-bit CONFIG register
  */
  /**************************************************************************/
  uint8_t getConfig();

  void readRegisterN(const uint8_t addr, uint8_t *const data, const uint8_t count);

  uint8_t readRegister8(const uint8_t addr);
  uint16_t readRegister16(const uint8_t addr);

  void writeRegister8(const uint8_t addr, const uint8_t data);
};

#endif
