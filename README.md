# MAX31865 NonBlocking

This is the Adafruit MAX31865 Arduino Library 

<a href="https://www.adafruit.com/products/3328"><img src="assets/image.jpg" height="300"/></a>

Tested and works great with the Adafruit Thermocouple Breakout w/MAX31865
   * http://www.adafruit.com/products/3328

These sensors use SPI to communicate, 4 pins are required to interface

Adafruit invests time and resources providing this open source code, please support Adafruit and open-source hardware by purchasing products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.

Modified by budulinek for everyone.

### Changes to the original Adafruit library:
- non-blocking automatic conversion mode
- non-blocking single shot conversion mode
- no software (bitbang) SPI, only hardware SPI
- uses standard SPI.h library
- modified API (function names), check source files for usage comments
- simplified temperature calculation (provides accurate results from -60°C up to 850°C)

For **single shot measurements**, use the `isConversionComplete()` function. The function runs all stages of resistance conversion (enables bias voltage, runs fault cycle, triggers conversion, reads RTD data, disables bias voltage) and returns true once the conversion is complete. See the MAX31865_Single example.

BSD license, check license.txt for more information
All text above must be included in any redistribution
