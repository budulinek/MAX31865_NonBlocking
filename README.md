# MAX31865 NonBlocking

Based on the Adafruit MAX31865 Arduino Library

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
- lightweight
- uses standard SPI.h library, selectable SPI port
- modified API (check examples and source files for usage instructions)
- selectable temperature calculation method, default setting (simplified Callendar-Van Dusen equation) provides high precision for -70°C to +850°C.

For **single shot measurements**, use the `isConversionComplete()` function. The function runs all stages of resistance conversion (enables bias voltage, runs fault cycle, triggers conversion, disables bias voltage) and returns true once the conversion is complete. Once the function returns true, you can `getResistance()` or `getTemperature()`. See the MAX31865_Single example.

BSD license, check license.txt for more information
All text above must be included in any redistribution
