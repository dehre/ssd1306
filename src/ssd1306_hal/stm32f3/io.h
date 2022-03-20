/*
    MIT License

    Copyright (c) 2018, Alexey Dynda

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

/*
 * @file ssd1306_hal/template/io.h This is template file for new platform with detailed instructions
 */

#ifndef _SSD1306_STM32F3_IO_H_
#define _SSD1306_STM32F3_IO_H_

#define SSD1306_STM32F3_PLATFORM

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define LOW 0
#define HIGH 1
#define INPUT 0
#define OUTPUT 1
/* Progmem attribute for data, located in Flash */
#define PROGMEM

#define CONFIG_PLATFORM_I2C_AVAILABLE
//#define CONFIG_PLATFORM_SPI_AVAILABLE

#ifdef __cplusplus
extern "C"
{
#endif

  // clang-format off

// !!! MANDATORY. The library will not work without these functions !!!
static inline int digitalRead(int pin) // digitalRead()
{
    return LOW;
}

static inline void digitalWrite(int pin, int level) // digitalWrite()
{
}

static inline void pinMode(int pin, int mode) // pinMode()
{
}

static inline int analogRead(int pin) // analogRead()
{
    return 0;
}

static inline uint32_t millis(void) // millis()
{
    return 0;
}

static inline uint32_t micros(void) // micros()
{
    return 0;
}

static inline void delay(uint32_t ms) // delay()
{
}

static inline void delayMicroseconds(uint32_t us) // delayMicroseconds()
{
}

// !!!  OPTIONAL !!!
static inline void randomSeed(int seed) // randomSeed() -  can be skipped
{
}

static inline void attachInterrupt(int pin, void (*interrupt)(), int level) // attachInterrupt() - can be skipped
{
}

static inline uint8_t pgm_read_byte(const void *ptr) // pgm_read_byte() - can be left as is
{
    return *((const uint8_t *)ptr);
}

static inline uint16_t eeprom_read_word(const void *ptr) // eeprom_read_word() - can be skipped
{
    return 0;
}

static inline void eeprom_write_word(const void *ptr, uint16_t val) // eeprom_write_word() - can be skipped
{
}

// clang-format on

// utoa is already defined in stdlib.h
// static inline char *utoa(unsigned int num, char *str, int radix) // util utoa() - can be skipped
// {
//     *str = '\0';
//     return str;
// }

// !!! PLATFORM I2C IMPLEMENTATION OPTIONAL !!!
#if defined(CONFIG_PLATFORM_I2C_AVAILABLE) && defined(CONFIG_PLATFORM_I2C_ENABLE)
  void ssd1306_platform_i2cInit(int8_t busId, uint8_t addr, ssd1306_platform_i2cConfig_t *cfg);
#endif

// !!! PLATFORM SPI IMPLEMENTATION OPTIONAL !!!
#if defined(CONFIG_PLATFORM_SPI_AVAILABLE) && defined(CONFIG_PLATFORM_SPI_ENABLE)
  void ssd1306_platform_spiInit(uint8_t busId, uint8_t cesPin, uint8_t dcPin);
#endif

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
static inline int random(int max) // random(n) - can be skipped if you don't use it
{
  return 0;
}

static inline int random(int min, int max) // random(a,b) - can be skipped if you don't use it
{
  return 0;
}
#endif

#endif
