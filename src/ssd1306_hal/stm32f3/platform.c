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

#include "ssd1306_hal/io.h"

#if defined(SSD1306_STM32F3_PLATFORM)

#include "intf/ssd1306_interface.h"
#include "stm32f3xx_hal.h"
#include "periph_pin_map.h"

////////////////////////////////////////////////////////////////////////////////////////
// !!! PLATFORM I2C IMPLEMENTATION OPTIONAL !!!
#if defined(CONFIG_PLATFORM_I2C_AVAILABLE) && defined(CONFIG_PLATFORM_I2C_ENABLE)

static STM32F3_I2C_PinMap s_i2c_pinmap;
static uint16_t s_i2c_addr = 0x3C;
static I2C_HandleTypeDef s_i2c_handle;
static uint32_t s_i2c_timeout_ms = 1000;

static uint8_t s_buffer[128];
static uint8_t s_dataSize = 0;

/* This function overwrites the weak symbol in the STM32F3xx_HAL_Driver (stm32f3xx_hal_i2c.c)
 * Don't rename it.
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    s_i2c_pinmap.gpio_clock_enable();
    s_i2c_pinmap.i2c_clock_enable();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = s_i2c_pinmap.sda_pin | s_i2c_pinmap.scl_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = s_i2c_pinmap.gpio_alternate_function;
    HAL_GPIO_Init(s_i2c_pinmap.gpio_port, &GPIO_InitStruct);
}

/* This function overwrites the weak symbol in the STM32F3xx_HAL_Driver (stm32f3xx_hal_i2c.c)
 * Don't rename it.
 */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
    s_i2c_pinmap.i2c_clock_disable();
    HAL_GPIO_DeInit(s_i2c_pinmap.gpio_port, s_i2c_pinmap.sda_pin);
    HAL_GPIO_DeInit(s_i2c_pinmap.gpio_port, s_i2c_pinmap.scl_pin);
}

static void platform_i2c_start(void)
{
    s_dataSize = 0;
}

static void platform_i2c_stop(void)
{
    while (HAL_I2C_Master_Transmit(&s_i2c_handle, (s_i2c_addr << 1), s_buffer, s_dataSize, s_i2c_timeout_ms) != HAL_OK)
    {
        if (HAL_I2C_GetError(&s_i2c_handle) != HAL_I2C_ERROR_NONE)
        {
            /* Some serious error happened, but we don't care. Our API functions have void type */
            return;
        }
    }
}

static void platform_i2c_send(uint8_t data)
{
    s_buffer[s_dataSize] = data;
    s_dataSize++;
    if (s_dataSize == sizeof(s_buffer))
    {
        /* Send function puts all data to internal buffer.  *
         * Restart transmission if internal buffer is full. */
        ssd1306_intf.stop();
        ssd1306_intf.start();
        ssd1306_intf.send(0x40);
    }
}

static void platform_i2c_close(void)
{
    HAL_I2C_MspDeInit(&s_i2c_handle);
}

static void platform_i2c_send_buffer(const uint8_t *data, uint16_t len)
{
    while (len--)
    {
        platform_i2c_send(*data);
        data++;
    }
}

void ssd1306_platform_i2cInit(int8_t busId, uint8_t addr, ssd1306_platform_i2cConfig_t *cfg)
{
    if (addr)
        s_i2c_addr = addr;
    ssd1306_intf.spi = 0;
    ssd1306_intf.start = &platform_i2c_start;
    ssd1306_intf.stop = &platform_i2c_stop;
    ssd1306_intf.send = &platform_i2c_send;
    ssd1306_intf.close = &platform_i2c_close;
    ssd1306_intf.send_buffer = &platform_i2c_send_buffer;

    if (cfg->sda != -1 || cfg->scl != -1)
        s_i2c_pinmap = s_i2c_pinmap_options[1];
    else
        s_i2c_pinmap = s_i2c_pinmap_options[0];

    s_i2c_handle.Init = (I2C_InitTypeDef){0};
    s_i2c_handle.Instance = s_i2c_pinmap.i2c_instance;
    s_i2c_handle.Init.Timing = 400000;
    s_i2c_handle.Init.OwnAddress1 = 0;
    s_i2c_handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    if (HAL_I2C_Init(&s_i2c_handle) != HAL_OK)
    {
        /* Some serious error happened, but we don't care. Our API functions have void type */
        return;
    }
}
#endif

////////////////////////////////////////////////////////////////////////////////////////
// !!! PLATFORM SPI IMPLEMENTATION OPTIONAL !!!
#if defined(CONFIG_PLATFORM_SPI_AVAILABLE) && defined(CONFIG_PLATFORM_SPI_ENABLE)

#include "intf/spi/ssd1306_spi.h"

static void platform_spi_start(void)
{
    // ... Open spi channel for your device with specific s_ssd1306_cs, s_ssd1306_dc
}

static void platform_spi_stop(void)
{
    // ... Complete spi communication
}

static void platform_spi_send(uint8_t data)
{
    // ... Send byte to spi communication channel
}

static void platform_spi_close(void)
{
    // ... free all spi resources here
}

static void platform_spi_send_buffer(const uint8_t *data, uint16_t len)
{
    // ... Send len bytes to spi communication channel here
}

void ssd1306_platform_spiInit(int8_t busId, int8_t cesPin, int8_t dcPin)
{
    if (cesPin >= 0)
        s_ssd1306_cs = cesPin;
    if (dcPin >= 0)
        s_ssd1306_dc = dcPin;
    ssd1306_intf.spi = 1;
    ssd1306_intf.start = &platform_spi_start;
    ssd1306_intf.stop = &platform_spi_stop;
    ssd1306_intf.send = &platform_spi_send;
    ssd1306_intf.close = &platform_spi_close;
    ssd1306_intf.send_buffer = &platform_spi_send_buffer;
    // init your interface here
    //...
}
#endif

#endif // SSD1306_STM32F3_PLATFORM
