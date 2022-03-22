#ifndef _SSD1306_STM32F3_PERIPH_PIN_MAP_H_
#define _SSD1306_STM32F3_PERIPH_PIN_MAP_H_

/* All STM32F3 targets make available PB6-PB7 for I2C1, and PA9-PA10 for I2C2 (when available). Source:
 * https://github.com/stm32duino/Arduino_Core_STM32/blob/e156c32db24d69cb4818208ccc28894e2f427cfa/variants/STM32F3xx/F303V(B-C)T/variant_generic.h#L197
 *
 * In `xxx_i2cInitEx()`, if both SDA and SCL pins are -1 (default) I2C1 is chosen, else I2C2. Eg:
 *   - ssd1306_128x32_i2c_init();             // I2C1 (default)
 *   - ssd1306_i2cInitEx(-1, -1, SSD1306_SA); // I2C1
 *   - ssd1306_i2cInitEx( 0,  0, SSD1306_SA); // I2C2
 */

#include <stdint.h>
#include "stm32f3xx.h"
#include "stm32f3xx_hal_gpio.h"

typedef struct STM32F3_I2C_PinMap
{
    GPIO_TypeDef *gpio_port;
    uint32_t sda_pin;
    uint32_t scl_pin;
    I2C_TypeDef *i2c_instance;
    uint32_t gpio_alternate_function;
    void (*gpio_clock_enable)(void);
    void (*i2c_clock_enable)(void);
    void (*i2c_clock_disable)(void);
} STM32F3_I2C_PinMap;

static inline void gpioa_clock_enable(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

static inline void gpiob_clock_enable(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
}

static inline void i2c1_clock_enable(void)
{
    __HAL_RCC_I2C1_CLK_ENABLE();
}

static inline void i2c2_clock_enable(void)
{
    __HAL_RCC_I2C2_CLK_ENABLE();
}

static inline void i2c1_clock_disable(void)
{
    __HAL_RCC_I2C1_CLK_DISABLE();
}

static inline void i2c2_clock_disable(void)
{
    __HAL_RCC_I2C2_CLK_DISABLE();
}

// clang-format off
static const STM32F3_I2C_PinMap s_i2c_pinmap_options[] = {
    {.gpio_port = GPIOB,
     .sda_pin = GPIO_PIN_7,
     .scl_pin = GPIO_PIN_6,
     .i2c_instance = I2C1,
     .gpio_alternate_function = GPIO_AF4_I2C1,
     .gpio_clock_enable = gpiob_clock_enable,
     .i2c_clock_enable = i2c1_clock_enable,
     .i2c_clock_disable = i2c1_clock_disable
    },
    {.gpio_port = GPIOA,
     .sda_pin = GPIO_PIN_10,
     .scl_pin = GPIO_PIN_9,
     .i2c_instance = I2C2,
     .gpio_alternate_function = GPIO_AF4_I2C2,
     .gpio_clock_enable = gpioa_clock_enable,
     .i2c_clock_enable = i2c2_clock_enable,
     .i2c_clock_disable = i2c2_clock_disable
    }
};
// clang-format on

#endif
