#ifndef _SSD1306_STM32F3_TARGETS_H_
#define _SSD1306_STM32F3_TARGETS_H_

/* Source:
 * https://github.com/STMicroelectronics/cmsis_device_f3/blob/167eefd811de90a58c41e0a32071cdfecede389a/Include/stm32f3xx.h#L125
 */
#if defined(USE_HAL_DRIVER) &&                                                                                         \
    (defined(STM32F301x8) || defined(STM32F302x8) || defined(STM32F302xC) || defined(STM32F302xE) ||                   \
     defined(STM32F303x8) || defined(STM32F303xC) || defined(STM32F303xE) || defined(STM32F373xC) ||                   \
     defined(STM32F334x8) || defined(STM32F318xx) || defined(STM32F328xx) || defined(STM32F358xx) ||                   \
     defined(STM32F378xx) || defined(STM32F398xx))

#ifndef STM32F3
#define STM32F3
#endif

#endif
#endif
