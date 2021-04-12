#ifndef _GPIO_UTILS_H
#define _GPIO_UTILS_H

typedef enum
{
   GPIO_DEBUG_PIN_1 = 1,
   GPIO_DEBUG_PIN_2,
   GPIO_DEBUG_PIN_3,
   GPIO_DEBUG_PIN_4
} GpioDebugPin_t;

void Gpio_ConfigDebugLed(void);
void Gpio_ToggleDebugPin(GpioDebugPin_t pin);

#endif

