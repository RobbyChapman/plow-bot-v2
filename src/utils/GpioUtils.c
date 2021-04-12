#include <stdint.h>
#include <stdbool.h>
#include "driverlib/rom.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/rom_map.h"
#include "utils/GpioUtils.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"

void Gpio_ToggleDebugPin(GpioDebugPin_t pin)
{
    uint32_t pinMask = 0;
    uint32_t portMask = 0;
    volatile uint32_t ticks = 5000000;

    switch(pin)
    {
        case GPIO_DEBUG_PIN_1:
            pinMask = GPIO_PIN_0;
            portMask = GPIO_PORTN_BASE;
            break;
        case GPIO_DEBUG_PIN_2:
            pinMask = GPIO_PIN_1;
            portMask = GPIO_PORTN_BASE;
            break;
        case GPIO_DEBUG_PIN_3:
            pinMask = GPIO_PIN_4;
            portMask = GPIO_PORTF_BASE;
            break;
        case GPIO_DEBUG_PIN_4:
            pinMask = GPIO_PIN_0;
            portMask = GPIO_PORTF_BASE;
            break;
        default:
            break;
    }

    GPIOPinWrite(portMask, pinMask, pinMask);
    SysCtlDelay(ticks);
    GPIOPinWrite(portMask, pinMask, 0);
}

void Gpio_ConfigDebugLed(void)
{
    uint32_t pulseCnt = 0;

    /* Configure D1 and D2 on port N */
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)){};
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* Configure D3 and D4 on port F */
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)){};
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);

    for (pulseCnt = 0; pulseCnt < 2; pulseCnt++)
    {
        /* Toggle Debug Pin 1 */
        Gpio_ToggleDebugPin(GPIO_DEBUG_PIN_1);
        /* Toggle Debug Pin 2 */
        Gpio_ToggleDebugPin(GPIO_DEBUG_PIN_2);
        /* Toggle Debug Pin 3 */
        Gpio_ToggleDebugPin(GPIO_DEBUG_PIN_3);
        /* Toggle Debug Pin 4 */
        Gpio_ToggleDebugPin(GPIO_DEBUG_PIN_4);
    }
}
