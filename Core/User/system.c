#include "stm32g4xx_hal.h"

// Disable all interrupts
void system_irq_disable(void)
{
    __disable_irq();
    __DSB();
    __ISB();
}


// Enable all interrupts
void system_irq_enable(void)
{
    __enable_irq();
}
