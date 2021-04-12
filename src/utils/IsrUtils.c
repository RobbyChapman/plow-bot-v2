#include <string.h>
#include "utils/IsrUtils.h"

#define ISR_MAX_REG_ENTRIES      10

typedef struct
{
   Isr_Registration_t reg;
   Isr_Callback_t callback;
} IsrConfig_t;

static IsrConfig_t isr_Map[ISR_MAX_REG_ENTRIES];

void Isr_Init(void)
{
   memset(isr_Map, 0x00, sizeof(isr_Map));
}

void Isr_Register(Isr_Registration_t reg, Isr_Callback_t callback)
{
   if (callback != NULL)
   {
      isr_Map[reg].reg = reg;
      isr_Map[reg].callback = callback;
   }
}

void uDMAErrorHandler(void)
{
   if (isr_Map[ISR_REG_DMA_ERROR].callback != NULL)
   {
      isr_Map[ISR_REG_DMA_ERROR].callback();
   }
}

void UARTIntHandler(void)
{
   if (isr_Map[ISR_REG_UART].callback != NULL)
   {
      isr_Map[ISR_REG_UART].callback();
   }
}
