#ifndef _ISR_UTILS_H
#define _ISR_UTILS_H

typedef void (*Isr_Callback_t)(void);

typedef enum
{
   ISR_REG_DMA = 0,
   ISR_REG_DMA_ERROR,
   ISR_REG_UART
} Isr_Registration_t;

void Isr_Init(void);
void Isr_Register(Isr_Registration_t reg, Isr_Callback_t callback);

#endif
