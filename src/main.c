
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_uart.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"

#include "utils/uartstdio.h"
#include "decoder/IbusDecoder.h"
#include "utils/GpioUtils.h"
#include "utils/IsrUtils.h"

#define UART_RXBUF_SIZE         32
#define DMA_BUF_COUNT           2
#define DMA_BUF_A_IDX           0
#define DMA_BUF_B_IDX           1

typedef enum
{
    DmaBufStatEmpty,
    DmaBufStatFill,
    DmaBufStatFull
} Dma_BufStat_t;

#if defined(ewarm)
#pragma data_alignment=1024
uint8_t pui8ControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(pui8ControlTable, 1024)
uint8_t pui8ControlTable[1024];
#else
uint8_t pui8ControlTable[1024] __attribute__ ((aligned(1024)));
#endif

#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

static uint32_t g_ui32SysClock = 0;
static uint32_t g_ui32RxBufACount = 0;
static uint32_t g_ui32RxBufBCount = 0;
static uint32_t g_ui32uDMAErrCount = 0;
static uint8_t g_ui8RxBufA[UART_RXBUF_SIZE]= {0};
static uint8_t g_ui8RxBufB[UART_RXBUF_SIZE] = {0};
static Dma_BufStat_t dmaBufStat[DMA_BUF_COUNT] = {DmaBufStatEmpty};

static void configDmaWithUart(void);

static void configDmaWithUart(void)
{
    /* Enable DMA peripheral */
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UDMA);

    /* Enable UART 6 peripheral */
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART1);

    /* Configure As UART 6 on PP0/1 */
    GPIOPinConfigure(GPIO_PB0_U1RX);
    ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0);

    /* UART 6 @ 115200 8-N-1 */
    ROM_UARTConfigSetExpClk(UART1_BASE, g_ui32SysClock, 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    /* Not sure about TX, but I know RX will have 4 byte threshold */
    ROM_UARTFIFOLevelSet(UART1_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);

    /* Enable the UART for operation, and enable the uDMA interface for RX only */
    ROM_UARTEnable(UART1_BASE);
    ROM_UARTDMAEnable(UART1_BASE, UART_DMA_RX);

    /* Enable DMA interrupts */
    ROM_IntEnable(INT_UDMAERR);

    /* Enable DMA */
    ROM_uDMAEnable();

    /* Point at the control table to use for channel control structures */
    ROM_uDMAControlBaseSet(pui8ControlTable);

    /* Put the attributes in a known state */
    ROM_uDMAChannelAttributeDisable(UDMA_CHANNEL_UART1RX, UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);

    /* Enable DMA attributes for UART6 RX (buffer A). In this case, the transfer data size
    is 8 bits, the source address does not increment since it will be reading from a register. The
    destination address increment is byte 8-bit bytes. The arbitration size is set to 4 to match the
    RX FIFO trigger threshold */
    ROM_uDMAChannelControlSet(UDMA_CHANNEL_UART1RX | UDMA_PRI_SELECT, UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_4);

    /* Enable DMA attributes for UART6 RX (buffer B). Setting this an the alternate control
    structure B. Configuration is the same as above */
    ROM_uDMAChannelControlSet(UDMA_CHANNEL_UART1RX | UDMA_ALT_SELECT, UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_4);

    /* Set transfer parameters for the primary RX control structure in ping-pong mode.
    The transfer source will be the UART RX data register, and the destination will be buffer A */
    ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART1RX | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG, (void *)(UART1_BASE + UART_O_DR), g_ui8RxBufA, sizeof(g_ui8RxBufA));

    /* Same logic applies above, except we are now setting up the alternative buffer (B) */
    ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART1RX | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG, (void *)(UART1_BASE + UART_O_DR), g_ui8RxBufB, sizeof(g_ui8RxBufB));

    /* The DMA should now be configured for UART RX in ping-ping mode, with both primary
    and secondary buffers pointing to the UART data register */
    ROM_uDMAChannelEnable(UDMA_CHANNEL_UART1RX);

    /* Enable DMA for UART RX. Stop DMA if there's a UART error */
    ROM_UARTDMAEnable(UART1_BASE, UART_DMA_RX | UART_DMA_ERR_RXSTOP);

    /* Enable UART */
    ROM_IntEnable(INT_UART1);
    ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
    ROM_UARTIntEnable(UART1_BASE, UART_INT_DMARX);
}

static void resyncStream(void)
{
    uint32_t idx = 0;
    uint8_t buf[64] = {0};
    bool foundEnd = false;
    bool foundStart = false;

    while(ROM_UARTCharsAvail(UART1_BASE))
    {
        /* Read the next character from the UART and write it back to the UART */
        buf[idx] = ROM_UARTCharGet(UART1_BASE);

        if (idx > 0)
        {
            if ((buf[idx-1] == 0x20) && (buf[idx] == 0x40))
            {
                foundStart = true;
                buf[0] = buf[idx-1];
                buf[1] = buf[idx];
                /* Clear remaining bytes. Set i to 3 */
                memset(buf+2, 0x00, sizeof(buf-2));
                idx = 3;
            }
            else
            {
                if ((foundStart == true) && (idx == 31))
                {
                    foundEnd = true;
                }
            }
        }

        if ((foundStart == true) && (foundEnd == true))
        {
            /* Parse packet, kick start DMA for next collection sequence */
            Gpio_ToggleDebugPin(GPIO_DEBUG_PIN_4);
        }

        idx++;
    }
}

static void DmaErrorCallback(void)
{
    uint32_t ui32Status;

    /* Check for uDMA error bit */
    ui32Status = ROM_uDMAErrorStatusGet();

    /* If there is a uDMA error, then clear the error and increment the error counter */
    if(ui32Status)
    {
        ROM_uDMAErrorStatusClear();
        g_ui32uDMAErrCount++;
    }
}

static void DmaCallback(void)
{

}

static void UartCallback(void)
{
        volatile uint32_t ui32Mode = 0;
        volatile uint32_t ui32Status = 0;

        ui32Status = ROM_UARTIntStatus(UART1_BASE, true);

        /* Clear interrupts */
        ROM_UARTIntClear(UART1_BASE, ui32Status);

        /* Check the DMA control table to see if the ping-pong "A" transfer is complete.
        The "A" transfer uses receive buffer "A", and the primary control structure */
        ui32Mode = ROM_uDMAChannelModeGet(UDMA_CHANNEL_UART1RX | UDMA_PRI_SELECT);

        /* If the primary control structure indicates stop, that means the "A" receive buffer
        is done.  The uDMA controller should still be receiving data into the "B" buffer */
        if(ui32Mode == UDMA_MODE_STOP)
        {
            /* Set fill state based on DMA buffer 'A' */
            if (dmaBufStat[DMA_BUF_A_IDX] == DmaBufStatFill)
            {
                dmaBufStat[DMA_BUF_A_IDX] = DmaBufStatFull;
                dmaBufStat[DMA_BUF_B_IDX] = DmaBufStatFill;
            }

            /* Increment a counter to indicate data was received into buffer A */
            g_ui32RxBufACount++;

            /* Set up the next transfer for the "A" buffer, using the primary control structure.
            When the ongoing receive into the "B" buffer is done, the uDMA controller will switch */
            ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART1RX | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG, (void *)(UART1_BASE + UART_O_DR), g_ui8RxBufA, sizeof(g_ui8RxBufA));
        }

        /* Check the DMA control table to see if the ping-pong "B" transfer is complete. The "B"
        transfer uses receive buffer "B", and the alternate control structure */
        ui32Mode = ROM_uDMAChannelModeGet(UDMA_CHANNEL_UART1RX | UDMA_ALT_SELECT);

        /* If the alternate control structure indicates stop, that means the "B" receive buffer is
        done. The uDMA controller should still be receiving data into the "A" buffer */
        if(ui32Mode == UDMA_MODE_STOP)
        {
            /* Set fill state based on DMA buffer 'B' */
            if (dmaBufStat[DMA_BUF_B_IDX] == DmaBufStatFill)
            {
                dmaBufStat[DMA_BUF_A_IDX] = DmaBufStatFill;
                dmaBufStat[DMA_BUF_B_IDX] = DmaBufStatFull;
            }

            /* Increment a counter to indicate data was received into buffer A */
            g_ui32RxBufBCount++;

            /* Set up the next transfer for the "B" buffer, using the alternate control structure.
            When the ongoing receive into the "A" buffer is done, the uDMA controller will switch */
            ROM_uDMAChannelTransferSet(UDMA_CHANNEL_UART1RX | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG, (void *)(UART1_BASE + UART_O_DR), g_ui8RxBufB, sizeof(g_ui8RxBufB));
        }
}

int main(void)
{
    /* Defaults */
    g_ui32SysClock = 0;
    g_ui32RxBufACount = 0;
    g_ui32RxBufBCount = 0;
    g_ui32uDMAErrCount = 0;

    memset(dmaBufStat,  0x00, sizeof(dmaBufStat));
    memset(g_ui8RxBufA, 0x00, sizeof(g_ui8RxBufA));
    memset(g_ui8RxBufB, 0x00, sizeof(g_ui8RxBufB));

    /* Set initial DMA fill state where A is filling */
    dmaBufStat[DMA_BUF_A_IDX] = DmaBufStatFill;
    dmaBufStat[DMA_BUF_B_IDX] = DmaBufStatEmpty;

    /* 120Mhz */
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

    /* Debug LED's D1-D4 */
    Gpio_ConfigDebugLed();

    /* Enable UART 0 peripheral */
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);

    /* Enable processor interrupts */
    ROM_IntMasterEnable();

    /* Configure As UART 1 on PA0/1 */
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000);

    /* Enable ISR */
    ROM_IntEnable(INT_UART0);

    configDmaWithUart();

    Ibd_Init();
    Isr_Init();
    Isr_Register(ISR_REG_DMA, DmaCallback);
    Isr_Register(ISR_REG_UART, UartCallback);
    Isr_Register(ISR_REG_DMA_ERROR, DmaErrorCallback);

    /* Spin */
    while(1)
    {
        Ibd_Update();
        if (dmaBufStat[DMA_BUF_A_IDX] == DmaBufStatFull)
        {
            if ((g_ui8RxBufA[0] != 0x20) || (g_ui8RxBufA[1] != 0x40))
            {
                resyncStream();
            }
            /* Toggle LED for testing */
            Gpio_ToggleDebugPin(GPIO_DEBUG_PIN_1);
            dmaBufStat[DMA_BUF_A_IDX] = DmaBufStatEmpty;
        }

        else if (dmaBufStat[DMA_BUF_B_IDX] == DmaBufStatFull)
        {
            if ((g_ui8RxBufB[0] != 0x20) || (g_ui8RxBufB[1] != 0x40))
            {
                resyncStream();
            }
            /* Toggle LED for testing */
            Gpio_ToggleDebugPin(GPIO_DEBUG_PIN_2);
            dmaBufStat[DMA_BUF_B_IDX] = DmaBufStatEmpty;
        }
    }
}
