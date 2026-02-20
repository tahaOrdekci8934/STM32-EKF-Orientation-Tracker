/**
 * @brief Initializes DMA1 Channel 7 for USART2 TX (Register-Level).
 */

#include <stdint.h>      /* uint32_t, uint16_t */
#include <telemetry.h>

#define UART_TIMEOUT  100000

/**
 * @brief Configure DMA1 Channel 7 for USART2 transmission.
 */
void Telemetry_DMA_Init(void)
{
    /* 1. Enable DMA1 clock (AHB bit 0: DMA1EN) */
    RCC->AHBENR |= (1 << 0);

    /* 2. Set peripheral address to USART2 Data Register */
    /* DMA will write data directly into USART2->DR */
    DMA1_Channel7->CPAR = (uint32_t)&(USART2->DR);

    /* 3. Configure DMA Channel Control Register (CCR)
     *
     * Bit 4      DIR   = 1 : Read from memory, write to peripheral
     * Bit 7      MINC  = 1 : Increment memory address after each transfer
     * Bit 12-13  PL    = 01: Medium priority
     *
     * Note:
     * Channel is expected to be disabled before this configuration.
     */
    DMA1_Channel7->CCR =
        (1 << 4)  |   /* Memory-to-peripheral direction */
        (1 << 7)  |   /* Memory increment mode */
        (1 << 12);    /* Medium priority level */

    /* 4. Enable USART2 DMA transmission (CR3 bit 7: DMAT) */
    /* Allows USART2 to request data from DMA when TX is ready */
    USART2->CR3 |= (1 << 7);
}

/**
 * @brief Starts a DMA transfer using DMA1 Channel 7.
 * @param packet_ptr Pointer to data buffer in memory
 * @param size       Number of bytes to transmit
 */
void Telemetry_Send_Burst(void *packet_ptr, uint16_t size)
{
    uint32_t timeout = UART_TIMEOUT;

    /* Wait until previous DMA transfer is complete (TCIF7 flag) */
    while (!(DMA1->ISR & (1 << 25)) && --timeout);

    /* Disable DMA channel before reconfiguration */
    DMA1_Channel7->CCR &= ~(1 << 0);

    /* Set memory address to the packet buffer */
    DMA1_Channel7->CMAR = (uint32_t)packet_ptr;

    /* Set number of bytes to transfer */
    DMA1_Channel7->CNDTR = size;

    /* Clear all interrupt flags for Channel 7 */
    DMA1->IFCR = (0xF << 24);

    /* Enable DMA channel to start transmission */
    DMA1_Channel7->CCR |= (1 << 0);
}
