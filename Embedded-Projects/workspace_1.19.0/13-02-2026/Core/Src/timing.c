#include <timing.h>

/* Global flag accessible from other modules */
volatile uint8_t ekf_trigger_flag = 0;

/**
 * @brief Initialize TIM2 at register level.
 */
void Timing_Init(void)
{
    /* 1. Enable TIM2 clock on APB1 bus (APB1ENR bit 0: TIM2EN) */
    RCC->APB1ENR |= (1 << 0);

    /* 2. Configure prescaler for 1 µs timer resolution
     *    72 MHz / (71 + 1) = 1 MHz
     */
    TIM2->PSC = 71;

    /* 3. Set auto-reload value for 50 ms period
     *    50,000 µs → ARR = 49999 (counter starts from 0)
     */
    TIM2->ARR = 49999;

    /* 4. Enable update interrupt (DIER bit 0: UIE) */
    TIM2->DIER |= (1 << 0);

    /* 5. Configure NVIC for TIM2 interrupt
     *    Priority level: 2
     *    IRQ number: TIM2_IRQn (28)
     */
    NVIC_SetPriority(TIM2_IRQn, 2);
    NVIC_EnableIRQ(TIM2_IRQn);

    /* 6. Force an update event to load PSC and ARR immediately (EGR bit 0: UG)
     *    This transfers prescaler and auto-reload values into active registers.
     */
    TIM2->EGR |= (1 << 0);

    /* 7. Clear update interrupt flag (SR bit 0: UIF)
     *    UG sets UIF automatically; it must be cleared to avoid
     *    an immediate interrupt when the timer starts.
     */
    TIM2->SR &= ~(1 << 0);

    /* 8. Enable timer counter (CR1 bit 0: CEN) */
    TIM2->CR1 |= (1 << 0);
}

/**
 * @brief Returns the current microsecond counter value.
 * @retval Current TIM2 counter value (0–49999 µs)
 */
uint32_t Get_Micros(void)
{
    /* Return current timer count value */
    return TIM2->CNT;
}
