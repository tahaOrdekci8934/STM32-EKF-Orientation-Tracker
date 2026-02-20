
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN EV */
extern volatile uint8_t ekf_trigger_flag;
/* USER CODE END EV */

/* USER CODE BEGIN 1 */
/**
 * @brief TIM2 Update Interrupt Service Routine
 *
 * This interrupt is triggered on every TIM2 update event
 * (counter overflow). It is used as a periodic trigger
 * for the EKF update task in the main loop.
 */
void TIM2_IRQHandler(void)
{
    /* Check if the interrupt was caused by an update event */
    if (TIM2->SR & TIM_SR_UIF)
    {
        /* Clear the update interrupt flag to avoid re-entering the ISR */
        TIM2->SR &= ~TIM_SR_UIF;

        /* Set trigger flag for EKF processing (handled outside the ISR) */
        ekf_trigger_flag = 1;
    }
}
/* USER CODE END 1 */


