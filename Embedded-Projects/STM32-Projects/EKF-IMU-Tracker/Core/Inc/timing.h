
#ifndef TIMING_H
#define TIMING_H

#include <stdint.h>

// Global flag to trigger EKF cycle in main loop
extern volatile uint8_t ekf_trigger_flag;

/**
 * @brief Configures TIM2 for a 50ms periodic interrupt (20Hz)
 * and 1 microsecond counter resolution.
 */
void Timing_Init(void);

/**
 * @brief Returns the current microsecond counter value.
 * Useful for profiling matrix calculation performance.
 */
uint32_t Get_Micros(void);

#endif /* TIMING_H */
