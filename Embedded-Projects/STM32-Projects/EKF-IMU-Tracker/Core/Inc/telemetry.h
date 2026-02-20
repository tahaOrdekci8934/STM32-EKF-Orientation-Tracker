
#ifndef TELEMETRY_LL_H_
#define TELEMETRY_LL_H_

#include <stdint.h>

/**
 * @brief Telemetry Data Structure
 * __attribute__((packed)) prevents the compiler from adding padding bytes,
 * ensuring the memory layout matches exactly what MATLAB expects.
 */
typedef struct __attribute__((packed)) {
    float q[4];          // Estimated Quaternion (q0, q1, q2, q3)
    float accel[3];      // Normalized Accelerometer Data (ax, ay, az)
    uint32_t sync_word;  // End of packet marker (e.g., 0x7F7F7F7F)
} TelemetryPacket_t;

/**
 * @brief Initializes DMA1 Channel 7 for USART2 TX using direct register access.
 * Must be called once during system initialization.
 */
void Telemetry_DMA_Init(void);

/**
 * @brief Triggers a non-blocking DMA transfer for the telemetry packet.
 * This is the high-efficiency alternative to HAL_UART_Transmit_DMA.
 * * @param packet Pointer to the TelemetryPacket_t structure
 * @param size Size of the packet in bytes
 */
void Telemetry_Send_Burst(void *packet_ptr, uint16_t size);

#endif /* TELEMETRY_LL_H_ */
