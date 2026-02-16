#ifndef EKF_H_
#define EKF_H_

#include "arm_math.h"

/* State and measurement dimensions */
#define STATE_DIM    4   /* Quaternion state: [q0, q1, q2, q3] */
#define MEASURE_DIM  3   /* Accelerometer measurement: [ax, ay, az] */

/**
 * @brief EKF handle structure.
 *
 * All matrices use statically allocated memory to avoid dynamic allocation
 * and improve real-time performance on embedded targets.
 */
typedef struct
{
    /* Sampling period (seconds) */
    float32_t dt;

    /* Core EKF data buffers */
    float32_t x_data[STATE_DIM];                    /* State vector */
    float32_t P_data[STATE_DIM * STATE_DIM];        /* State covariance */
    float32_t Q_data[STATE_DIM * STATE_DIM];        /* Process noise covariance */
    float32_t R_data[MEASURE_DIM * MEASURE_DIM];    /* Measurement noise covariance */

    /* CMSIS-DSP matrix instances */
    arm_matrix_instance_f32 x;
    arm_matrix_instance_f32 P;
    arm_matrix_instance_f32 Q;
    arm_matrix_instance_f32 R;

    /* Predefined matrices for speed (no runtime allocation) */
    float32_t A_data[16];   /* State transition Jacobian (4x4) */
    float32_t H_data[12];   /* Measurement Jacobian (3x4) */
    float32_t K_data[12];   /* Kalman gain (4x3) */

    arm_matrix_instance_f32 A;
    arm_matrix_instance_f32 H;
    arm_matrix_instance_f32 K;

    /* Temporary buffers for matrix operations */
    float32_t tmp4x4_1[16];
    float32_t tmp4x4_2[16];
    float32_t tmp3x3_1[9];
    float32_t tmp3x3_2[9];
    float32_t tmp4x3_1[12];

    arm_matrix_instance_f32 m4x4_1;
    arm_matrix_instance_f32 m4x4_2;
    arm_matrix_instance_f32 m3x3_1;
    arm_matrix_instance_f32 m3x3_2;
    arm_matrix_instance_f32 m4x3_1;

} EKF_Handle;

/* EKF public API */
void EKF_Init(EKF_Handle *ekf, float32_t dt);
void EKF_Predict(EKF_Handle *ekf, float gx, float gy, float gz);
void EKF_Update(EKF_Handle *ekf, float ax, float ay, float az);

#endif /* EKF_H_ */
