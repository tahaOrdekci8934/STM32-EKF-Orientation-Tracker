#include <EKF_algorithm.h>
#include <math.h>

/**
 * @brief Initialize EKF state, covariance matrices, and constants.
 * @param ekf Pointer to EKF handle structure
 * @param dt  Sampling period in seconds
 */
void EKF_Init(EKF_Handle *ekf, float32_t dt)
{
    ekf->dt = dt;

    /* Initialize state and covariance matrices */
    arm_mat_init_f32(&ekf->x, 4, 1, ekf->x_data);
    arm_mat_init_f32(&ekf->P, 4, 4, ekf->P_data);
    arm_mat_init_f32(&ekf->Q, 4, 4, ekf->Q_data);
    arm_mat_init_f32(&ekf->R, 3, 3, ekf->R_data);
    arm_mat_init_f32(&ekf->A, 4, 4, ekf->A_data);
    arm_mat_init_f32(&ekf->H, 3, 4, ekf->H_data);
    arm_mat_init_f32(&ekf->K, 4, 3, ekf->K_data);

    /* Initialize temporary matrices used during EKF steps */
    arm_mat_init_f32(&ekf->m4x4_1, 4, 4, ekf->tmp4x4_1);
    arm_mat_init_f32(&ekf->m4x4_2, 4, 4, ekf->tmp4x4_2);
    arm_mat_init_f32(&ekf->m3x3_1, 3, 3, ekf->tmp3x3_1);
    arm_mat_init_f32(&ekf->m3x3_2, 3, 3, ekf->tmp3x3_2);
    arm_mat_init_f32(&ekf->m4x3_1, 4, 3, ekf->tmp4x3_1);

    /* Initial state: identity quaternion */
    ekf->x_data[0] = 1.0f;   /* q0 */

    /* Initial covariance matrix P (diagonal) */
    for (int i = 0; i < 16; i++)
        ekf->P_data[i] = (i % 5 == 0) ? 0.01f : 0.0f;

    /* Process noise covariance Q (diagonal) */
    for (int i = 0; i < 16; i++)
        ekf->Q_data[i] = (i % 5 == 0) ? 0.0001f : 0.0f;

    /* Measurement noise covariance R (diagonal) */
    for (int i = 0; i < 9; i++)
        ekf->R_data[i] = (i % 4 == 0) ? 0.05f : 0.0f;
}

/**
 * @brief EKF prediction step using gyroscope measurements.
 * @param ekf EKF handle
 * @param gx  Gyroscope X (rad/s)
 * @param gy  Gyroscope Y (rad/s)
 * @param gz  Gyroscope Z (rad/s)
 */
void EKF_Predict(EKF_Handle *ekf, float gx, float gy, float gz)
{
    /* Cache current quaternion values */
    float32_t q0 = ekf->x_data[0];
    float32_t q1 = ekf->x_data[1];
    float32_t q2 = ekf->x_data[2];
    float32_t q3 = ekf->x_data[3];

    /* 1. State prediction (quaternion kinematics) */
    ekf->x_data[0] += 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * ekf->dt;
    ekf->x_data[1] += 0.5f * ( q0 * gx + q2 * gz - q3 * gy) * ekf->dt;
    ekf->x_data[2] += 0.5f * ( q0 * gy - q1 * gz + q3 * gx) * ekf->dt;
    ekf->x_data[3] += 0.5f * ( q0 * gz + q1 * gy - q2 * gx) * ekf->dt;

    /* Normalize quaternion to maintain unit length */
    float32_t n = sqrtf(
        ekf->x_data[0] * ekf->x_data[0] +
        ekf->x_data[1] * ekf->x_data[1] +
        ekf->x_data[2] * ekf->x_data[2] +
        ekf->x_data[3] * ekf->x_data[3]
    );

    for (int i = 0; i < 4; i++)
        ekf->x_data[i] /= n;

    /* 2. State transition Jacobian A = I + 0.5 * Omega * dt */
    float32_t hdt = 0.5f * ekf->dt;

    ekf->A_data[0]  = 1.0f;     ekf->A_data[1]  = -hdt * gx; ekf->A_data[2]  = -hdt * gy; ekf->A_data[3]  = -hdt * gz;
    ekf->A_data[4]  =  hdt * gx; ekf->A_data[5]  = 1.0f;     ekf->A_data[6]  =  hdt * gz; ekf->A_data[7]  = -hdt * gy;
    ekf->A_data[8]  =  hdt * gy; ekf->A_data[9]  = -hdt * gz; ekf->A_data[10] = 1.0f;     ekf->A_data[11] =  hdt * gx;
    ekf->A_data[12] =  hdt * gz; ekf->A_data[13] =  hdt * gy; ekf->A_data[14] = -hdt * gx; ekf->A_data[15] = 1.0f;

    /* 3. Covariance prediction: P = A * P * A' + Q */
    arm_mat_mult_f32(&ekf->A, &ekf->P, &ekf->m4x4_1);

    arm_matrix_instance_f32 AT;
    float32_t AT_data[16];
    arm_mat_init_f32(&AT, 4, 4, AT_data);

    arm_mat_trans_f32(&ekf->A, &AT);
    arm_mat_mult_f32(&ekf->m4x4_1, &AT, &ekf->m4x4_2);
    arm_mat_add_f32(&ekf->m4x4_2, &ekf->Q, &ekf->P);
}

/**
 * @brief EKF update step using accelerometer measurements.
 * @param ekf EKF handle
 * @param ax  Accelerometer X (normalized)
 * @param ay  Accelerometer Y (normalized)
 * @param az  Accelerometer Z (normalized)
 */
void EKF_Update(EKF_Handle *ekf, float ax, float ay, float az)
{
    float32_t q0 = ekf->x_data[0];
    float32_t q1 = ekf->x_data[1];
    float32_t q2 = ekf->x_data[2];
    float32_t q3 = ekf->x_data[3];

    /* 1. Measurement Jacobian H (gravity vector model) */
    ekf->H_data[0]  = -2.0f * q2; ekf->H_data[1]  =  2.0f * q3; ekf->H_data[2]  = -2.0f * q0; ekf->H_data[3]  = 2.0f * q1;
    ekf->H_data[4]  =  2.0f * q1; ekf->H_data[5]  =  2.0f * q0; ekf->H_data[6]  =  2.0f * q3; ekf->H_data[7]  = 2.0f * q2;
    ekf->H_data[8]  =  2.0f * q0; ekf->H_data[9]  = -2.0f * q1; ekf->H_data[10] = -2.0f * q2; ekf->H_data[11] = 2.0f * q3;

    /* 2. Kalman gain computation: K = P * H' * inv(H * P * H' + R) */
    arm_matrix_instance_f32 HT;
    float32_t HT_data[12];
    arm_mat_init_f32(&HT, 4, 3, HT_data);

    arm_mat_trans_f32(&ekf->H, &HT);
    arm_mat_mult_f32(&ekf->P, &HT, &ekf->m4x3_1);
    arm_mat_mult_f32(&ekf->H, &ekf->m4x3_1, &ekf->m3x3_1);
    arm_mat_add_f32(&ekf->m3x3_1, &ekf->R, &ekf->m3x3_2);
    arm_mat_inverse_f32(&ekf->m3x3_2, &ekf->m3x3_1);
    arm_mat_mult_f32(&ekf->m4x3_1, &ekf->m3x3_1, &ekf->K);

    /* 3. Innovation (measurement residual) */
    float32_t vx = 2.0f * (q1 * q3 - q0 * q2);
    float32_t vy = 2.0f * (q0 * q1 + q2 * q3);
    float32_t vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    float32_t dz[3] = { ax - vx, ay - vy, az - vz };

    /* 4. State update: x = x + K * dz */
    ekf->x_data[0] += ekf->K_data[0]  * dz[0] + ekf->K_data[1]  * dz[1] + ekf->K_data[2]  * dz[2];
    ekf->x_data[1] += ekf->K_data[3]  * dz[0] + ekf->K_data[4]  * dz[1] + ekf->K_data[5]  * dz[2];
    ekf->x_data[2] += ekf->K_data[6]  * dz[0] + ekf->K_data[7]  * dz[1] + ekf->K_data[8]  * dz[2];
    ekf->x_data[3] += ekf->K_data[9]  * dz[0] + ekf->K_data[10] * dz[1] + ekf->K_data[11] * dz[2];

    /* 5. Covariance update: P = (I - K * H) * P
     * Simplified form: P = P - K * H * P
     */
    arm_mat_mult_f32(&ekf->K, &ekf->H, &ekf->m4x4_1);
    arm_mat_mult_f32(&ekf->m4x4_1, &ekf->P, &ekf->m4x4_2);
    arm_mat_sub_f32(&ekf->P, &ekf->m4x4_2, &ekf->P);

    /* Final quaternion normalization */
    float32_t final_n = sqrtf(
        ekf->x_data[0] * ekf->x_data[0] +
        ekf->x_data[1] * ekf->x_data[1] +
        ekf->x_data[2] * ekf->x_data[2] +
        ekf->x_data[3] * ekf->x_data[3]
    );

    for (int i = 0; i < 4; i++)
        ekf->x_data[i] /= final_n;

    if (final_n > 0.000001f) {
        for (int i = 0; i < 4; i++)
            ekf->x_data[i] /= final_n;
    } else {
        /* Failsafe reset in extremely rare numerical error cases */
        ekf->x_data[0] = 1.0f;
        ekf->x_data[1] = 0.0f;
        ekf->x_data[2] = 0.0f;
        ekf->x_data[3] = 0.0f;
    }
}
