#include "tinkerforgeIMU.h"

static void onIMUData(TF_IMUV3 *device, int16_t acceleration[3],
                             int16_t magnetic_field[3], int16_t angular_velocity[3],
                             int16_t euler_angle[3], int16_t quaternion[4],
                             int16_t linear_acceleration[3], int16_t gravity_vector[3],
                             int8_t temperature, uint8_t calibration_status,
                             void *user_data){


    TinkerforgeIMU *self = (TinkerforgeIMU *)user_data;

    float gyroData[3];
    float accelData[3];

    // Umrechnung:
    //  - Accel: 1 cm/s^2 → m/s^2 (durch 100) :contentReference[oaicite:7]{index=7}
    //  - Gyro:  1/16 °/s → rad/s          :contentReference[oaicite:8]{index=8}

    constexpr float CM_PER_S2_TO_M_PER_S2 = 1.0f / 100.0f;
    constexpr float DEG_TO_RAD = static_cast<float>(M_PI) / 180.0f;
    constexpr float GYRO_SCALE = 1.0f / 16.0f; // 1/16 °/s

    // Accel
    for (int i = 0; i < 3; ++i) {
        accelData[i] = static_cast<float>(acceleration[i]) * CM_PER_S2_TO_M_PER_S2;
    }

    // Gyro
    for (int i = 0; i < 3; ++i) {
        float deg_per_s = static_cast<float>(angular_velocity[i]) * GYRO_SCALE;
        gyroData[i] = deg_per_s * DEG_TO_RAD;
    }

    self->m_userCallback(gyroData, accelData, self->m_context_data);
}

TinkerforgeIMU::TinkerforgeIMU(){

}

int TinkerforgeIMU::start(TF_HAL *hal, uint32_t period, IMUUserCallback callback, void *context_data){
    m_context_data = context_data;
    m_userCallback = callback;
    tf_imu_v3_create(&imu, NULL, hal);
    tf_imu_v3_register_all_data_callback(&imu, &onIMUData, this);
    return tf_imu_v3_set_all_data_callback_configuration(&imu, period, false);
}