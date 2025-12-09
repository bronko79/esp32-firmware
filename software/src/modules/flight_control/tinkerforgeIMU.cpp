#include "tinkerforgeIMU.h"

static void onIMUData(TF_IMUV3 *device, int16_t acceleration[3],
                             int16_t magnetic_field[3], int16_t angular_velocity[3],
                             int16_t euler_angle[3], int16_t quaternion[4],
                             int16_t linear_acceleration[3], int16_t gravity_vector[3],
                             int8_t temperature, uint8_t calibration_status,
                             void *user_data){


    TinkerforgeIMU *self = (TinkerforgeIMU *)user_data;
    float gyroData[3];
    float linearAccData[3];

    constexpr float DEG_TO_RAD = static_cast<float>(M_PI) / 180.0f;
    constexpr float GYRO_SCALE = 1.0f / 16.0f; // 1/16 °/s

    for (int i = 0; i < 3; ++i) {
        // Gyro 
        float deg_per_s = static_cast<float>(angular_velocity[i]) * GYRO_SCALE;
        gyroData[i] = deg_per_s * DEG_TO_RAD; 
        linearAccData[i] = linear_acceleration[i];
    }

    self->m_userCallback(gyroData, linearAccData,
        Quaternion(quaternion[0], quaternion[1], quaternion[2],quaternion[3])
        , self->m_context_data);
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