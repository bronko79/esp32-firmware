#include "AttitudeEstimator.h"

AttitudeEstimator::AttitudeEstimator(float alpha) {
    m_alpha = alpha;
}

void AttitudeEstimator::update(const float gyro[3], const float accel[3], float dt) {
    // 1. Gyro-Integration
    m_att.roll  += gyro[0] * dt;
    m_att.pitch += gyro[1] * dt;
    m_att.yaw   += gyro[2] * dt;

    // 2. Winkel aus Beschleunigung (vereinfachtes Modell)
    const float ax = accel[0];
    const float ay = accel[1];
    const float az = accel[2];

    const float roll_acc  = std::atan2(ay, az);
    const float pitch_acc = std::atan2(-ax, std::sqrt(ay * ay + az * az));

    // 3. Complementary Filter
    m_att.roll  = m_alpha * m_att.roll  + (1.0f - m_alpha) * roll_acc;
    m_att.pitch = m_alpha * m_att.pitch + (1.0f - m_alpha) * pitch_acc;
    // yaw hier ohne Korrektur (würde Magnetometer benötigen)
}

const Attitude &AttitudeEstimator::getAttitude() const {
    return m_att;
}

void AttitudeEstimator::reset() {
    m_att = Attitude{};
}
