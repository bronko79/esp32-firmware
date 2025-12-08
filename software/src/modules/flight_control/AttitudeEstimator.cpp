#include "AttitudeEstimator.h"

AttitudeEstimator::AttitudeEstimator() {
    m_alpha = (float)0.98;
}

AttitudeEstimator::AttitudeEstimator(float alpha) {
    m_alpha = alpha;
}

void AttitudeEstimator::update(float euler[3]) {
    m_att.roll  = euler[2];
    m_att.pitch = euler[1];
    m_att.yaw   = euler[0];
}

const Attitude &AttitudeEstimator::getAttitude() const {
    return m_att;
}

void AttitudeEstimator::reset() {
    m_att = Attitude{};
}
