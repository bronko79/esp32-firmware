#include "PID.h"

static float clamp(float v, float minVal, float maxVal) {
    return std::max(minVal, std::min(v, maxVal));
}

PID::PID(float kp, float ki, float kd, float integratorLimit, float outputLimit) {
    m_kp = kp; 
    m_ki = ki;
    m_kd = kd;
    m_integratorLimit = integratorLimit;
    m_outputLimit = outputLimit;
}

float PID::update(float setpoint, float measurement, float dt) {
    const float error = setpoint - measurement;

    // P-Anteil
    const float pTerm = m_kp * error;

    // I-Anteil (mit Anti-Windup)
    m_integrator += error * dt;
    m_integrator = clamp(m_integrator, -m_integratorLimit, m_integratorLimit);
    const float iTerm = m_ki * m_integrator;

    // D-Anteil
    const float derivative = (error - m_prevError) / dt;
    const float dTerm = m_kd * derivative;

    m_prevError = error;

    float output = pTerm + iTerm + dTerm;
    output = clamp(output, -m_outputLimit, m_outputLimit);
    return output;
}

void PID::reset() {
    m_integrator = 0.0f;
    m_prevError = 0.0f;
}
