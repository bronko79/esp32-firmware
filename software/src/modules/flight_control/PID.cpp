#include "PID.h"

constexpr float deg2rad(float deg) { return deg * static_cast<float>(M_PI) / 180.0f; }

static float clamp(float v, float minVal, float maxVal) {
    return std::max(minVal, std::min(v, maxVal));
}

PID::PID() {
    //PID (4.0f, 0.0f, 0.0f, 0.5f, 1); 
}

PID::PID(float kp, float ki, float kd, float integratorLimit, float outputLimit) : 
        m_kp(kp)
        , m_ki(ki)
        , m_kd(kd)
        , m_integratorLimit(integratorLimit)
        , m_outputLimit(outputLimit)
{

}

float PID::update(float setpoint, float measurement, float dt) {
    // Sicherheitscheck: dt sollte > 0 sein
    if (dt <= 0.0f) {
        // Kein Fortschritt in der Zeit -> nur P-Term ohne Integrator/D-Anteil
        float error = setpoint - measurement;
        float output = m_kp * error;
        return clamp(output, -m_outputLimit, m_outputLimit);
    }

    const float error = setpoint - measurement;

    // P-Anteil
    const float pTerm = m_kp * error;

    // I-Anteil (mit einfachem Anti-Windup)
    m_integrator += error * dt;
    m_integrator = clamp(m_integrator, -m_integratorLimit, m_integratorLimit);
    const float iTerm = m_ki * m_integrator;

    // D-Anteil (auf Fehler, alternativ kann man auch auf measurement ableiten)
    const float derivative = (error - m_prevError) / dt;
    const float dTerm = m_kd * derivative;

    m_prevError = error;

    float output = pTerm + iTerm + dTerm;

    // Output-Limit
    output = clamp(output, -m_outputLimit, m_outputLimit);
    return output;
}

/// Setzt Integrator und vorherigen Fehler zurück
void PID::reset() {
    m_integrator = 0.0f;
    m_prevError = 0.0f;
}

