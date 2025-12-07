#include <cmath>

class PID {

public:
    PID(float kp, float ki, float kd, float integratorLimit, float outputLimit);
    float update(float setpoint, float measurement, float dt);
    void reset();

private:
    float m_kp;
    float m_ki;
    float m_kd;

    float m_integrator = 0.0f;
    float m_prevError = 0.0f;

    float m_integratorLimit;
    float m_outputLimit;

};