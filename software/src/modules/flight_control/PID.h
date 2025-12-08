#include <cmath>

class PID {

public:
    PID();
    PID(float kp, float ki, float kd, float outMin, float outMax);
    float update(float ist, float soll, float timeSec);
    void reset();

private:
    float m_kp;
    float m_ki;
    float m_kd;

    float m_integrator = 0.0f;
    float m_prevError  = 0.0f;

    float m_integratorLimit;
    float m_outputLimit;

};