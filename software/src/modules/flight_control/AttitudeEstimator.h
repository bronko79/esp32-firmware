#include <cmath>

struct Attitude {
    float roll  = 0.0f; // rad
    float pitch = 0.0f; // rad
    float yaw   = 0.0f; // rad (hier einfach Gyro-Integration)
};

class AttitudeEstimator {

private:
    Attitude m_att;
    float m_alpha; // z.B. 0.98

public:
    AttitudeEstimator(float alpha);
    void update(const float gyro[3], const float accel[3], float dt);
    const Attitude& getAttitude() const;
    void reset();   
};