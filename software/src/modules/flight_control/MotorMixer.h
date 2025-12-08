#include <cmath>

class MotorMixer {

    public:
        MotorMixer();
        // Input: normierter Gesamt-Schub [0..1], Momente tau* ~ [-1..1]
        void mix(float thrust, float tauRoll, float tauPitch, float tauYaw, float motors[4]) const;

};