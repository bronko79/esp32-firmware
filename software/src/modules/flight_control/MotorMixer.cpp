#include "MotorMixer.h"

static float clamp(float v, float minVal, float maxVal) {
    return std::max(minVal, std::min(v, maxVal));
}

MotorMixer::MotorMixer() {
    
}

void MotorMixer::mix(float thrust, float tauRoll, float tauPitch, float tauYaw, float motors[4]) const
{
    // X-Konfiguration, Vorzeichen ggf. an dein Setup anpassen
    // M0: front-left
    // M1: front-right
    // M2: rear-right
    // M3: rear-left
/*
    motors[0] = thrust + tauRoll + tauPitch - tauYaw; // M0
    motors[1] = thrust - tauRoll + tauPitch + tauYaw; // M1
    motors[2] = thrust - tauRoll - tauPitch - tauYaw; // M2
    motors[3] = thrust + tauRoll - tauPitch + tauYaw; // M3
*/

    motors[0] = thrust + tauRoll - tauPitch - tauYaw; // M0
    motors[1] = thrust - tauRoll - tauPitch + tauYaw; // M1
    motors[2] = thrust - tauRoll + tauPitch - tauYaw; // M2
    motors[3] = thrust + tauRoll + tauPitch + tauYaw; // M3


    for (int i = 0; i < 4; ++i) {
        motors[i] = clamp(motors[i], 0.0f, 1.0f);
    }
}