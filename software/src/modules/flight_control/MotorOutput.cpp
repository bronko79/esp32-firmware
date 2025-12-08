#include "MotorOutput.h"

MotorOutput::MotorOutput() {
}

MotorOutput::MotorOutput(TF_HAL *hal) {
    tf_servo_v2_create(&servoBricklet, nullptr, hal);
    
    for (uint8_t i = 0; i < 4; ++i) {
        // Optionale Motion-Konfiguration, hier "schnell" und ohne Rampen
        tf_servo_v2_set_motion_configuration(&servoBricklet, i,
                                            0,      // velocity = 0 -> unbegrenzt 
                                            0,      // acceleration
                                            0);     // deceleration

        // Servo-Channel aktivieren
        tf_servo_v2_set_enable(&servoBricklet, i, true);

    }
}

void MotorOutput::writeMotors(const float motors[4], int16_t degreeMin, int16_t degreeMax) {
    // Degree-Range für die verwendeten Kanäle setzen
    // 0..10000 bedeutet: writeMotors(0..1) -> 0..10000
    for (uint8_t i = 0; i < 4; ++i) {
        tf_servo_v2_set_degree(&servoBricklet, i, degreeMin, degreeMax);


    }
}