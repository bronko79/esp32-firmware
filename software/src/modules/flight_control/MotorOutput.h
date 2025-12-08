#include "bindings/hal_common.h"
#include "bindings/bricklet_servo_v2.h"

class MotorOutput {

    public:
        MotorOutput();
        MotorOutput(TF_HAL *hal);

        // motors[i] im Bereich [0, 1] (wird in PWM o. Ä. umgesetzt)
        void writeMotors(const float motors[4], int16_t degreeMin, int16_t degreeMax);


        TF_ServoV2 servoBricklet;
};
