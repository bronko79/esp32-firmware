class MotorOutput {

    public:
        // motors[i] im Bereich [0, 1] (wird in PWM o. Ä. umgesetzt)
        void writeMotors(const float motors[4]);

};
