#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <cstdint>

class pidController
{
    public:
        int pidCalculation(float setpoint, float feedback, float delta_time, bool is_angle);
        int16_t leftMotorSpeed(int16_t min_speed, uint16_t max_speed, int16_t motor_bias, int u);
        int16_t rightMotorSpeed(int16_t min_speed, uint16_t max_speed, int16_t motor_bias, int u);
        void setKp(float kp);
        void setKi(float ki);
        void setKd(float kd);
        void setParameters(float kp, float ki, float kd);


    private:
        /* VARIAVEIS */
        float _kp;
        float _ki;
        float _kd;
        float _integral;
        float _previous_error;
        int _u;
};

#endif