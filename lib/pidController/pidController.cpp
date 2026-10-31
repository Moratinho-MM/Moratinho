#include <pidController.h>

void pidController::setKp(float kp) {
    _kp = kp;
}

void pidController::setKi(float ki) {
    _ki = ki;
}

void pidController::setKd(float kd) {
    _kd = kd;
}

void pidController::setParameters(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

int pidController::pidCalculation(float setpoint, float feedback, float delta_time, bool is_angle)
{
    float error = setpoint - feedback;

    if (is_angle){
        if (error > 180)
        {
            error -= 360;
        }
        else if (error < -180)
        {
            error += 360;
        }
    }
    

    _integral += error * delta_time;
    float derivative = (error - _previous_error) / delta_time;
    
    float output = (_kp * error) + (_ki * _integral) + (_kd * derivative);

    _previous_error = error;
    _u = static_cast<int>(output);
    return _u;
}

int16_t pidController::rightMotorSpeed(int16_t min_speed, uint16_t max_speed, int16_t motor_bias, int u)
{
    int16_t speed = motor_bias + u;
    if (speed > max_speed) speed = max_speed;
    if (speed < min_speed) speed = min_speed;
    return speed;
}

int16_t pidController::leftMotorSpeed(int16_t min_speed, uint16_t max_speed, int16_t motor_bias, int u)
{
    int16_t speed = motor_bias - u;
    if (speed > max_speed) speed = max_speed;
    if (speed < min_speed) speed = min_speed;
    return speed;
}