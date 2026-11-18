#include "pidController.h"
#include <cmath>

float pidController::wrap180(float angle)
{
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle <= -180.0f) angle += 360.0f;
    return angle;
}

float pidController::clampFloat(float val, float minVal, float maxVal)
{
    if (val < minVal) return minVal;
    if (val > maxVal) return maxVal;
    return val;
}

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
    _ki = 0.0f; // Keep Ki = 0 to prevent windup across open intersections
    _kd = kd;
    (void)ki;
}

void pidController::setWallKp(float kpWall) {
    _kp_wall = kpWall;
}

void pidController::setGyroPD(float kpGyro, float kdGyro) {
    _kp = kpGyro;
    _ki = 0.0f;
    _kd = kdGyro;
}

void pidController::resetGating() {
    _prev_left_mm = -1.0f;
    _prev_right_mm = -1.0f;
    _left_ignore_cycles = 0;
    _right_ignore_cycles = 0;
}

float pidController::computeWallAngleOffset(
    float dist_left_mm, bool left_updated,
    float dist_right_mm, bool right_updated,
    float kWallPresentMaxMm,
    float kCenterTargetMm,
    float kMaxStepDeltaMm,
    float maxWallAngleDeg)
{
    // Edge gating evaluated strictly on consecutive ToF updates
    if (left_updated) {
        if (_prev_left_mm >= 0.0f && std::fabs(dist_left_mm - _prev_left_mm) > kMaxStepDeltaMm) {
            _left_ignore_cycles = kEdgeGateCycles;
        } else if (_left_ignore_cycles > 0) {
            --_left_ignore_cycles;
        }
        _prev_left_mm = dist_left_mm;
    }

    if (right_updated) {
        if (_prev_right_mm >= 0.0f && std::fabs(dist_right_mm - _prev_right_mm) > kMaxStepDeltaMm) {
            _right_ignore_cycles = kEdgeGateCycles;
        } else if (_right_ignore_cycles > 0) {
            --_right_ignore_cycles;
        }
        _prev_right_mm = dist_right_mm;
    }

    bool has_left_wall  = (_left_ignore_cycles == 0)  && (dist_left_mm < kWallPresentMaxMm);
    bool has_right_wall = (_right_ignore_cycles == 0) && (dist_right_mm < kWallPresentMaxMm);

    float error_mm = 0.0f;
    if (has_left_wall && has_right_wall) {
        // When closer to left wall (dist_left < dist_right), error > 0 commands turn right (+angle)
        error_mm = dist_right_mm - dist_left_mm;
    } else if (has_left_wall) {
        // When closer to left wall (dist_left < kCenterTargetMm), error > 0 commands turn right (+angle)
        error_mm = (kCenterTargetMm - dist_left_mm) * 2.0f;
    } else if (has_right_wall) {
        // When closer to right wall (dist_right < kCenterTargetMm), error < 0 commands turn left (-angle)
        error_mm = (dist_right_mm - kCenterTargetMm) * 2.0f;
    } else {
        error_mm = 0.0f;
    }

    float wall_angle_deg = _kp_wall * error_mm;
    return clampFloat(wall_angle_deg, -maxWallAngleDeg, maxWallAngleDeg);
}

int pidController::computeGyroPD(float cell_heading_deg, float wall_angle_deg, float current_yaw_deg, float gyro_z_dps)
{
    // theta_target = wrap180(theta_cell + theta_wall)
    float theta_target = wrap180(cell_heading_deg + wall_angle_deg);
    float angular_error = wrap180(theta_target - current_yaw_deg);

    // Motor speed mixing: left = bias - u, right = bias + u.
    // Pointing right of target (angular_error < 0) -> need to steer left (right wheel faster, u > 0)
    // Pointing left of target (angular_error > 0) -> need to steer right (left wheel faster, u < 0)
    // Rate damping: spinning right (gyro_z_dps > 0) -> need torque to the left (u > 0)
    float pwm_rot = (-_kp * angular_error) + (_kd * gyro_z_dps);
    _previous_error = angular_error;
    _u = static_cast<int>(pwm_rot);
    return _u;
}

int pidController::pidCalculation(float setpoint, float feedback, float delta_time, bool is_angle)
{
    float error = is_angle ? wrap180(setpoint - feedback) : (setpoint - feedback);

    float derivative = (delta_time > 0.0f) ? ((error - _previous_error) / delta_time) : 0.0f;
    float output = (_kp * error) + (_kd * derivative);

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