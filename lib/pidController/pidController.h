#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <cstdint>

class pidController
{
    public:
        // Existing generic PID API preserved for compatibility
        int pidCalculation(float setpoint, float feedback, float delta_time, bool is_angle);
        int16_t leftMotorSpeed(int16_t min_speed, uint16_t max_speed, int16_t motor_bias, int u);
        int16_t rightMotorSpeed(int16_t min_speed, uint16_t max_speed, int16_t motor_bias, int u);
        void setKp(float kp);
        void setKi(float ki);
        void setKd(float kd);
        void setParameters(float kp, float ki, float kd);

        // Cascaded Outer Wall P-Controller & Inner Gyro PD Controller
        static float wrap180(float angle);
        static float clampFloat(float val, float minVal, float maxVal);

        void setWallKp(float kpWall);
        void setGyroPD(float kpGyro, float kdGyro);
        void resetGating();

        float getKp() const { return _kp; }
        float getKi() const { return _ki; }
        float getKd() const { return _kd; }
        float getWallKp() const { return _kp_wall; }

        float computeWallAngleOffset(
            float dist_left_mm, bool left_updated,
            float dist_right_mm, bool right_updated,
            float kWallPresentMaxMm,
            float kCenterTargetMm,
            float kMaxStepDeltaMm,
            float maxWallAngleDeg = 12.0f
        );

        int computeGyroPD(float cell_heading_deg, float wall_angle_deg, float current_yaw_deg, float gyro_z_dps);

    private:
        /* VARIAVEIS */
        float _kp = 2.0f;
        float _ki = 0.0f;
        float _kd = 0.01f;
        float _integral = 0.0f;
        float _previous_error = 0.0f;
        int _u = 0;

        // Outer wall loop gain & edge-gating state
        float _kp_wall = 0.12f; // deg per mm (clamped to +/- 12 deg)
        float _prev_left_mm = -1.0f;
        float _prev_right_mm = -1.0f;
        uint8_t _left_ignore_cycles = 0;
        uint8_t _right_ignore_cycles = 0;

        static constexpr uint8_t kEdgeGateCycles = 3;
};

#endif