#pragma once

#include "../EngineDriver/EngineDriver.h"
#include "../pidController/pidController.h"
#include "../MPU6050_Goytacaz/MPU6050_Goytacaz.h"
#include "../VL530X_Goytacaz/VL530X_Goytacaz.h"
#include "../Orientation/Orientation.h"
#include "../Coordinates/Coordinates.h"

#include "../ConstantConfigs.h"
#include "../Simulator/Simulator.h"

class MovController {

    private:

    Orientation globalDirection;
    short posX = 0, posY = 0;

    static constexpr float TICKS_CM = (12.0f * 10.0f) / (PI * WHEEL_DIAM);
    
    EngineDriver* leftEngine;
    EngineDriver* rightEngine;

    IRSensors* ir; 
    IMU* imu;

    //********** Cascaded Outer Wall P + Inner Gyro PD **********
    pidController pid;
    float setPoint = 0.0f;     // Cell heading theta_cell in [-180, 180] deg
    float target_yaw = 0.0f;   // Combined target heading wrap180(theta_cell + theta_wall)
    int bias = 40;

    // Lateral wall thresholds (mm) for 180 mm cell (~168 mm inner wall-to-wall gap)
    static constexpr float kWallPresentMaxMm   = 110.0f;
    static constexpr float kCenterTargetMm     = 84.0f;
    static constexpr float kMaxStepDeltaMm     = 15.0f;
    static constexpr float kMaxWallAngleDeg    = 12.0f;

    // Front VL53L0X longitudinal thresholds (mm)
    static constexpr float kFrontTurnTriggerMm = 85.0f;
    static constexpr float kFrontStopMm        = 25.0f;
    static constexpr float kIdealFrontStopMm   = 42.0f;

    const float WALL_TRESHOLD = 15.0f;     // cm compatibility threshold
    const float WALL_KEEP_DISTANCE = 9.0f; // cm compatibility distance
    const float BASE_VELOCITY = 100;

    /* offsets */
    int motor_d_offset = 0;
    int motor_e_offset = 1;

    float angle_offset = 6.0f; // Tightened stopping window for wrapped gyro turns
    float turnBias     = 50;

    /* Engine speed variables */
    int motor_left_speed = 0;
    int motor_right_speed = 0;

    unsigned long timerLasUpdate = 0;

    void do_pid();
    void do_pid(int linear_bias);
    int computeLinearBias(float averageDistCm, float dist_front_mm) const;
    void executeTurnTo(float target_yaw_deg);

    public:
    MovController() = default;

    bool begin(IRSensors* ir, IMU* imu, EngineDriver* leftEngine, EngineDriver* rightEngine);

    void setPid(float kp, float ki, float kd);
    void setWallKp(float kpWall);

    unsigned long getDeltaTime();

    bool isFrontTurnTriggered();
    bool isFrontEmergencyStop();
    void alignFrontWall();

    void moveFront();
    void moveFront(int linear_bias);

    void moveCell();
    void moveCells(int count);

    void stop();
    void brake();

    void turnLeft();
    void turnRight();
    void turn180degrees();
    void turnDegrees(float angle);

    void setBias(int b) { bias = b; }
    int getBias() const { return bias; }
    void setTurnBias(float tb) { turnBias = tb; }
    float getTurnBias() const { return turnBias; }
    float getKp() const { return pid.getKp(); }
    float getKi() const { return pid.getKi(); }
    float getKd() const { return pid.getKd(); }
    float getWallKp() const { return pid.getWallKp(); }
    void setAngleOffset(float ao) { angle_offset = ao; }
    float getAngleOffset() const { return angle_offset; }
    void setMotorOffsets(int d, int e) { motor_d_offset = d; motor_e_offset = e; }
    int getMotorDOffset() const { return motor_d_offset; }
    int getMotorEOffset() const { return motor_e_offset; }
    void setSensorOffsetsMm(int16_t left, int16_t front, int16_t right) {
        if (ir) ir->setOffsetsMm(left, front, right);
    }
    void getSensorOffsetsMm(int16_t& left, int16_t& front, int16_t& right) const {
        if (ir) {
            left = ir->getLeftOffsetMm();
            front = ir->getFrontOffsetMm();
            right = ir->getRightOffsetMm();
        }
    }

    void turnFromAtoB(directions beggining, directions end);
    
    directions getFrontDir() const;

    directions getLeftDir() const;

    directions getRightDir() const;

    short getPosX() const;

    short getPosY() const;
};