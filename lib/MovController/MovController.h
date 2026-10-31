
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
    short posX, posY = 0;

    static constexpr float TICKS_CM = (12.0f * 10.0f) / (PI * WHEEL_DIAM);
    
    EngineDriver* leftEngine;
    EngineDriver* rightEngine;

    IRSensors* ir; IMU* imu;

    //********** PID **********
    pidController pid;
    float setPoint = 0;
    float target_yaw = 0.0f;
    int bias = 90;
    
    const float WALL_TRESHOLD = 15.0f;     // Distância máxima para considerar que existe uma parede (ex: mm)
    const float WALL_KEEP_DISTANCE = 9.0f; // Distância do centro do robô até a parede
    const float BASE_VELOCITY = 100;

    /* offsets */
    int motor_d_offset = 0;
    int motor_e_offset = 1;

    float angle_offset = 32.0;

    /* Egine speed variables*/
    int motor_left_speed;
    int motor_right_speed;

    unsigned long timerLasUpdate = 0;

    public:
    MovController() = default;

    bool begin(IRSensors* ir, IMU* imu, EngineDriver* leftEngine, EngineDriver* rightEngine);

    void setPid(float kp, float ki, float kd);

    unsigned long getDeltaTime();
    
    private : void do_pid();

    public:
    void moveFront();

    void moveCell();

    void stop();

    void turnLeft();

    void turnRight();

    void turn180degrees();

    void turnFromAtoB(directions beggining, directions end);
    
    directions getFrontDir() const;

    directions getLeftDir() const;

    directions getRightDir() const;

    short getPosX() const;

    short getPosY() const;
};
