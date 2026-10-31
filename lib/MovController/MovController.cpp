#include "MovController.h"


bool MovController::begin(IRSensors* ir, IMU* imu, EngineDriver* leftEngine, EngineDriver* rightEngine)
{
    float kp=2.0, ki = 0.0, kd = 0.01; // Default values
    pid.setParameters(kp, ki, kd);
    
    this->ir = ir; this->imu = imu;

    this->rightEngine = rightEngine;
    this->leftEngine =  leftEngine;

    if (!leftEngine->isEncoderSetUp() || !rightEngine->isEncoderSetUp())
    {
        // throw error
        return false;
    }
    return true;
}

void MovController::setPid(float kp, float ki, float kd){
    pid.setKp(kp);
    pid.setKi(ki);
    pid.setKd(kd);
}

unsigned long MovController::getDeltaTime()
{
    unsigned long currentTime = millis();
    unsigned long dt = currentTime - timerLasUpdate;
    timerLasUpdate = currentTime;
    return dt;
}

void MovController::do_pid()
{
    unsigned long deltaTime = getDeltaTime();
    float rightDistance = ir->getRightDistance();
    float leftDistance = ir->getLeftDistance();
    float yaw = imu->getYaw();

    int error = 0; // Sinal de controle que irá para os motores
    
    bool has_left_wall = (leftDistance < WALL_TRESHOLD);
    bool has_right_wall = (rightDistance < WALL_TRESHOLD);
    
    if (has_left_wall && has_right_wall)
    {
        // AMBAS AS PAREDES: Centralizar
        // Se as distâncias são iguais, o feedback (esq - dir) será 0.
        float set_point = 0.0f;
        float feedback = leftDistance - rightDistance;
        
        error = pid.pidCalculation(setPoint, yaw, deltaTime, false);
    
        target_yaw = yaw;
    }

    else if (has_left_wall) 
    {
        // APENAS PAREDE ESQUERDA: Manter distância ideal
        float setpoint = WALL_KEEP_DISTANCE;
        error = pid.pidCalculation(setpoint, leftDistance, deltaTime, false);
        target_yaw = yaw;
    
    } 
    else if (has_right_wall) 
    {
        // APENAS PAREDE DIREITA: Manter distância ideal
        float setpoint = WALL_KEEP_DISTANCE;
        // O sinal do 'u' pode precisar ser invertido aqui (ex: -u) dependendo de 
        // qual lado aumenta/diminui a velocidade nos motores.
        error = -pid.pidCalculation(setpoint, rightDistance, deltaTime, false);
        target_yaw = yaw;
    }
    else
    {
        // NENHUMA PAREDE: Usar a IMU (Giroscópio)
        // Usa a instância do PID configurada para ângulos (com o limite de 180)
        error = pid.pidCalculation(target_yaw, yaw, deltaTime, true);
    }

    motor_left_speed = pid.leftMotorSpeed(-200, 200, bias, error);
    motor_right_speed = pid.rightMotorSpeed(-200, 200, bias, error);
    
}

void MovController::moveFront() 
{
    imu->IMU_READ();
    do_pid();
    leftEngine->turnOn(motor_left_speed);
    rightEngine->turnOn(motor_right_speed);
}

void MovController::stop()
{
    leftEngine->turnOff();
    rightEngine->turnOff();
}
void MovController::moveCell()
{
    posX += offsetX(globalDirection.front); 
    posY += offsetY(globalDirection.front);
    
    #ifdef SIMULATION
    SIMULATOR::moveForward();
    #else
    #endif
}

void MovController::turnLeft()
{
    /* TODO : IMPLEMENTAR O CONTOLE DO MOTOR */
    globalDirection.turnLeft();
    #ifdef SIMULATION
    SIMULATOR::turnLeft();
    #else
    #endif

}

void MovController::turnRight()
{
    /* TODO : IMPLEMENTAR O CONTOLE DO MOTOR */
    globalDirection.turnRight();
    #ifdef SIMULATION
    SIMULATOR::turnRight();
    #else
    #endif
}

void MovController::turn180degrees()
{
    /* TODO : IMPLEMENTAR O CONTOLE DO MOTOR */
    #ifdef SIMULATION
    SIMULATOR::turnRight();
    SIMULATOR::turnRight();
    #else
    #endif
}

void MovController::turnFromAtoB(directions beggining, directions end)
{
    globalDirection.turnFromAtoB(beggining, end);
    switch (beggining)
    {
    case NORTH:
        switch (end)
        {
        case EAST: turnRight(); break;
        case WEST: turnLeft(); break;
        case SOUTH: turn180degrees(); break;
        default: break;
        } break;
    case SOUTH:
        switch (end)
        {
        case EAST: turnLeft(); break;
        case WEST: turnRight(); break;
        case NORTH: turn180degrees(); break;
        default: break;
        } break;
    case EAST:
        switch (end)
        {
        case NORTH: turnLeft(); break;
        case SOUTH: turnRight(); break;
        case WEST: turn180degrees(); break;
        default: break;
        } break;
    case WEST:
        switch (end)
        {
        case SOUTH: turnLeft(); break;
        case NORTH: turnRight(); break;
        case EAST: turn180degrees(); break;
        default: break;
        } break;
    }
}
directions MovController::getFrontDir() const
{
    return globalDirection.front;
}

directions MovController::getLeftDir() const
{
    return globalDirection.left;
}

directions MovController::getRightDir() const
{
    return globalDirection.right;
}

short MovController::getPosX() const 
{
    return posX;
}

short MovController::getPosY() const 
{
    return posY;
}

