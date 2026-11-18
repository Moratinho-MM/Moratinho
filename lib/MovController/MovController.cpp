#include "MovController.h"
#include <cmath>

bool MovController::begin(IRSensors* ir, IMU* imu, EngineDriver* leftEngine, EngineDriver* rightEngine)
{
    // Inner gyro PD gains (Ki = 0) and outer wall P gain
    float kp = 2.2f, ki = 0.0f, kd = 0.08f;
    pid.setParameters(kp, ki, kd);
    pid.setWallKp(0.12f);
    pid.resetGating();
    
    this->ir = ir;
    this->imu = imu;

    this->rightEngine = rightEngine;
    this->leftEngine = leftEngine;

    if (!leftEngine->isEncoderSetUp() || !rightEngine->isEncoderSetUp())
    {
        return false;
    }

    imu->IMU_READ();
    setPoint = pidController::wrap180(imu->getYaw());
    target_yaw = setPoint;
    return true;
}

void MovController::setPid(float kp, float ki, float kd)
{
    pid.setKp(kp);
    pid.setKi(0.0f); // Enforce Ki = 0 on both loops
    pid.setKd(kd);
    (void)ki;
}

void MovController::setWallKp(float kpWall)
{
    pid.setWallKp(kpWall);
}

unsigned long MovController::getDeltaTime()
{
    unsigned long currentTime = millis();
    unsigned long dt = currentTime - timerLasUpdate;
    timerLasUpdate = currentTime;
    return dt;
}

int MovController::computeLinearBias(float averageDistCm, float dist_front_mm) const
{
    // Step 1: Emergency collision cutoff
    if (dist_front_mm <= kFrontStopMm) {
        return 0;
    }

    constexpr int kMinStartBias    = 48;  // Overcomes motor static friction without slipping
    constexpr int kMinApproachBias = 42;  // Slow crawl when approaching wall or cell boundary
    constexpr float kAccelRampCm   = 3.5f; // Accelerate over first 3.5 cm of cell
    constexpr float kDecelRampCm   = 4.0f; // Decelerate over last 4.0 cm of cell (14.0 to 18.0 cm)

    int target_bias = bias;

    // 1. Acceleration ramp at start of cell (prevents initial wheel spin/slip)
    if (averageDistCm < kAccelRampCm) {
        float ratio = averageDistCm / kAccelRampCm;
        ratio = pidController::clampFloat(ratio, 0.0f, 1.0f);
        target_bias = kMinStartBias + static_cast<int>(ratio * (bias - kMinStartBias));
    }
    // 2. Deceleration ramp at end of cell (prevents skid on stop)
    else if (averageDistCm > (CELL_SIZE - kDecelRampCm)) {
        float dist_remaining = CELL_SIZE - averageDistCm;
        float ratio = dist_remaining / kDecelRampCm;
        ratio = pidController::clampFloat(ratio, 0.0f, 1.0f);
        target_bias = kMinApproachBias + static_cast<int>(ratio * (bias - kMinApproachBias));
    }

    // 3. Smooth front wall approach deceleration override
    if (dist_front_mm < kFrontTurnTriggerMm) {
        float ratio = (dist_front_mm - kIdealFrontStopMm) / (kFrontTurnTriggerMm - kIdealFrontStopMm);
        ratio = pidController::clampFloat(ratio, 0.0f, 1.0f);
        int front_bias = kMinApproachBias + static_cast<int>(ratio * (bias - kMinApproachBias));
        if (front_bias < target_bias) {
            target_bias = front_bias;
        }
    }

    return target_bias;
}

void MovController::do_pid(int linear_bias)
{
    (void)getDeltaTime();

    // Non-blocking ToF reads (mm)
    float dist_right_mm = ir->getRightDistanceMm();
    float dist_left_mm  = ir->getLeftDistanceMm();

    // Update IMU yaw and bias-corrected Z angular velocity
    imu->IMU_READ();
    float current_yaw = pidController::wrap180(imu->getYaw());
    float gyro_z_dps  = imu->getGyroZ();

    // Step 3: Outer loop (Lateral wall P-controller + consecutive ToF edge gating)
    float wall_angle_deg = pid.computeWallAngleOffset(
        dist_left_mm,  ir->wasLeftUpdated(),
        dist_right_mm, ir->wasRightUpdated(),
        kWallPresentMaxMm,
        kCenterTargetMm,
        kMaxStepDeltaMm,
        kMaxWallAngleDeg
    );

    // Step 5: Inner loop (Gyroscope PD controller around wrap180(theta_cell + wall_angle))
    target_yaw = pidController::wrap180(setPoint + wall_angle_deg);
    int error = pid.computeGyroPD(setPoint, wall_angle_deg, current_yaw, gyro_z_dps);

    if (linear_bias == 0) {
        motor_left_speed  = 0;
        motor_right_speed = 0;
        return;
    }

    motor_left_speed  = pid.leftMotorSpeed(-200, 200, linear_bias, error);
    motor_right_speed = pid.rightMotorSpeed(-200, 200, linear_bias, error);
}

void MovController::do_pid()
{
    float dist_front_mm = ir->getFrontDistanceMm();
    int linear_bias = computeLinearBias(0.0f, dist_front_mm);
    do_pid(linear_bias);
}

bool MovController::isFrontTurnTriggered()
{
    return ir->getFrontDistanceMm() < kFrontTurnTriggerMm;
}

bool MovController::isFrontEmergencyStop()
{
    return ir->getFrontDistanceMm() <= kFrontStopMm;
}

void MovController::alignFrontWall()
{
    #ifndef SIMULATION
    float dist_front_mm = ir->getFrontDistanceMm();
    // Only align if a front wall is actually present within a single cell
    if (dist_front_mm > 150.0f) {
        return;
    }

    constexpr float kFrontAlignKp = 1.4f;
    constexpr float kToleranceMm  = 3.0f;
    constexpr int   kMinAlignPwm  = 38; // Overcome motor deadband
    constexpr int   kMaxAlignPwm  = 65;
    unsigned long startMs = millis();

    while ((millis() - startMs) < 350) {
        dist_front_mm = ir->getFrontDistanceMm();
        float dist_err = dist_front_mm - kIdealFrontStopMm;
        if (std::fabs(dist_err) <= kToleranceMm) {
            break;
        }

        imu->IMU_READ();
        float current_yaw = pidController::wrap180(imu->getYaw());
        int rot_pwm = pid.computeGyroPD(setPoint, 0.0f, current_yaw, imu->getGyroZ());

        float effort = kFrontAlignKp * dist_err;
        int lin_pwm = 0;
        if (effort > 0.0f) {
            lin_pwm = kMinAlignPwm + static_cast<int>(pidController::clampFloat(effort, 0.0f, static_cast<float>(kMaxAlignPwm - kMinAlignPwm)));
        } else {
            lin_pwm = -kMinAlignPwm - static_cast<int>(pidController::clampFloat(-effort, 0.0f, static_cast<float>(kMaxAlignPwm - kMinAlignPwm)));
        }

        leftEngine->turnOn(pid.leftMotorSpeed(-200, 200, lin_pwm, rot_pwm));
        rightEngine->turnOn(pid.rightMotorSpeed(-200, 200, lin_pwm, rot_pwm));
        delay(2);
    }
    brake();
    delay(20);
    stop();
    #endif
}

void MovController::moveFront() 
{
    do_pid();
    leftEngine->turnOn(motor_left_speed);
    rightEngine->turnOn(motor_right_speed);
}

void MovController::moveFront(int linear_bias)
{
    do_pid(linear_bias);
    leftEngine->turnOn(motor_left_speed);
    rightEngine->turnOn(motor_right_speed);
}

void MovController::stop()
{
    leftEngine->turnOff();
    rightEngine->turnOff();
}

void MovController::brake()
{
    leftEngine->brake();
    rightEngine->brake();
}

void MovController::moveCell()
{
    #ifdef SIMULATION
    SIMULATOR::moveForward();
    posX += offsetX(globalDirection.front); 
    posY += offsetY(globalDirection.front);
    #else
    leftEngine->resetEncoderMeasure();
    rightEngine->resetEncoderMeasure();
    pid.resetGating();

    float averageDistance = 0.0f;
    
    while (averageDistance < CELL_SIZE) {
        float dist_front_mm = ir->getFrontDistanceMm();

        // Stop early if front sensor detects wall at ideal cell-center stop distance
        if (dist_front_mm <= kIdealFrontStopMm) {
            break;
        }

        int linear_bias = computeLinearBias(averageDistance, dist_front_mm);
        if (linear_bias == 0) {
            break;
        }

        moveFront(linear_bias);

        // Absolute encoder counts protect against reversed-polarity encoder channels
        long leftTicks = std::labs(leftEngine->getEncoderMeasure());
        long rightTicks = std::labs(rightEngine->getEncoderMeasure());
        averageDistance = ((leftTicks + rightTicks) / 2.0f) / TICKS_CM;
        delay(2); // ~500 Hz control loop tick
    }

    brake();
    delay(25);
    stop();

    // Logical position updated only after successful traversal
    posX += offsetX(globalDirection.front); 
    posY += offsetY(globalDirection.front);

    delay(20);
    #endif
}

void MovController::moveCells(int count)
{
    for (int i = 0; i < count; ++i) {
        moveCell();
        if (isFrontEmergencyStop() || ir->getFrontDistanceMm() <= kIdealFrontStopMm) {
            break;
        }
    }
}

void MovController::executeTurnTo(float target_yaw_deg)
{
    setPoint = pidController::wrap180(target_yaw_deg);
    target_yaw = setPoint;
    pid.resetGating();

    constexpr float kTurnKp = 1.8f;
    constexpr float kTurnKd = 0.05f;
    constexpr int kMinTurnPwm = 42;  // Overcomes motor deadband
    constexpr int kMaxTurnPwm = 70;  // Prevents aggressive wheel slip
    constexpr float kTurnToleranceDeg = 2.0f;
    constexpr unsigned long kMaxTurnTimeMs = 1200;

    unsigned long startMs = millis();
    int inToleranceCount = 0;

    while ((millis() - startMs) < kMaxTurnTimeMs) {
        imu->IMU_READ();
        float current_yaw = pidController::wrap180(imu->getYaw());
        float err = pidController::wrap180(setPoint - current_yaw);

        if (std::fabs(err) <= kTurnToleranceDeg) {
            inToleranceCount++;
            if (inToleranceCount >= 5) { // Settled for ~10 ms within tolerance
                break;
            }
        } else {
            inToleranceCount = 0;
        }

        float gyro_z = imu->getGyroZ();
        float effort = (kTurnKp * err) - (kTurnKd * gyro_z);

        // Apply deadband compensation
        int pwm = 0;
        if (effort > 0.0f) {
            pwm = kMinTurnPwm + static_cast<int>(pidController::clampFloat(effort, 0.0f, static_cast<float>(kMaxTurnPwm - kMinTurnPwm)));
            // err > 0 means current_yaw < setPoint: need to turn RIGHT (+yaw)
            // Differential drive: Left wheel forward, Right wheel reverse
            leftEngine->turnOn(pwm);
            rightEngine->turnOn(-pwm);
        } else {
            pwm = kMinTurnPwm + static_cast<int>(pidController::clampFloat(-effort, 0.0f, static_cast<float>(kMaxTurnPwm - kMinTurnPwm)));
            // err < 0 means current_yaw > setPoint: need to turn LEFT (-yaw)
            // Differential drive: Left wheel reverse, Right wheel forward
            leftEngine->turnOn(-pwm);
            rightEngine->turnOn(pwm);
        }

        delay(2);
    }

    brake();
    delay(30);
    stop();

    leftEngine->resetEncoderMeasure();
    rightEngine->resetEncoderMeasure();
    pid.resetGating();
    delay(40);
}

void MovController::turnLeft()
{
    alignFrontWall();
    globalDirection.turnLeft();
    #ifdef SIMULATION
    SIMULATOR::turnLeft();
    #else
    executeTurnTo(setPoint - 90.0f);
    #endif
}

void MovController::turnRight()
{
    alignFrontWall();
    globalDirection.turnRight();
    #ifdef SIMULATION
    SIMULATOR::turnRight();
    #else
    executeTurnTo(setPoint + 90.0f);
    #endif
}

void MovController::turn180degrees()
{
    alignFrontWall();
    globalDirection.turn180degrees();
    #ifdef SIMULATION
    SIMULATOR::turnRight();
    SIMULATOR::turnRight();
    #else
    executeTurnTo(setPoint + 180.0f);
    #endif
}

void MovController::turnDegrees(float angle)
{
    #ifdef SIMULATION
    // no-op in simulation
    #else
    executeTurnTo(setPoint + angle);
    #endif
}

void MovController::turnFromAtoB(directions beggining, directions end)
{
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
    default: 
        ErrorHandler::throwError("Tried to turn to ilegal direction", "MovController: turnFromAtoB");
        break;
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