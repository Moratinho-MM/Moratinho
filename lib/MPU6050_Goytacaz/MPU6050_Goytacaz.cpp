#include "MPU6050_Goytacaz.h"

void IMU::begin()
{
    dmpReady = false;
    IMU_CONFIG();
}

void IMU::IMU_CONFIG() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock
    #endif

    IMU_BOOT();
}

float IMU::readRawGyroZDps()
{
    int16_t rawGz = mpu.getRotationZ();
    // Negated to match ypr[0] yaw sign convention (+yaw = increasing angle)
    return -(static_cast<float>(rawGz) / gyroScaleLsbPerDps);
}

void IMU::calibrateGyroBias(uint16_t samples)
{
    // Keep robot completely still for ~500 ms after power-on and average 500 Z-axis samples
    double sum = 0.0;
    for (uint16_t i = 0; i < samples; ++i) {
        sum += static_cast<double>(readRawGyroZDps());
        delay(1);
    }
    gyroZBiasDps = static_cast<float>(sum / static_cast<double>(samples));
}

void IMU::IMU_BOOT() {
    const char* emitter = "MPU6050_Goytacaz";
    
    ErrorHandler::throwWarning("Initializing IMU, pease wait...", emitter);
    mpu.initialize();

    bool mpuSuccess = mpu.testConnection();
    mpuSuccess ? ErrorHandler::throwWarning("IMU initialization successfull.", emitter) :
                 ErrorHandler::throwError("IMU initialization failed.", emitter);
    
    // Load and configure the DMP
    devStatus = mpu.dmpInitialize();

    // Supply baseline gyro/accel offsets
    mpu.setXGyroOffset(3.00000);
    mpu.setYGyroOffset(-60.00000);
    mpu.setZGyroOffset(12.00000);
    mpu.setZAccelOffset(896.00000); 

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);

        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        ErrorHandler::throwError("Failed to configure DMP.", emitter);
    }

    // Determine active gyro full-scale sensitivity after DMP/IMU setup
    switch (mpu.getFullScaleGyroRange()) {
        case MPU6050_GYRO_FS_250:  gyroScaleLsbPerDps = 131.0f; break;
        case MPU6050_GYRO_FS_500:  gyroScaleLsbPerDps = 65.5f;  break;
        case MPU6050_GYRO_FS_1000: gyroScaleLsbPerDps = 32.8f;  break;
        case MPU6050_GYRO_FS_2000:
        default:                   gyroScaleLsbPerDps = 16.4f;  break;
    }

    // 500 ms stationary bias calibration
    calibrateGyroBias(500);
}

void IMU::IMU_READ() {
    // Update bias-corrected Z-axis angular velocity every tick (zero phase lag for PD derivative)
    gyroZDps = readRawGyroZDps() - gyroZBiasDps;

    if (!dmpReady) return;

    // Read latest packet from DMP FIFO if available
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        yaw = wrap180(ypr[0] * 180.0f / M_PI);
        #endif
    }
}

float IMU::getYaw() const
{
    return yaw;
}

float IMU::getGyroZ() const
{
    return gyroZDps;
}

float IMU::wrap180(float angle)
{
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle <= -180.0f) angle += 360.0f;
    return angle;
}