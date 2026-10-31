#include "MPU6050_Goytacaz.h"


void IMU::begin()
{
    dmpReady = false;
    IMU_CONFIG();
}

void IMU::IMU_CONFIG() {

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having
                               // compilation difficulties
    #endif

    IMU_BOOT();
}

void IMU::IMU_BOOT() {
    // initialize device
    const char* emitter = "MPU6050_Goytacaz";
    
    ErrorHandler::throwWarning("Initializing IMU, pease wait...", emitter);
    mpu.initialize();
    // verify connection

    bool mpuSuccess = mpu.testConnection();

    mpuSuccess ? ErrorHandler::throwWarning("IMU initialization successfull.", emitter) :
    ErrorHandler::throwError("IMU initialization failed.", emitter);
    
    // load and configure the DMP
    devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(3.00000);
    mpu.setYGyroOffset(-60.00000);
    mpu.setZGyroOffset(12.00000);
    mpu.setZAccelOffset(896.00000); 
    // 1688 factory default for my test chip
    // make sure it worked (returns 0 if so)

    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        // mpu.PrintActiveOffsets();
        //  turn on the DMP, now that it's ready
        // Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        // mpuIntStatus = mpu.getIntStatus();

        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }

    else ErrorHandler::throwError("Failed to configure DMP.", emitter);
}

void IMU::IMU_READ() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet

        #ifdef OUTPUT_READABLE_YAWPITCHROLL

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        // Serial.print("yaw:");
        yaw = ypr[0] * 180 / M_PI;

        #endif
    }
}

float IMU::getYaw() const
{
    return yaw;
}
