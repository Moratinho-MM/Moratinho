#pragma once

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include "I2Cdev.h"
#include "../../libdeps/MPU6050/MPU6050_6Axis_MotionApps20.h"
#include "../ErrorHandler/ErrorHandler.h"

#define OUTPUT_READABLE_YAWPITCHROLL

// Variáveis úteis
extern volatile float yaw;

class IMU 
{
    private:
    MPU6050 mpu;

    // MPU control/status vars
    bool dmpReady;          // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    // orientation/motion vars
    Quaternion q;                    // [w, x, y, z]         quaternion container
    VectorFloat gravity;             // [x, y, z]            gravity vector
    float ypr[3];                    // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    volatile float yaw;

    public  : void begin();

    public  : void IMU_CONFIG();

    private : void IMU_BOOT();

    public:

    void IMU_READ();

    float getYaw() const;
};