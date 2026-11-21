#pragma once

#include <VL53L0X.h>
#include <Wire.h>
#include "../ErrorHandler/ErrorHandler.h"

class IRSensors 
{
    private:
    VL53L0X rightIfs;
    VL53L0X frontIfs;
    VL53L0X leftIfs;

    float lastRightDist = 100.0f;
    float lastFrontDist = 100.0f;
    float lastLeftDist  = 100.0f;

    // Static calibration offsets in mm (measured dead-center in a 180mm cell)
    int16_t rightOffsetMm = 0;
    int16_t frontOffsetMm = 0;
    int16_t leftOffsetMm  = 33;

    // Flags indicating whether a fresh non-blocking sample arrived on the latest read
    bool rightUpdated = false;
    bool frontUpdated = false;
    bool leftUpdated  = false;

    float readFilteredDistance(VL53L0X& sensor, float& lastValid, int16_t offsetMm, bool& didUpdate);
    
    public:
    void begin(int x_shut_f, int xshut_r, int xshut_l);

    void setOffsetsMm(int16_t leftOffset, int16_t frontOffset, int16_t rightOffset);
    int16_t getLeftOffsetMm() const { return leftOffsetMm; }
    int16_t getFrontOffsetMm() const { return frontOffsetMm; }
    int16_t getRightOffsetMm() const { return rightOffsetMm; }

    float getRightDistance();
    float getFrontDistance();
    float getLeftDistance();

    float getRightDistanceMm();
    float getFrontDistanceMm();
    float getLeftDistanceMm();

    bool wasRightUpdated() const { return rightUpdated; }
    bool wasFrontUpdated() const { return frontUpdated; }
    bool wasLeftUpdated()  const { return leftUpdated; }

    void showDistances();
};