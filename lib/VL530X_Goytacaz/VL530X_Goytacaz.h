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

    public:
    void begin(int x_shut_f, int xshut_r, int xshut_l);

    float getRightDistance();

    float getFrontDistance();

    float getLeftDistance();

    void showDistances();

};