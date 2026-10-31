#pragma once

#include <string>
#include "BluetoothHandler.h"

class SIMULATOR {

    public:

    static int mazeWidth();
    static int mazeHeight();

    static bool wallFront();
    static bool wallRight();
    static bool wallLeft();

    static void moveForward();
    static void turnRight();
    static void turnLeft();

    static void setWall(int x, int y, char direction);
    static void clearWall(int x, int y, char direction);

    //static void setColor(int x, int y, char color);
    //static void clearColor(int x, int y);
    //static void clearAllColor();

    static void setDistance(int x, int y, const int d);
    static void clearDistance(int x, int y);
    static void clearAllDistances();

    //static bool wasReset();
    //static void ackReset();

};