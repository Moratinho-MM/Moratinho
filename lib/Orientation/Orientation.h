#pragma once

#include "../Coordinates/Coordinates.h"

class Orientation {
    
    public:
    directions front;
    directions right;
    directions left;
    
    Orientation();

    void turnLeft();
    void turnRight();
    void turn180degrees();
    void turnFromAtoB(directions beggining, directions end);
};