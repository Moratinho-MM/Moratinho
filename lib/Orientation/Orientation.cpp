#include "Orientation.h"

Orientation::Orientation()
{
    // Norte é adotado como a direção padrão de inicio, isso não se refere 
    // ao Norte geográfico, mas sim a uma forma de relacionar as direções
    // com a direçao inicial

    front = NORTH;
    right = rightSide(front);
    left = leftSide(front);
}

void Orientation::turnLeft()
{
    front = leftSide(front);
    right = rightSide(front);
    left = leftSide(front);
}

void Orientation::turnRight()
{
    front = rightSide(front);
    right = rightSide(front);
    left = leftSide(front);
}

void Orientation::turn180degrees()
{        
    front = opositeSide(front);
    right = rightSide(front);
    left = leftSide(front);
}

void Orientation::turnFromAtoB(directions beggining, directions end)
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
    }
    
}
