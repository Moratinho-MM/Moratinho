#include "Coordinates.h"
#include "../ErrorHandler/ErrorHandler.h"

char translateToAPI(directions dir) {
    char apiDirections[] = {'n', 's', 'e', 'w'};
    return apiDirections[dir];
}

const directions allDirections[4] = {NORTH, SOUTH, EAST, WEST};

short offsetX(directions dir)
{
    switch (dir)
    {
    case EAST: return 1; break;
    case WEST: return -1; break;
    default: return 0; break;
    }
}

short offsetY(directions dir)
{
    switch (dir)
    {
    case NORTH: return 1; break;
    case SOUTH  : return -1; break;
    default   : return 0; break;
    }
}

directions opositeSide(directions dir)
{
    switch (dir)
    {
    case NORTH: return SOUTH;
    case SOUTH  : return NORTH;
    case EAST: return WEST;
    case WEST: return EAST;
    
    default: ErrorHandler::throwError("Invalid direction was given!", "Coordinates.cpp");
    }
}

directions leftSide(directions dir) {
    switch (dir)
    {
        case NORTH: return WEST;
        case SOUTH: return EAST;
        case EAST: return NORTH;
        case WEST: return SOUTH;
        default: ErrorHandler::throwError("Invalid direction was given!", "Coordinates.cpp");
    }
}

directions rightSide(directions dir) {
    switch (dir)
    {
        case NORTH: return EAST;
        case SOUTH: return WEST;
        case EAST: return SOUTH;
        case WEST: return NORTH;
        default: ErrorHandler::throwError("Invalid direction was given!", "Coordinates.cpp");
    }
}
