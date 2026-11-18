#pragma once

typedef enum directions
{
    NORTH,
    SOUTH,
    EAST,
    WEST,
    INVALID_DIRECTION
} directions;

extern const directions allDirections[4];

short offsetX(directions dir);
short offsetY(directions dir);
directions opositeSide(directions dir);
directions leftSide(directions dir);
directions rightSide(directions dir);