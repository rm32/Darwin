#include "Item.hpp"



Item::Item()
{
    x =0.0;
    y =0.0;
    isFound = false;
}

double Item::getX() const
{
    return x;
}

double Item::getY() const
{
    return y;
}

bool Item::checkIfFound() const
{
    return isFound;
}

double Item::getMaxPos(int i)
{
    return maxPos[i];
}

double Item::getMinPos(int i)
{
    return minPos[i];
}