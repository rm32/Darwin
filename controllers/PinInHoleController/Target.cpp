#include "Target.hpp"


Target::Target()
{
    xAxis =0.0;
    yAxis =0.0;
    isFound = false;
}

double Target::getXAxis() const
{
    return xAxis;
}

double Target::getYAxis() const
{
    return yAxis;
}

bool Target::checkIfFound() const
{
    return isFound;
}

double Target::getMaxLimbPos(int i)
{
    return maxLimbPos[i];
}

double Target::getMinLimbPos(int i)
{
    return minLimbPos[i];
}