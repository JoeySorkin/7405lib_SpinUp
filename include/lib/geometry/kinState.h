#pragma once
#include "Pose.h"

class kinState{
private:
    struct substate{
        double x;
        double y;
        double theta;
    } velo, accel;

public:
    Pose position;
    substate velocity();
    substate acceleration();
    void updateVelocity(double _x, double _y, double _h);
    void updateAcceleration(double _x, double _y, double _h);
};
