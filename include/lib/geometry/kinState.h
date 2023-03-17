#pragma once
#include "Pose.h"

class kinState{
private:
    struct substate{
        double x;
        double y;
        double theta;
    } velo_state, accel_state;

public:
    kinState();
    
    kinState(Pose pos, substate velo, substate accel): position(pos), velo_state(velo), accel_state(accel){};

    Pose position;
    substate velocity();
    substate acceleration();

    void setVelocity(double _x, double _y, double _h);
    void setAcceleration(double _x, double _y, double _h);
    kinState predictFuture(double dt); //somebody please help me name things
};
