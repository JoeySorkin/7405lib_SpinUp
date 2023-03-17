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

    kinState(Pose pos, double xvelo, double yvelo, double hvelo, double xaccel, double yaccel, double haccel): 
    position(pos), velo_state((substate){xvelo, yvelo, hvelo}), accel_state((substate){xaccel, yaccel, haccel}){};

    kinState(double xpos, double ypos, double hpos, double xvelo, double yvelo, double hvelo, double xaccel, double yaccel, double haccel): 
    position(xpos, ypos, hpos), velo_state((substate){xvelo, yvelo, hvelo}), accel_state((substate){xaccel, yaccel, haccel}){};

    Pose position;
    substate velocity();
    substate acceleration();

    void setVelocity(double _x, double _y, double _h);
    void setAcceleration(double _x, double _y, double _h);
    kinState predictFuture(double dt); //somebody please help me name things
};
