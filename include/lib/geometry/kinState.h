#pragma once

#include "Pose.h"

class kinState{
public:
  struct substate{
    double x;
    double y;
    double theta;
  };

private:
    substate velo_state, accel_state;

public:
    kinState() = default;
    kinState(Pose pos, substate velo, substate accel): position(pos), velo_state(velo), accel_state(accel){};

    Pose position;
    substate velocity();
    substate acceleration();

    void setVelocity(double _x, double _y, double _h);
    void setAcceleration(double _x, double _y, double _h);
    kinState predict(double dt);
};
