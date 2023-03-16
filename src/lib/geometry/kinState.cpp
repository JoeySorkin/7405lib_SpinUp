#include "lib/geometry/kinState.h"

kinState::substate kinState::acceleration(){
    return accel;
}

kinState::substate kinState::velocity(){
    return velo;
}

void kinState::updateVelocity(double _x, double _y, double _h){
    velo.x = _x;
    velo.y = _y;
    velo.theta = _h;
}

void kinState::updateAcceleration(double _x, double _y, double _h){
    accel.x = _x;
    accel.y = _y;
    accel.theta = _h;
}