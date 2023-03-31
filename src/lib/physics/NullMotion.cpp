//
// Created by Joey Sorkin on 3/24/23.
//
#include "lib/physics/NullMotion.h"
#include "lib/geometry/kinState.h"

Motion::MotorVoltages NullMotion::calculateVoltages(kinState state) {
    return {0.0,0.0};
}

bool NullMotion::isSettled(kinState state){
    return false;
}