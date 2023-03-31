#pragma once
#include "Constants.h"
#include "Motion.h"
#include "lib/geometry/TrapezoidProfile.h"

class ProfiledTurn : public Motion {
private:
    double _targetHeading;
    double _omega;
    double _alphaUp, _alphaDown;
    double threshold;
    uint32_t _start_time = 0;
    TrapezoidProfile _profile;

public:
    ProfiledTurn(double targetHeading, double omega = chassis::maxOmega, double alphaUp = chassis::maxAlphaUp, double alphaDown = chassis::maxAlphaDown, double threshold = 1);
    MotorVoltages calculateVoltages(kinState state) override;
    bool isSettled(kinState state) override;
};