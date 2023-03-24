//
// Created by Joey Sorkin on 3/24/23.
//

#ifndef INC_7405SPINUP_NULLMOTION_H
#define INC_7405SPINUP_NULLMOTION_H
#include "Motion.h"
class NullMotion : public Motion {
  MotorVoltages calculateVoltages(kinState state) override;
};

#endif // INC_7405SPINUP_NULLMOTION_H
