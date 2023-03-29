//
// Created by Joey Sorkin on 3/29/23.
//

#ifndef INC_7405SPINUP_TIMEDMOTION_H
#define INC_7405SPINUP_TIMEDMOTION_H

#include "Motion.h"
class TimedMotion : public Motion {
  MotorVoltages calculateVoltages(kinState state) override;
};


#endif // INC_7405SPINUP_TIMEDMOTION_H
