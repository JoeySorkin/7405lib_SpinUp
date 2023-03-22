//
// Created by Joey Sorkin on 10/4/22.
//

#ifndef INC_7405MSPINUP_TIMEOUT_H
#define INC_7405MSPINUP_TIMEOUT_H

#include "../../main.h"

class Timeout
{
private:
    uint32_t _time;
    uint32_t _timeout;

public:
    Timeout(uint32_t timeout)
    {
        _time = pros::c::millis();
        _timeout = timeout;
    }
    bool timedOut() const { return (pros::c::millis() - _time) > _timeout; }
};
#endif // INC_7405MSPINUP_TIMEOUT_H