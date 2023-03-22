#include "Drive.h"

Drive *Drive::INSTANCE = nullptr;

void Drive::initialize()
{
    task = pros::c::task_create([](void *_)
                                { sDrive->runner(_); },
                                nullptr,
                                TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT,
                                "Drive");
}

void Drive::runner(void *ignored)
{
    Motion currentMotion = Motion::NullMotion();
    while (true)
    {
        // get current motion
        currentMotion = getCurrentMotion(200); // TODO: add thread overload error thingy

        // get computed motor voltages
        auto motor_volts = currentMotion.calculateVoltages(sOdom->getCurrentState());

        // set those
        setVoltageLeft(motor_volts.left);
        setVoltageRight(motor_volts.right);

        // check if motion has timed out (if it has, set current motion to nullmotion)
        if (isTimedOut)
        {
            currentMotion = Motion::NullMotion();
            isTimedOut = false;
        }

        pros::delay(20);
    }
}

bool Drive::waitUntilSettled(uint32_t timeout)
{
    Timeout timeout = Timeout(timeout);

    while (!timeout.timedOut())
    {
        if (isSettled.load())
        {
            isSettled = false;
            return true;
        }
    }

    // Timed out
    isTimedOut = true;

    // wait until the Drive thread realizes we timed out
    while (isTimedOut)
    {
        pros::delay(20);
    }

    return false;
}
