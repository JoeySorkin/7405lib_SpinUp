
#include "Robot.h"

void Robot::initialize()
{
    pros::lcd::initialize();
}

void Robot::setOpMode(Robot::OpMode op)
{
    opmode.store(op);
    switch (op)
    {
    case DRIVER:
        // sDrive->setBrakeMode(MOTOR_BRAKE_COAST);
        break;
    case AUTONOMOUS:
        // sDrive->setBrakeMode(MOTOR_BRAKE_HOLD);
        break;
    }
}

Robot::OpMode Robot::getOpMode()
{
    return opmode.load();
}
