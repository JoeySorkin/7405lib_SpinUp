#include "Shooter.h"
#include "Constants.h"
#include "pros/rtos.hpp"
#include <cstddef>
#include <cstdio>
#include <valarray>
#include "Controller.h"

Shooter* Shooter::INSTANCE = nullptr;

Shooter::Shooter()
    : scataMotor(ports::scataMotor, true), scataRotation(ports::scataRotation, true), rightExpansionPiston(ports::rightExpansion),
    leftExpansionPiston(ports::leftExpansion), toggleBoostPiston(1), scataState(state::READY){}

void Shooter::initialize(){
    scataRotation.reset_position();
    shooter_task = pros::c::task_create([](void* _) { sShooter->shooterRunner(_); }, nullptr, TASK_PRIORITY_DEFAULT,
	                                 TASK_STACK_DEPTH_DEFAULT, "Shooter Task");
}

void Shooter::shooterRunner(void* params){
    while (true){
        if((scataState == state::FIRING) && (std::abs(scataRotation.get_position()-prevPos) < 500)){
            scataMotor.move_voltage(12000);
        }
        else {
            if (scataRotation.get_position() < scata::lowPowerThreshold){
                scataMotor.move_voltage(scata::highPower);
                scataState = state::RECHARGE;
            }
            else if (scataRotation.get_position() < scata::stopThreshold){
                scataMotor.move_voltage(scata::lowPower);
                scataState = state::RECHARGE;
            }
            else {
                scataMotor.move_voltage(0);
                scataState = state::READY;
            }


            if (sController->getDigital(Controller::l1) && scataState == state::READY){
                scataState = state::FIRING;
            }
        }

        prevPos = scataRotation.get_position();

        pros::delay(20);
    }
}

Shooter::state Shooter::getState(){
    return scataState;
}

void Shooter::fireScata(){
    scataState = state::FIRING;
}