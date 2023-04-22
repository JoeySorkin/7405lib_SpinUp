#include "Shooter.h"
#include "Constants.h"
#include "pros/rtos.hpp"
#include <cstddef>
#include <valarray>
#include "Controller.h"

Shooter* Shooter::INSTANCE = nullptr;

Shooter::Shooter()
    : scataMotor(ports::scataMotor), scataRotation(ports::scataRotation), leftBoostPiston({5, 'B'}, false), 
    rightBoostPiston({5, 'H'}, true), toggleBoostPiston({5, 'A'}, false), rightExpansionPiston(ports::rightExpansion),
    leftExpansionPiston(ports::leftExpansion){}

void Shooter::initialize(){
    scataRotation.reset();
    shooter_task = pros::c::task_create([](void* _) { sShooter->shooterRunner(_); }, nullptr, TASK_PRIORITY_DEFAULT,
	                                 TASK_STACK_DEPTH_DEFAULT, "Shooter Task");
}

void Shooter::shooterRunner(void* params){
    while (true){

        //pull back cata, only allow shootingflag to turn true if cata is pulled back.
        //high power when scata is far from being at bottom, low power when scata is
        //close to being fully pulled back so it pullslower and we dont overshoot and missfire
        if (scataRotation.get_position() < scata::lowPowerThreshold){
            scataMotor.move_voltage(scata::highPower);
        }
        else if (scataRotation.get_position() < scata::stopThreshold){
            scataMotor.move_voltage(scata::lowPower);
        }
        else {
            isReady = true;
        }

        if (sController->getDigital(Controller::l1) && isReady){
            fireScata();
        }


        //detects when to fire. if the difference in current and last position changed 
        //significantly, it means the scata fired, so we turn isShooting off and depower
        //the scata
        if (isShooting && std::abs(scataRotation.get_position() - prevPos) < 500){
            scataMotor.move_voltage(12000);
        }
        else {
            toggleBoost(false);
            isShooting = false;
            scataMotor.move_voltage(0);
        }


        if (sController->getDigital(Controller::l2)){
            toggleBoost(true);
        }

        prevPos = scataRotation.get_position();

        pros::delay(20);
    }
}

void Shooter::fireScata(){
    isShooting = true;
    isReady = false;
}

void Shooter::toggleBoost(bool toggle){
    if (toggle && isReady){
        leftBoostPiston.set_value(true);
        rightBoostPiston.set_value(true);
    }
    else {
        leftBoostPiston.set_value(false);
        rightBoostPiston.set_value(false);
    }
}

void Shooter::fireExpansion(){
    leftExpansionPiston.set_value(true);
    rightExpansionPiston.set_value(true);
}