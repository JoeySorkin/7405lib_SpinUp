#include "Shooter.h"
#include "Constants.h"
#include "pros/rtos.hpp"
#include <cstddef>
#include <cstdio>
#include <valarray>
#include "Controller.h"

Shooter* Shooter::INSTANCE = nullptr;

Shooter::Shooter()
    : scataMotor(ports::scataMotor, true), scataRotation(ports::scataRotation, true), leftBoostPiston({5, 'B'}, false), 
    rightBoostPiston({5, 'H'}, true), toggleBoostPiston({5, 'A'}, false), rightExpansionPiston(ports::rightExpansion),
    leftExpansionPiston(ports::leftExpansion), isShooting(false), isBoosted(false){}

void Shooter::initialize(){
    scataRotation.reset_position();
    shooter_task = pros::c::task_create([](void* _) { sShooter->shooterRunner(_); }, nullptr, TASK_PRIORITY_DEFAULT,
	                                 TASK_STACK_DEPTH_DEFAULT, "Shooter Task");
}

void Shooter::shooterRunner(void* params){
    while (true){
        static int counter;
        counter ++;

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
            scataMotor.move_voltage(0);
            isReady = true;
        }


        if (sController->getDigital(Controller::l1) && isReady){
            fireScata();
        }

        sController->registerCallback([this]() { toggleBoost(!isBoosted);}, [this]() {}, Controller::master,
	                              Controller::l2, Controller::rising);


        //detects when to fire. if the difference in current and last position changed 
        //significantly, it means the scata fired, so we turn isShooting off and depower
        //the scata
        if (isShooting && std::abs(scataRotation.get_position() - prevPos) < 500){
            scataMotor.move_voltage(12000);
            if (isBoosted){
                leftBoostPiston.set_value(true);
                rightBoostPiston.set_value(true);
                printf("pushing\n");
            }
            // else {
            //     leftBoostPiston.set_value(false);
            //     rightBoostPiston.set_value(false);
            //     printf("pulling\n");
            // }
        }
        else {
            isShooting = false;
            leftBoostPiston.set_value(false);
            rightBoostPiston.set_value(false);
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
    isBoosted = toggle;
}

void Shooter::fireExpansion(){
    leftExpansionPiston.set_value(true);
    rightExpansionPiston.set_value(true);
}