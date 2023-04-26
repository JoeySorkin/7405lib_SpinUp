#include "Shooter.h"
#include "Constants.h"
#include "pros/rtos.hpp"
#include <cstddef>
#include <cstdio>
#include <valarray>
#include "Controller.h"

Shooter* Shooter::INSTANCE = nullptr;

Shooter::Shooter()
    : scataMotor(ports::scataMotor, true), scataRotation(ports::scataRotation, true), rightExpansionPiston({21, 'H'}, false),
    leftExpansionPiston({21, 'E'}, false), toggleBoostPiston({21, 'A'}, false), scataState(state::READY), expansionMode(false), 
    expansionModeCount(0), emergencyOverride(0), boost(true){}

void Shooter::initialize(){
    scataRotation.reset_position();
    shooter_task = pros::c::task_create([](void* _) { sShooter->shooterRunner(_); }, nullptr, TASK_PRIORITY_DEFAULT,
	                                 TASK_STACK_DEPTH_DEFAULT, "Shooter Task");

    // sController->registerCallback([this](){if(expansionModeCount > 1000){expansionMode = true; pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "---");} else{expansionModeCount += 900;}}, [this](){if(expansionModeCount >= 0){expansionModeCount -= 20;}}, Controller::master, Controller::x, Controller::rising);

    // sController->registerCallback([this](){if(expansionMode){leftExpansionPiston.set_value(true);}}, [](){}, Controller::master, Controller::left, Controller::rising);

    // sController->registerCallback([this](){if(expansionMode){rightExpansionPiston.set_value(true);}}, [](){}, Controller::master, Controller::right, Controller::rising);

    // sController->registerCallback([this](){if(expansionMode){leftExpansionPiston.set_value(true); rightExpansionPiston.set_value(true);}}, [](){}, Controller::master, Controller::up, Controller::rising);

    sController->registerCallback([this](){emergencyOverride += 500;}, [this](){if (emergencyOverride >= 0){emergencyOverride -= 20;}}, Controller::master, Controller::a, Controller::rising);
}

void Shooter::shooterRunner(void* params){
    while (true){
        if(emergencyOverride > 501){
            pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "-");
            manualScata();
        }
        else{
            automaticScata();
        }  
        prevPos = scataRotation.get_position();
        printf("expansioncount:%i\n", expansionModeCount);

        if(expansionModeCount >= 0) {expansionModeCount -= 20;}

        if(pros::c::controller_get_digital_new_press(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_X))
        {
            if(expansionModeCount > 1000)
            {
                expansionMode = true;
                pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "---");
            }
            else
            {
                expansionModeCount += 1000;
            }
        }

        if(pros::c::controller_get_digital_new_press(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_LEFT))
        {
            if(expansionMode)
            {
                leftExpansionPiston.set_value(true);
            }
        }

        if(pros::c::controller_get_digital_new_press(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_RIGHT))
        {
            if(expansionMode)
            {
                rightExpansionPiston.set_value(true);
            }
        }
        
        if(pros::c::controller_get_digital_new_press(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_UP))
        {
            if(expansionMode)
            {
                leftExpansionPiston.set_value(true);
                rightExpansionPiston.set_value(true);
            }
        }

        if(pros::c::controller_get_digital_new_press(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_Y))
        {
            boost = !boost;
            toggleBoostPiston.set_value(boost);
        }
        pros::delay(20);
    }
}

Shooter::state Shooter::getState(){
    return scataState;
}

void Shooter::fireScata(){
    scataState = state::FIRING;
}

void Shooter::manualScata(){
    if (sController->getDigital(Controller::l1)){
       scataMotor.move_voltage(12000);
    }
    else{
        scataMotor.move_voltage(0);
    }
}

void Shooter::automaticScata(){
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
}

void Shooter::switchBandBoost(){
    toggleBoostPiston.set_value(true);
}