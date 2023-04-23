#include "Intake.h"
#include "Controller.h"
#include "Shooter.h"

Intake* Intake::INSTANCE = nullptr;

Intake::Intake() : motors(ports::intake) {
	motors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
	motors.set_gearing(pros::E_MOTOR_GEAR_200);
}

//we need to add an if/else statement to not allow intaking when shooter is not ready. 
//use the "readyToFire" function inside of shooter to check
void Intake::initialize() {
	sController->registerCallback([this]() {if (sShooter->getState() == Shooter::state::READY){moveVoltage(12000);}; }, [this]() { moveVoltage(0); }, Controller::master,
	                              Controller::r1, Controller::hold);

	sController->registerCallback([this]() { moveVoltage(-12000); },
	                              [this]() {
		                              // this needs to be done because otherwise the r1's callback's moveVoltage will be
		                              // overridden and it is in this callback because r2's callback will always be
		                              // called after r1
		                              if (motors.at(0).get_power() < 0) { moveVoltage(0); }
	                              },
	                              Controller::master, Controller::r2, Controller::hold);
}

void Intake::moveVoltage(int mv) {
	motors.move_voltage(mv);
}

void Intake::moveVel(int vel) {
	motors.move_velocity(vel);
}

void Intake::brake() {
	motors.brake();
}