#include "Controller.h"
#include "pros/misc.h"
#include "pros/rtos.h"

Controller* Controller::INSTANCE = nullptr;

Controller::Controller() : task(nullptr), buttonBinds(), buttonStates() {
	// Initalize the button states to be false
	for (size_t i = Digital::l1; i <= Digital::a; i++) {
		buttonStates[{ID::master, (Digital) i}] = {false, false};
		buttonStates[{ID::partner, (Digital) i}] = {false, false};
	}
}

void Controller::initialize() {
	task = pros::c::task_create([](void* ign) { sController->backend(); }, nullptr, TASK_PRIORITY_DEFAULT,
	                            TASK_STACK_DEPTH_DEFAULT, "Controller");
}

void Controller::backend() {
	while (true) {

		// poll for the new controller states
		// and call corresponding callbacks when the conditions for a callback is met
		for (const auto& [key, value] : buttonStates) {
			bool state = pros::c::controller_get_digital(static_cast<pros::controller_id_e_t>(key.first),
			                                             static_cast<pros::controller_digital_e_t>(key.second));

			// updates state - now we store new current state, and now the previous state
			buttonStates[key] = {state, value.first};

			if (state) {
				// button currently pressed
				if (value.second) {
					// HOLD - button was previously pressed
				} else {
					// RISING - button wasn't previuosly pressed
				}
			} else {
				// button not currently pressed
				if (value.second) {
					// FALLING - button was previously pressed
				} else {
					// button wasn't pressed in last 2 updates
					// call all the default callbacks associated with this button
				}
			}
		}

		pros::delay(10);
	}
}

void Controller::registerCallback(const DigitalFunc& callback, const DigitalFunc& defaultFunc, ID id, Digital digital,
                                  ButtonMode mode) {
	//
}

bool Controller::removeCallback(ID id, Digital digital, ButtonMode mode) {
	return false;
}

[[nodiscard]] int32_t Controller::getDigital(Digital digital, ID id) {
	return pros::c::controller_get_digital(static_cast<pros::controller_id_e_t>(id),
	                                       static_cast<pros::controller_digital_e_t>(digital));
}

[[nodiscard]] int32_t Controller::getAnalog(Analog analog, ID id) {
	return pros::c::controller_get_analog(static_cast<pros::controller_id_e_t>(id),
	                                      static_cast<pros::controller_analog_e_t>(analog));
}