#include "Controller.h"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <mutex>
#include <tuple>

Controller* Controller::INSTANCE = nullptr;

Controller::Controller() : task(nullptr), mutex(), buttonBinds(), buttonStates() {
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

		// TODO: just add check for when we're in auton or in opcontrol
		// and in auton, just don't update button states - or maybe just reset button states to being false and then
		// sleeping until something changes

		// poll for the new controller states
		// and call corresponding callbacks when the conditions for a callback is met
		if (!pros::competition::is_autonomous() && !pros::competition::is_disabled()) {
			pros::delay(20);
			continue;
		}

		mutex.take(TIMEOUT_MAX);
		for (const auto& [key, value] : buttonStates) {
			bool state = pros::c::controller_get_digital(static_cast<pros::controller_id_e_t>(key.first),
			                                             static_cast<pros::controller_digital_e_t>(key.second));

			// updates state - now we store new current state, and now the previous state
			buttonStates[key] = {state, value.first};

			if (state) {
				// button currently pressed
				if (value.second) {
					// HOLD - button was previously pressed
					callCallback(key, hold);
				} else {
					// RISING - button wasn't previuosly pressed
					callCallback(key, rising);
					callCallback(key, hold);
				}
			} else {
				// button not currently pressed
				if (value.second) {
					// FALLING - button was previously pressed
					callCallback(key, falling);
				} else {
					// button wasn't pressed in last 2 updates
					// call all the default callbacks associated with this button

					// due to the current way we store the callbacks for the three different callback possibilities
					// we have to do a lookup 3 different times
					// which is kinda slow
					// kinda crappily wrote
					callCallback(key, rising, true);
					callCallback(key, hold, true);
					callCallback(key, falling, true);
				}
			}
		}
		mutex.give();

		pros::delay(10);
	}
}

void Controller::callCallback(const std::pair<ID, Digital>& key, ButtonMode mode, bool callDefault) {
	auto [id, digital] = key;
	std::tuple<ID, Digital, ButtonMode> lookup = std::make_tuple(id, digital, mode);

	auto iter = buttonBinds.find(lookup);
	if (iter != buttonBinds.end()) {
		if (callDefault) {
			std::get<1>(iter->second)();
		} else {
			std::get<0>(iter->second)();
		}
	}
}

void Controller::registerCallback(const DigitalFunc& callback, const DigitalFunc& defaultFunc, ID id, Digital digital,
                                  ButtonMode mode) {
	std::lock_guard<pros::Mutex> lock(mutex);
	buttonBinds[std::make_tuple(id, digital, mode)] = {callback, defaultFunc};
}

bool Controller::removeCallback(ID id, Digital digital, ButtonMode mode) {
	std::lock_guard<pros::Mutex> lock(mutex);
	return buttonBinds.erase(std::make_tuple(id, digital, mode));
}

int32_t Controller::getDigital(Digital digital, ID id) const {
	return pros::c::controller_get_digital(static_cast<pros::controller_id_e_t>(id),
	                                       static_cast<pros::controller_digital_e_t>(digital));
}

int32_t Controller::getAnalog(Analog analog, ID id) const {
	return pros::c::controller_get_analog(static_cast<pros::controller_id_e_t>(id),
	                                      static_cast<pros::controller_analog_e_t>(analog));
}