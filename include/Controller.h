#pragma once

#include "main.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include <functional>
#include <map>
#include <utility>

#define sController Controller::getInstance()

class Controller {
public:
	enum ID { master = 0, partner };
	enum Analog { left_x = 0, left_y, right_x, right_y };
	enum Digital { l1 = 6, l2, r1, r2, up, down, left, right, x, b, y, a };
	enum ButtonMode { hold, rising, falling };

private:
	using DigitalFunc = std::function<void(void)>;

	static Controller* INSTANCE;
	pros::task_t task;
	pros::Mutex mutex;

	// stores callbacks - controller, which button, and on which action to call callback
	std::map<std::tuple<ID, Digital, ButtonMode>, std::pair<DigitalFunc, DigitalFunc>> buttonBinds;

	// stores current button state + prevous button state
	// std::pair<bool, bool> - first stores the current state, second stores the previous state
	std::map<std::pair<ID, Digital>, std::pair<bool, bool>> buttonStates;

	Controller();

	void backend();

	void callCallback(const std::pair<ID, Digital>& key, ButtonMode mode, bool callDefault = false);

public:
	static Controller* getInstance() {
		if (!INSTANCE) { INSTANCE = new Controller(); }

		return INSTANCE;
	}

	void initialize();

	void registerCallback(const DigitalFunc& callback, const DigitalFunc& defaultFunc, ID id, Digital digital,
	                      ButtonMode mode);
	bool removeCallback(ID id, Digital digital, ButtonMode mode);

	[[nodiscard]] int32_t getDigital(Digital digital, ID id = master) const;
	[[nodiscard]] int32_t getAnalog(Analog analog, ID id = master) const;
};