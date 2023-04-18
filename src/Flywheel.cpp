//
// Created by Joey Sorkin on 11/9/22.
//

#ifndef INC_7405MSPINUP_SHOOTER_H
#define INC_7405MSPINUP_SHOOTER_H
#include "../Constants.h"
#include "../main.h"
#include "lib/controllers/PIDF.h"
#include "lib/utils/Piston.h"
#include <atomic>

#define Flywheel Flywheel::getInstance()

class Flywheel {
private:
	static Flywheel *INSTANCE;
	pros::task_t task;
	[[noreturn]] void runner(void *ignored);
	std::atomic<bool> override = false;
	double _last_vel, _last_filtered_vel;
	double _actual_vel, _filtered_vel;
	double kF_shift = 0;
	std::atomic<double> _target_speed;
	std::atomic<bool> _coasting;


	std::atomic<double> revamp_a, revamp_b;

	std::atomic<bool> manual_shoot_flag;

	std::atomic<int> disks_in_silo = 3;

	int _stable_buff;

	libM::PIDF pidf;

	void setVoltage(int32_t voltage);
	void countDisksInSilo();

	//  pros::Motor motorA{ports::flywheel_a, pros::E_MOTOR_GEAR_600, true};
	pros::Motor motorB{ports::flywheel_b, pros::E_MOTOR_GEAR_600, false};

public:
	std::atomic<bool> triple_shoot_flag;
	static Flywheel *getInstance() {
		if (!INSTANCE) {
			INSTANCE = new Flywheel();
		}
		return INSTANCE;
	}
	libM::LUT kF_lut, kP_lut, kI_lut, kD_lut;
	libM::LUT revamp_lut;
	void setVelocity(double set_vel);
	void setVoltageMan(double volt);
	void initialize();
	int getDisksInSilo();
	/**
   *
   * @return true if timed out :)
	 */
	bool waitUntilSettled(uint32_t timeout = TIMEOUT_MAX, double thresh = 20);

	bool waitUntilVelocity(double target, double acc,
	                       uint32_t timeout = TIMEOUT_MAX);
	double getFilteredVelocity();
	double getActualVelocity();

	double getTargetVelocity();
	void shoot();
	void shoot_controller();
	void triple_shoot(uint32_t revamp_time_a = 400, uint32_t revamp_time_b = 400);
	void triple_shoot_controller(uint32_t revamp_time_a = 400,
	                             uint32_t revamp_time_b = 400);
	uint32_t estimateRevampTime();
	void no_time_triple_shot();
	void reverse(bool state);
	void automatic();

	void triple_shoot_legit();
	bool isStable(double thresh = 20);
	bool getShootFlag();
	void shoot_auton(double revamp = 300);
	void boost_auton(double revamp = 300);
};

#endif // INC_7405MSPINUP_SHOOTER_H
