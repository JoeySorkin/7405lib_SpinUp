#include "Constants.h"
#include "lib/controllers/PIDF.h"
#include "lib/utils/LUT.h"
#include "main.h"
#include "pros/distance.hpp"
#include "pros/rotation.hpp"
#include <atomic>

#define sFlywheel Flywheel::getInstance()

class Flywheel {
private:
	static Flywheel* INSTANCE;
	pros::task_t task;
	[[noreturn]] void runner(void* ignored);
	std::atomic<bool> override = false;
	double _last_vel, _last_filtered_vel;
	double _actual_vel, _filtered_vel, controllerVel = 1900;
	double kF_shift = 0;
	std::atomic<double> _target_speed;
	std::atomic<bool> _coasting;


	std::atomic<double> revamp_a, revamp_b;

	std::atomic<bool> manual_shoot_flag;

	std::atomic<int> disks_in_silo = 3;

	int _stable_buff;

	PIDF pidf;

	pros::Rotation indexerRotation{1};

	void setVoltage(int32_t voltage);
	void countDisksInSilo();

	//  pros::Motor motorA{ports::flywheel_a, pros::E_MOTOR_GEAR_600, true};
	pros::Motor motor{ports::flywheelMotor, pros::E_MOTOR_GEAR_600, false};

public:
	std::atomic<bool> triple_shoot_flag;
	static Flywheel* getInstance() {
		if (!INSTANCE) { INSTANCE = new Flywheel(); }
		return INSTANCE;
	}
	LUT kF_lut, kP_lut, kI_lut, kD_lut;
	LUT revamp_lut;
	void setVelocity(double set_vel);
	void setVoltageMan(double volt);
	void initialize();
	int getDisksInSilo();
	/**
	 *
	 * @return true if timed out :)
	 */
	bool waitUntilSettled(uint32_t timeout = TIMEOUT_MAX, double thresh = 20);

	bool waitUntilVelocity(double target, double acc, uint32_t timeout = TIMEOUT_MAX);
	double getFilteredVelocity();
	double getActualVelocity();

	double getTargetVelocity();
	void shoot();
	void shoot_controller();
	// void triple_shoot(uint32_t revamp_time_a = 400, uint32_t revamp_time_b = 400);
	void triple_shoot(int mv = 12000, int intake = -12000, int delay = 100);
	void triple_shoot_controller(uint32_t revamp_time_a = 400, uint32_t revamp_time_b = 400);
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