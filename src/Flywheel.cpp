//
// Created by Joey Sorkin on 11/9/22.
//

#include "Flywheel.h"
#include "Controller.h"
#include "Intake.h"
#include "Robot.h"
#include "lib/utils/CircularBuffer.h"
#include "lib/utils/Math.h"
#include "lib/utils/Timeout.h"

Flywheel* Flywheel::INSTANCE = nullptr;

void Flywheel::initialize() {

	sController->registerCallback(
	        [this]() {
		        if (util::fpEquality(this->getTargetVelocity(), controllerVel)) {
			        this->setVelocity(0);
		        } else {
			        this->setVelocity(controllerVel);
		        }
	        },
	        []() {}, Controller::master, Controller::l1, Controller::rising);

	sController->registerCallback([this]() { this->triple_shoot_controller(200, 200); }, []() {}, Controller::master,
	                              Controller::r2, Controller::rising);

	sController->registerCallback([this]() { this->controllerVel += 20; }, []() {}, Controller::master, Controller::up,
	                              Controller::rising);

	sController->registerCallback([this]() { this->controllerVel -= 20; }, []() {}, Controller::master,
	                              Controller::down, Controller::rising);

	kF_lut.add_data(3863.160000, 3.106265);
	kF_lut.add_data(3688.488000, 3.117809);
	kF_lut.add_data(3544.328000, 3.103550);
	kF_lut.add_data(3387.064000, 3.100030);
	kF_lut.add_data(1040.424000, 3.364013);
	kF_lut.add_data(1213.080000, 3.297392);
	kF_lut.add_data(1379.536000, 3.261966);
	kF_lut.add_data(2406.280000, 3.116844);
	kF_lut.add_data(1555.696000, 3.213996);
	kF_lut.add_data(1918.776000, 3.126993);
	kF_lut.add_data(2093.408000, 3.104985);
	kF_lut.add_data(2251.688000, 3.108779);
	kF_lut.add_data(3207.624000, 3.117572);
	kF_lut.add_data(2923.032000, 3.078995);
	kF_lut.add_data(855.440000, 3.506967);
	kF_lut.add_data(1695.568000, 3.243751);
	kF_lut.add_data(2571.768000, 3.110700);
	kF_lut.add_data(3091.264000, 3.073177);
	kF_lut.add_data(2726.176000, 3.117920);

	kF_lut.add_data(0, 0);

	kP_lut.add_data(0, 5);
	kI_lut.add_data(0, 0.4);
	kD_lut.add_data(0, 0);

	kP_lut.add_data(1850, 3);
	kI_lut.add_data(1850, 0.5);
	kD_lut.add_data(1850, 0);
	//
	//  kP_lut.add_data(2320, 7);
	//  kI_lut.add_data(2320, 0.3);
	//  kD_lut.add_data(2320, 0.001);

	kP_lut.add_data(2350, 4);
	kI_lut.add_data(2350, 2);
	kD_lut.add_data(2350, 0.001);
	//
	kP_lut.add_data(2518, 6);
	kI_lut.add_data(2518, 0.5);
	kD_lut.add_data(2518, 0.001);
	//
	kP_lut.add_data(2650, 9);
	kI_lut.add_data(2650, 5);
	kD_lut.add_data(2650, 0.001);

	pidf.setIntegralBound(200);

	pidf.setGains({0, 0, 0, 0});
	//  pidf.invertD();

	task = pros::c::task_create([](void* _) { sFlywheel->runner(_); }, nullptr, TASK_PRIORITY_DEFAULT,
	                            TASK_STACK_DEPTH_DEFAULT, "Flywheel Task");

	revamp_lut.add_data(2100, 400);
}

void Flywheel::setVoltage(int32_t voltage) {
	//  motorA.move_voltage(voltage);
	motor.move_voltage(voltage);
}

void Flywheel::runner(void* ignored) {
	double last_pow;
	uint32_t time = pros::millis();
	libM::CircularBuffer<double, 5> cbuf{};
	int i = 0;
	while (true) {
		//    countDisksInSilo();
		i += 1;
		//    double mAV =
		//        motorA.get_actual_velocity(); // eric says i dont need you anymore
		double mBV = motor.get_actual_velocity();
		// pros::lcd::print(3, "Temp Front: %.1f", motor.get_temperature());
		// if (mBV == PROS_ERR_F) {// add in mA
		// 	printf("FUCKFUCKFUCKFUCKFUCKFUCKFUCK\n");
		// }
		//    _actual_vel = (mAV + mBV) / 2.0 * 5.0;
		_actual_vel = mBV * 6.0;
		cbuf.push_back(_actual_vel);
		double sum = 0;
		for (int i = 0; i < cbuf.size(); i++) { sum += cbuf[i]; }
		_filtered_vel = cbuf.avg(3);
		double target_vel = getTargetVelocity();
		double lmod = 500;

		pros::lcd::print(5, "Target Vel: %f", target_vel);
		pros::lcd::print(6, "Vel: %f", _filtered_vel);

		double error = target_vel - _filtered_vel;
		double power = util::clamp(0.0, 12000.0, pidf.calculate(error));

		//    printf("VEL:%.2f\n", _actual_vel);
		if (!override) {
			if (target_vel >= 0) {
				if (true) {
					// printf("FilteredVel:%.2f,TargetVel:%.2f,Stable%d,Power:%.2f,Error:%.2f,"
					//        "Maxtemp:%.2f \n",
					//        _filtered_vel, target_vel, fabs(error) < 50 ? 1 : 0, (power), error,
					//        fmax(0.0, motor.get_temperature()));
				}
			}
			setVoltage(power);
		} else {
			// printf("OVERRIDE\n");
		}
		if (triple_shoot_flag) {

			//      triple_shoot(revamp_a, revamp_b); OLD version

			// times the triple shot

			/*
			 *  a triple shot that DEPENDS on "waitUntilSettled"...
			 *  no bullshit with sending extra boosts of voltage
			 */
			triple_shoot(revamp_a, revamp_b);

			triple_shoot_flag = false;

		} else if (manual_shoot_flag) {
			sIntake->moveVel(-200);
			pros::delay(200);// TODO: make a constant
			sIntake->moveVel(0);
			//      override = true;
			//      setVoltage(12000);
			//      pros::delay(300);
			//      override = false;

			manual_shoot_flag = false;
		}
		last_pow = power;
		pros::c::task_delay_until(&time, 20);
		//    writeToCSV(pros::millis(), _last_vel, _last_filtered_vel, _actual_vel,
		//               _filtered_vel, val);
	}
}

bool Flywheel::getShootFlag() {
	return manual_shoot_flag.load();
}

void Flywheel::boost_auton(double revamp) {
	override = true;
	setVoltage(12000);
	pros::delay(revamp);
	override = false;
}
void Flywheel::shoot_auton(double revamp) {
	shoot();
	override = true;
	setVoltage(12000);
	pros::delay(revamp);
	override = false;
}

void Flywheel::setVelocity(double set_vel) {
	if (_target_speed.load() != set_vel) {
		_target_speed = set_vel;
		pidf.setSetpoint(set_vel);
		double new_kP = kP_lut.get_val(set_vel);
		double new_kI = kI_lut.get_val(set_vel);
		double new_kD = kD_lut.get_val(set_vel);
		double new_kF = kF_lut.get_val(set_vel + kF_shift);
		pidf.setConstants({
		        new_kP,
		        new_kI,
		        new_kD,
		        new_kF,
		});
		//    printf("%.2f: Constants P %.2f \t I %.2f \t D %.2f \t F %.2f\n",
		//    set_vel,
		//           pidf.getGains().P, pidf.getGains().I, pidf.getGains().D,
		//           pidf.getGains().F);
		//    printf("%.2f: Constants P %.2f \t I %.2f \t D %.2f \t F %.2f\n",
		//    set_vel,
		//           new_kP, new_kI, new_kD, new_kF);
	}
}

double Flywheel::getTargetVelocity() {
	return _target_speed.load();
}

double Flywheel::getActualVelocity() {
	return _actual_vel;
}

double Flywheel::getFilteredVelocity() {
	return _filtered_vel;
}

bool Flywheel::isStable(double thresh) {
	if (fabs((getTargetVelocity() - getFilteredVelocity())) < thresh) {//
		_stable_buff += 1;
	} else {
		_stable_buff = 0;
	}
	return _stable_buff > 2;
}

// void Flywheel::writeToCSV(double time, double set_vel, double actual_vel,
//                            double filtered_vel, double power, double gain) {
//     fprintf(logFile, "%f,%f,%f,%f,%f\n", time, set_vel, actual_vel, power,
//     gain);
// }

//  void Flywheel::close() {
////    fclose(logFile);
////  }

bool Flywheel::waitUntilSettled(uint32_t timeout, double thresh) {
	Timeout timer{timeout};
	while (!isStable(thresh)) {

		if (timer.timedOut()) { return true; }
		pros::c::delay(20);
	}
	return false;
}

void Flywheel::shoot() {
	sIntake->moveVel(-200);
	pros::delay(200);
	sIntake->brake();
}

uint32_t Flywheel::estimateRevampTime() {
	double vel = getActualVelocity();
	return round(revamp_lut.get_val(vel));// bruh
}

void Flywheel::triple_shoot(uint32_t revamp_time_a, uint32_t revamp_time_b) {
	override = true;
	int disks = disks_in_silo.load();

	setVoltage(12000);

	sIntake->moveVoltage(-12000);
	pros::delay(600);// TODO: make a constant
	sIntake->moveVoltage(0);


	//  pros::c::delay(revamp_time_a);
	//  if (disks > 1) {
	//    sIntake->index(200);
	//    pros::c::delay(revamp_time_b);
	//    if (disks > 2) {
	//      sIntake->index(200);
	//    }
	//  }
	override = false;
}

bool Flywheel::waitUntilVelocity(double target, double acc, uint32_t timeout) {
	Timeout timer{timeout};
	while (fabs(getFilteredVelocity() - target) < acc) {
		if (timer.timedOut()) { return false; }
		pros::c::delay(20);
	}
	return true;
}

void Flywheel::reverse(bool state) {
	if (state) {
		override = true;
		setVoltage(-6000);
	} else if (override) {
		override = false;
	}
}

void Flywheel::triple_shoot_controller(uint32_t revamp_time_a, uint32_t revamp_time_b) {
	revamp_a = revamp_time_a;
	revamp_b = revamp_time_b;
	triple_shoot_flag = true;
}

void Flywheel::shoot_controller() {
	manual_shoot_flag = true;
}

void Flywheel::setVoltageMan(double volt) {
	override = true;
	setVoltage(volt);
}

void Flywheel::automatic() {
	override = false;
}

void Flywheel::countDisksInSilo() {
	//  pros::lcd::print(2, "%i disks", disks_in_silo.load());
	//  if (fabs(pros::c::distance_get(ports::silo_distance) - 52.0) < 5) {
	//    disks_in_silo = 3;
	//  } else if (fabs(pros::c::distance_get(ports::silo_distance) - 75.0) < 5) {
	//    disks_in_silo = 2;
	//  } else if (fabs(pros::c::distance_get(ports::silo_distance) - 99.0) < 5) {
	//    disks_in_silo = 1;
	//  }
	//  pros::lcd::print(2, "%i disks", disks_in_silo.load());
	disks_in_silo = 3;
	// bruh
}

void Flywheel::triple_shoot_legit() {
	for (int i = disks_in_silo; i > 0 && triple_shoot_flag; i--) {
		_stable_buff = 0;
		waitUntilSettled();
		pros::c::delay(200);
		shoot();
	}
}

int Flywheel::getDisksInSilo() {
	return disks_in_silo.load();
}