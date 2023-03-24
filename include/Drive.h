#include "Odometry.h"
#include "lib/physics/Motion.h"
#include "lib/utils/Timeout.h"
#include "main.h"
#include <atomic>
#define sDrive Drive::getInstance()
class Drive
{
private:
    // General Stuff
    static Drive *INSTANCE;
    pros::task_t task;
    void runner(void *ignored);

    // Motors
    pros::Motor frontLeft{ports::frontLeftMotor}, frontRight{ports::frontRightMotor}, middleLeft{ports::middleLeftMotor}, middleRight{ports::middleRightMotor}, backLeft{ports::backLeftMotor}, backRight{ports::backRightMotor};

    // Motor Control
    void setVoltageLeft(int16_t voltage);
    void setVoltageRight(int16_t voltage);

    void setBrakeMode(pros::motor_brake_mode_e_t breakmode);
    pros::motor_brake_mode_e_t getBrakeMode();

    // Thread Saftey
    Motion* getCurrentMotion(uint32_t timeout = TIMEOUT_MAX);
    void giveCurrentMotion();
    std::atomic<bool> isSettled = false;
    std::atomic<bool> isTimedOut = false;

public:
    static Drive *getInstance()
    {
        if (!INSTANCE)
        {
            INSTANCE = new Drive();
        }
        return INSTANCE;
    }
    void initialize();

    // Control
    /**
     * @return returns true if settled, false if timed out
     */
    bool waitUntilSettled(uint32_t timeout = TIMEOUT_MAX);
};