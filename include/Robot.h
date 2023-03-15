
#include "main.h"

#define sRobot Robot::getInstance()
class Robot
{
public:
    enum OpMode
    {
        DRIVER,
        AUTONOMOUS
    };

private:
    // Singleton Stuff
    static Robot *INSTANCE;
    Robot() = default;
    // OpMode
    std::atomic<OpMode> opmode;

public:
    void initialize();

    //  OpMode
    OpMode getOpMode();
    void setOpMode(OpMode op_mode);

    // Singleton Stuff
    static Robot *getInstance()
    {
        if (!INSTANCE)
        {
            INSTANCE = new Robot();
        }
        return INSTANCE;
    }
};