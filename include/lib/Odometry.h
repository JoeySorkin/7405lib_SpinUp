#pragma once

#include "../Constants.h"
#include "../main.h"

#define sOdom Odometry::getInstance()

class Odometry:
{
private:
    pros::Rotation leftWheel();
    pros::Rotation rightWheel();
    pros::Rotation backWheel();
    static Odometry *INSTANCE;

public:
    void initialize();
    static Odometry *getInstance()
    {
        if (!INSTANCE)
        {
            INSTANCE = new Odometry();
        }
        return INSTANCE;
    }
};