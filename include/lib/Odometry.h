#pragma once

#include "Constants.h"
#include "main.h"

#define Odom Odometry::getInstance()

class Odometry:
{
private:
    pros::rotation leftWheel();
    pros::rotation rightWheel();
    pros::rotation backWheel();
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