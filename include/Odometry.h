#pragma once

#include "../main.h"
#include "geometry/kinState.h"

#define sOdom Odometry::getInstance()

class Odometry
{
private:
    pros::Rotation leftWheel, rightWheel, backWheel;
    pros::task_t odom_task;
    pros::Mutex odomMutex;

    double prev_l, prev_r, prev_b;

    kinState curr_state;

    static Odometry *INSTANCE;

    void updatePosition(void *params);

public:
    Odometry();
    kinState getCurrentState();
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