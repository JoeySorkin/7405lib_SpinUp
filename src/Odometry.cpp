#include "Odometry.h"
#include "Constants.h"
#include "lib/geometry/kinState.h"
#include "pros/rtos.h"
#include <cmath>
#include <math.h>

Odometry* Odometry::INSTANCE = nullptr;

Odometry::Odometry() : prev_b(0), prev_l(0), prev_r(0), leftWheel(ports::leftRotation), rightWheel(ports::rightRotation), 
backWheel(ports::backRotation) {backWheel.reset_position(); leftWheel.reset_position(); rightWheel.reset_position();}

void Odometry::initialize(){
    odom_task = pros::c::task_create([](void* _) { sOdom->updatePosition(_); },
    nullptr, TASK_PRIORITY_DEFAULT,
    TASK_STACK_DEPTH_DEFAULT, "Odometry Task");
}

void Odometry::updatePosition(void* params){
    while (true) {
        uint32_t time = pros::millis();

        double l_dist = ((leftWheel.get_position() - prev_l) / 36000.0) * M_PI * odometers::leftDeadwheelDiameter;
        double r_dist = ((rightWheel.get_position() - prev_r) / 36000.0) * M_PI * odometers::rightDeadwheelDiameter;
        double b_dist = ((backWheel.get_position() - prev_b) / 36000.0) * M_PI * odometers::backDeadwheelDiameter;

        prev_l = leftWheel.get_position();
        prev_r = rightWheel.get_position();
        prev_b = backWheel.get_position();

        double dh = (l_dist - r_dist) / (odometers::trackWidth);

        double perp_offset = b_dist + (odometers::backOffset * dh); //need to test this

        double distance = (dh == 0) ? (l_dist + r_dist) / 2.0: ((l_dist + r_dist) / dh) * sin (dh / 2.0); //also need to test this

        double dx = distance + perp_offset * cos(curr_state.position.getTheta() + (M_PI / 2)) + perp_offset * sin(curr_state.position.getTheta() + (M_PI / 2));
        double dy = distance * sin(curr_state.position.getTheta() + (M_PI / 2)) + perp_offset * cos(curr_state.position.getTheta() + (M_PI / 2));
        double dt = 20.0 / 1000.0;

        stateMutex.take(TIMEOUT_MAX);
        curr_state.setAcceleration((curr_state.velocity().x - (dx/dt))/dt, (curr_state.velocity().y - (dy/dt))/dt, (curr_state.velocity().theta - (dh/dt))/dt);
        curr_state.setVelocity(dx/dt, dy/dt, dh/dt);
        curr_state.position.transformBy(dx, dy, dh);
        stateMutex.give(); 

        pros::c::task_delay_until(&time, 20);
    }
}

kinState Odometry::getCurrentState(){

    stateMutex.take(TIMEOUT_MAX);
    kinState p = curr_state;
    stateMutex.give();
    return p;
}