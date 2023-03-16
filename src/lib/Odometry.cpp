#include "lib/Odometry.h"
#include "Constants.h"
#include <math.h>

Odometry* Odometry::INSTANCE = nullptr;
Odometry::Odometry() : prev_b(0), prev_l(0), prev_r(0), leftWheel(ports::leftRotation),
rightWheel(ports::rightRotation), backWheel(ports::backRotation)
{backWheel.reset_position(); leftWheel.reset_position(); rightWheel.reset_position();}

void Odometry::initialize(){
    odom_task = pros::c::task_create([](void* _) { sOdom->updatePosition(_); },
    nullptr, TASK_PRIORITY_DEFAULT,
    TASK_STACK_DEPTH_DEFAULT, "Odometry Task");
}

void Odometry::updatePosition(void* params){
    while (true) {
        uint32_t time = pros::millis();

        double l_dist = ((leftWheel.get_position() - prev_l) / 36000.0) * M_PI * chassis::leftDeadwheelDiameter;
        double r_dist = ((rightWheel.get_position() - prev_r) / 36000.0) * M_PI * chassis::rightDeadwheelDiameter;
        double b_dist = ((backWheel.get_position() - prev_b) / 36000.0) * M_PI * chassis::backDeadwheelDiameter;
;
        prev_l = leftWheel.get_position();
        prev_r = rightWheel.get_position();
        prev_b = backWheel.get_position();

        double theta = (l_dist - r_dist) / (chassis::trackWidth);
        double arclength;
        double distance;

        if (theta == 0){
            distance = (l_dist + r_dist) / 2.0;
        }
        else{
            arclength = (l_dist + r_dist) / (2.0 * theta);
            distance = ((l_dist + r_dist) / theta) * sin (theta / 2.0);
        }

        //temporary
        double dy = distance * sin(curr_state.position.getTheta() + (M_PI / 2));
        double dx = distance * cos(curr_state.position.getTheta() + (M_PI / 2));

        stateMutex.take(TIMEOUT_MAX);
        
        stateMutex.give(); 

        //curr_state.position.rotateBy(theta);
        //curr_state.position.translate(dx, dy);

        pros::c::task_delay_until(&time, 20);
    }
}