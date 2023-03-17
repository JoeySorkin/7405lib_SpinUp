#include "lib/geometry/Pose.h"
using namespace libM;

double Pose::getX() const { return _x; }
double Pose::getY() const { return _y; }
double Pose::getTheta() const { return _theta; }

double Pose::getShortTheta() const { 

}

double Pose::distanceTo(Pose other) const {
    return sqrt(pow(_x - other._x, 2.0) + pow(_y - other._y, 2.0));
}

double Pose::headingToPoint(Pose other) const {
    double dx = other._x - _x;
    double dy = other._y - _y;
    return (M_PI / 2.0) - atan2(dy, dx);
} 

Pose Pose::transformBy(double delta_x, double delta_y, double delta_theta) const {
    return {_x + delta_x, _y + delta_y, _theta + delta_theta};
}

Pose Pose::transformBy(Pose other) const {
    return transformBy(other._x, other._y, other._theta);
}

Pose Pose::getTransformation(Pose &A, Pose &B){
    { B._x - A._x, B._y - A._y, B._theta - A._theta; } // no return function??
}

Pose Pose::scale(double scalar) const {
    return {_x * scalar, _y * scalar, _theta};
}