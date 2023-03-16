#pragma once

class Pose{
    private:
        double _x, _y, _theta;

    public:
        Pose() = default;
        Pose(double x, double y, double theta) : _x(x), _y(y), _theta(theta){};

        double getX() const;
        double getY() const;
        double getTheta() const;
        double getShortTheta() const;
};