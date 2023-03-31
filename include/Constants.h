 #pragma once

namespace ports
{
    const int frontLeftMotor = 3;
    const int frontRightMotor = 1;
    const int middleLeftMotor = 8;
    const int middleRightMotor = 5;
    const int backLeftMotor = 4;
    const int backRightMotor = 11;

    const int leftRotation = 12;
    const int rightRotation = 20;
    const int backRotation = 13;
}

namespace odometers
{
    const double trackWidth = 10.705;
    const double leftDeadwheelDiameter = 3.25 * (1.0 / 1.375); //1.375 is gear ratio
    const double rightDeadwheelDiameter = 3.25 * (1.0 / 1.375);
    const double backDeadwheelDiameter = 2.75;
    const double backOffset = 1;
}

namespace chassis{
    const double maxVelo = 1;
    const double accel = 1;
    const double deccel = 1;

    const double maxOmega = 1;
    const double maxAlphaUp = 1;
    const double maxAlphaDown = 1; 
}