#pragma once

class TrapezoidProfile {
private:
	struct State {
		double pos, vel, acc;
	};

	double dist, acc_max, dec_max, v_max;
	double t_acc, t_coast, t_dec;
	double d_acc, d_coast, d_dec;

	int direction;

	double acceleration(double t) const;
	double velocity(double t) const;
	double position(double t) const;

public:
	TrapezoidProfile(double distance, double max_acceleration, double max_deceleration, double max_velocity);

	// time in seconds
	State getState(double t) const;

	double getTargetDist() const;
	double getTotalTime() const;
};

/*#pragma once
#include "Pose.h"
#include <cmath>

class TrapezodialProfile {
public:
    struct Point {
        double pos;// dist from start of motion profile
        double vel;
        double acc;

        explicit Point(double dist, double maxAccel, double maxVel) : pos(dist), vel(vel), acc(acc) {}
    };

private:
    double dist;
    double maxVel;
    double maxAcc;

    double accelTime; // at what time since start of movement are we accelerating (s)
    double cruiseTime;// at what time since start of movement are we cruising (s)
    double decelTime; //  at what time since start of movement are we deceling (s)
    double accelDist;
    double cruiseDist;
    double decelDist;

    int direction;

public:
    // does the big boy number crunching
    TrapezodialProfile(double dist, double maxAcc, double maxDecel, double maxVel)
        : dist(dist), maxVel(maxVel), maxAcc(maxAcc), accelTime(0), cruiseTime(0), decelTime(0) {

        direction = dist >= 0 ? 1 : -1;

        // todo: do we need to do anything special for when initial vel or end vel are not 0? - probably? - probably
        // only occurs in stuff like pp? maybe just to calculate whether or not its trapezoidal or triangular?

        accelTime = maxVel / maxAcc;
        double halfwayDist = dist / 2.0;
        double accelDist = 0.5 * maxAcc * std::pow(accelTime, 2);

        // we can't accel to full speed so it now becomes triangular profile
        // determines if we can reach cruise speed or not
        if (accelDist > halfwayDist) { accelTime = std::sqrt(halfwayDist / (0.5 * maxAcc)); }

        // recompute accelDist based on whether or not we can reach max vel
        accelDist = 0.5 * maxAcc * std::pow(accelTime, 2);

        // recompute maxVel to conform with new accelDist
        maxVel = accelTime * maxAcc;// actually im p sure we dont need this but im braindead rn

        cruiseDist = dist - accelDist * 2;
        cruiseTime = cruiseDist / maxVel;

        decelTime = accelTime * 2 + cruiseTime;
        decelDist = accelDist;
        // printf("Accel Time: %f Accel Dist: %f Cruise Time: %f Cruise Dist: %f Decel Time: %f Decel Dist: %f\n",
        // accelTime,
        //        accelDist, cruiseTime, cruiseDist, decelTime, accelDist);
    }

    // time since start of motion profile/movement
    Point calculate(double t) const {
        Point result(start);

        if (t < accelTime) {
            result.acc = maxAcc;
            result.vel += maxAcc * t;
            result.dist += (0.5 * maxAcc * t) * t;
        } else if (t < cruiseTime) {
            result.acc = 0;
            result.vel = maxVel;
            result.dist += (start.vel + accelTime * maxAcc * 0.5) * accelTime + maxVel * (t - accelTime);
        } else if (t <= decelTime) {
            second_t timeLeft = decelTime - t;
            result.acc = -maxAcc;
            result.vel = end.vel + timeLeft * maxAcc;
            result.dist = end.dist - (end.vel + timeLeft * maxAcc * 0.5) * timeLeft;
        } else {
            result = end;
        }

        result.acc *= direction;
        result.dist *= direction;
        result.vel *= direction;

        return result;
    }

    double totalDist() const {
        return dist;
    }

    double totalTime() const {
        return decelTime;
    }

    int getDirection() const {
        return direction;
    }

    void setDirection(int direction) {
        this->direction = direction;
    }
};// https://www.ctrlaltftc.com/advanced/motion-profiling*/