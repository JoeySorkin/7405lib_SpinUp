#pragma once 

#include "main.h"
#include "pros/adi.hpp"
#include "pros/rotation.hpp"
#include "Constants.h"
#include "pros/rtos.h"
#include <atomic>

#define sShooter Shooter::getInstance()


class Shooter{
    private:
        pros::Motor scataMotor;
        pros::Rotation scataRotation;
        int prevPos;
        std::atomic<bool> isShooting;
        std::atomic<bool> isReady;

        pros::ADIDigitalOut leftBoostPiston, rightBoostPiston, toggleBoostPiston;

        pros::ADIDigitalOut leftExpansionPiston, rightExpansionPiston;

        pros::task_t shooter_task;
        static Shooter* INSTANCE;
        void shooterRunner(void* params);

        Shooter();
        Shooter(const Shooter&) = delete;
        Shooter& operator=(const Shooter&) = delete;

    public: 
        static Shooter* getInstance(){
            if (!INSTANCE) { INSTANCE = new Shooter(); }
		    return INSTANCE;
        }
    
        void initialize();
        void toggleBoost(bool toggle);
        void fireScata();
        void fireExpansion();
};