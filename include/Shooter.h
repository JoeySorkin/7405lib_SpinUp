#pragma once 

#include "main.h"
#include "pros/adi.hpp"
#include "pros/rotation.hpp"
#include "Constants.h"
#include "pros/rtos.h"
#include <atomic>
#include "Intake.h"

#define sShooter Shooter::getInstance()


class Shooter{
    public: 
        enum class state{READY = 1, FIRING, RECHARGELOW, RECHARGEHIGH};

    private:
        pros::Motor scataMotor;
        pros::Rotation scataRotation;
        int prevPos;
        std::atomic<int> emergencyOverride;      
        state scataState;

        pros::ADIDigitalOut toggleBoostPiston;

        pros::ADIDigitalOut leftExpansionPiston, rightExpansionPiston;

        pros::task_t shooter_task;
        static Shooter* INSTANCE;
        void shooterRunner(void* params);
        void automaticScata();
        void manualScata();
        bool expansionMode;
        bool boost;
        int expansionModeCount = 0;


        Shooter();
        Shooter(const Shooter&) = delete;
        Shooter& operator=(const Shooter&) = delete;

    public: 
        static Shooter* getInstance(){
            if (!INSTANCE) { INSTANCE = new Shooter(); }
		    return INSTANCE;
        }
        void initialize();
        state getState();
        void fireScata();
        void switchBandBoost();
};