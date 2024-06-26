#pragma once
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/board/board.hpp"
#include "drivers_singleton.hpp"
#include <cmath>
#include <iostream>
#include <string>
#include "tap/architecture/periodic_timer.hpp"
#include "tap/motor/dji_motor.hpp"
#include "drivers_singleton.hpp"
#include <cmath>

namespace ThornBots {
    class ModeledTurretController {
    public:
        ModeledTurretController();
        ~ModeledTurretController();
        double calculate(double currentPosition, double currentVelocity, double currentDrivetrainVelocity, double targetPosition, double deltaT);
        
    private:
        //START getters and setters
        double buildup = 0;
        double pastTargetVelocity = 0;
        double pastOutput = 0;

        // Physical constants
        const double C = 0.0169;               // kg-s/m^2
        const double J = 0.031;//289;               // kg-m^2
        const double UK = 0.07;                // N-m
        const double KB = 0.39;               // V-rad/s
        const double KT = 0.35;               // N-m/A
        const double RA = 1.03;               // ohm
        const double RATIO = 2;                // unitless
        const double VOLT_MAX = 24;                   //V
        const double VELO_MAX = VOLT_MAX/(KB*RATIO);    //rad/s
        // Position controller constants
        const double KP = 26;                // sec^-1

        // Feedforward constants
        const double A_SCALE = 0.9;//0.8            // unitless
        const double KSTATIC = (UK * RA) / (KT * RATIO);  // A
        const double KV = KB * RATIO;          // V-s/rad
        const double KA = J / (KT * RATIO);    // A-s^2/rad
        const double KVISC = C / (KT * RATIO); // A-s/rad

        // Gain scheduling
        const double KDT = -0.5;              // unitless
        const double KDT_REV = -0.5;           // unitless

        // Velocity feedback
        const double KPV = 0.05;                  // A-s/rad
        const double KIV = 1;                 // A/rad
        const double IV_MAX = 0.1;             // units TBD
        const double INT_THRESH = VOLT_MAX * 0.85;   // V
        const double TAKEBACK = 0.01;          // unitless


        double signum(double num) { 
            return (num > 0) ? 1 : ((num < 0) ? -1 : 0);
        }
 
    };
}
