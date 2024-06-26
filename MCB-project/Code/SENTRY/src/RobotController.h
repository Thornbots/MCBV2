#pragma once

#include "tap/algorithms/smooth_pid.hpp"
#include <cmath>
#include "drivers_singleton.hpp"
#include "DriveTrainController.h"
#include "TurretController.h"
#include "ShooterController.h"
#include "JetsonCommunication.h"

namespace ThornBots {
    //Don't ask me why. Timers only work when global. #Certified taproot Moment
    static tap::arch::PeriodicMilliTimer motorsTimer(2);
    static tap::arch::PeriodicMilliTimer IMUTimer(2);
    static tap::arch::PeriodicMilliTimer updateInputTimer(2);

    class RobotController {
        public: //Public Variables
            static constexpr double PI = 3.14159;
            static constexpr double MAX_SPEED = 11000; //controller //7000
            static constexpr double SLOW_SPEED = 1000;
            static constexpr double MED_SPEED = 7000;
            static constexpr double FAST_SPEED = 8000;
            static constexpr double FAST_BEYBLADE_FACTOR = 0.9 * 10000 / MAX_SPEED; //0.7
            static constexpr double SLOW_BEYBLADE_FACTOR = 0.6 * 10000 / MAX_SPEED; //0.35
            static constexpr double TURNING_CONSTANT = 0.5;
            static constexpr double dt = 0.002;
            constexpr static double YAW_TURNING_PROPORTIONAL = -0.02;
            // static constexpr double 
        private: //Private Variables
            tap::Drivers *drivers;
            ThornBots::DriveTrainController *driveTrainController;
            ThornBots::TurretController *turretController;
            ThornBots::ShooterController *shooterController;
            ThornBots::JetsonCommunication jetsonCommunication;
            double left_stick_horz, left_stick_vert, right_stick_horz, right_stick_vert = 0;
            double leftStickAngle, rightStickAngle, leftStickMagnitude, rightStickMagnitude = 0;
            double wheelValue = 0;
            double driveTrainRPM, yawRPM, yawAngleRelativeWorld = 0.0;
            tap::communication::serial::Remote::SwitchState leftSwitchState, rightSwitchState = tap::communication::serial::Remote::SwitchState::MID;
            bool useKeyboardMouse = false;
            double yawEncoderCache = 0;
            double desiredYawAngleWorld, desiredYawAngleWorld2, driveTrainEncoder = 0.0;
            double stickAccumulator = 0, targetYawAngleWorld = 0, targetDTVelocityWorld = 0;
            bool robotDisabled = false;
            
        public: //Public Methods
            RobotController(tap::Drivers* driver,
                            ThornBots::DriveTrainController* driveTrainController,
                            ThornBots::TurretController* turretController,
                            ThornBots::ShooterController* shooterController);
            RobotController()=default;
            
            void initialize();

            void update();

            void stopRobot();
            void disableRobot();
            void enableRobot();

            bool toggleKeyboardAndMouse();

        private: //Private Methods
            void updateAllInputVariables();

            /*
            * Returns the angle (in radians) x and y form with 0 being straight ahead. atan2(x/y).
            * If x and y are both 0, this is typically undefined, so we assume it is 0.
            * Positive 90 degrees is CCW 90 degrees from 0. Negative 90 degrees is CW 90 degrees from 0
            * 90 degrees is joystick left, -90 is joystick right, and down is +-180 degrees
            */
            double getAngle(double x, double y);

            double getMagnitude(double x, double y);

            void updateWithController();
            void updateWithMouseKeyboard();

    };
}