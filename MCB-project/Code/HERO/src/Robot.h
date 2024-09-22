#pragma once

#include <cmath>

#include "tap/algorithms/smooth_pid.hpp"

#include "DrivetrainSubsystem.h"
#include "GimbalSubsystem.h"
#include "ShooterSubsystem.h"
#include "drivers_singleton.hpp"

namespace ThornBots {
    using namespace tap::communication::serial;
    // Don't ask me why. Ti mers only work when global. #Certified taproot Moment
    static tap::arch::PeriodicMilliTimer motorsTimer(2);
    static tap::arch::PeriodicMilliTimer IMUTimer(2);
    static tap::arch::PeriodicMilliTimer updateInputTimer(2);

    class Robot {
    public:  // Public Variables
        // static constexpr double PI = 3.14159;
        static constexpr double MAX_SPEED = 11000;  // controller //7000
        static constexpr double SLOW_SPEED = 4000;
        static constexpr double MED_SPEED = 11000;
        static constexpr double FAST_SPEED = 11000;                              // will allow power limit to be 50% higher
        static constexpr double FAST_BEYBLADE_FACTOR = 0.8 * 10000 / MAX_SPEED;  // 0.7
        static constexpr double SLOW_BEYBLADE_FACTOR = 0.4 * 10000 / MAX_SPEED;  // 0.35
        static constexpr double TURNING_CONSTANT = 0.5;
        static constexpr double dt = 0.002;
        constexpr static double YAW_TURNING_PROPORTIONAL = -0.02;
        // static constexpr double
    private:  // Private Variables
        tap::Drivers* drivers;
        ThornBots::DrivetrainSubsystem* drivetrainSubsystem;
        ThornBots::GimbalSubsystem* gimbalSubsystem;
        ThornBots::ShooterSubsystem* shooterSubsystem;
        double left_stick_horz, left_stick_vert, right_stick_horz, right_stick_vert = 0;
        double leftStickAngle, rightStickAngle, leftStickMagnitude, rightStickMagnitude = 0;
        double wheelValue = 0;
        double driveTrainRPM, yawRPM, yawAngleRelativeWorld = 0.0, imuOffset;
        Remote::SwitchState leftSwitchState, rightSwitchState = Remote::SwitchState::MID;
        bool useKeyboardMouse = false;
        double yawEncoderCache = 0;
        double desiredYawAngleWorld, desiredYawAngleWorld2, driveTrainEncoder = 0.0;
        double stickAccumulator = 0, targetYawAngleWorld = 0, targetDTVelocityWorld = 0;  // changed targetYawAngleWorld from 0 to PI
        bool robotDisabled = false;

        bool pastKeyReleased[16];
        bool mouseLHasBeenReleased = true;

        double currentBeybladeFactor = 0;

    public:  // Public Methods
        Robot(tap::Drivers* driver, ThornBots::DrivetrainSubsystem* driveTrainController, ThornBots::GimbalSubsystem* turretController,
              ThornBots::ShooterSubsystem* shooterController);

        void initialize();

        void update();

        inline void stopRobot() {
            drivetrainSubsystem->stopMotors();
            gimbalSubsystem->stopMotors();
            shooterSubsystem->stopMotors();
            robotDisabled = true;
        }

        inline void disableRobot() {
            stopRobot();
            drivetrainSubsystem->disable();
            gimbalSubsystem->disable();
            shooterSubsystem->disable();
        }

        inline void enableRobot() {
            robotDisabled = false;
            drivetrainSubsystem->enable();
            gimbalSubsystem->enable();
            shooterSubsystem->enable();
        }

        bool toggleKeyboardAndMouse();

    private:  // Private Methods
        void updateAllInputVariables();

        inline double getAngle(double x, double y) {
            // error handling to prevent runtime errors in atan2
            if (x == 0 && y == 0) return (double)0.0;

            return atan2(y, x);  // Return (double) [pi, pi] which we want. Doing x/y to rotate the unit
                                 // circle 90 degrees CCW (make 0 straight ahead)
        }

        inline double getMagnitude(double x, double y) { return sqrt(pow(x, 2) + pow(y, 2)); }

        inline bool keyJustPressed(Remote::Key key) {
            int index = static_cast<int>(key);
            if (drivers->remote.keyPressed(key)) {
                if (pastKeyReleased[index]) {
                    pastKeyReleased[index] = false;
                    return true;
                }
            } else {
                pastKeyReleased[index] = true;
            }
            return false;
        }

        void updateWithController();
        void updateWithMouseKeyboard();
    };
}  // namespace ThornBots