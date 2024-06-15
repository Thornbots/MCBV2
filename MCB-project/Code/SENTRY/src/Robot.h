#pragma once

#include <cmath>

#include "tap/algorithms/smooth_pid.hpp"

#include "DrivetrainSubsystem.h"
#include "GimbalSubsystem.h"
#include "JetsonCommunication.h"
#include "ShooterSubsystem.h"
#include "drivers_singleton.hpp"
#include "AutoAim.h"

namespace ThornBots {
    using namespace tap::communication::serial;
    // Don't ask me why. Timers only work when global. #Certified taproot Moment
    static tap::arch::PeriodicMilliTimer motorsTimer(2);
    static tap::arch::PeriodicMilliTimer IMUTimer(2);
    static tap::arch::PeriodicMilliTimer updateInputTimer(2);
    static tap::arch::PeriodicMilliTimer cvTimer(33);

    enum Program { MANUAL, MATCH, CV_TEST };

    class Robot {
    public:                                         // Public Variables
        static constexpr double MAX_SPEED = 11000;  // controller //7000
        static constexpr double SLOW_SPEED = 1000;
        static constexpr double MED_SPEED = 7000;
        static constexpr double FAST_SPEED = 8000;
        static constexpr double FAST_BEYBLADE_FACTOR = 0.9 * 10000 / MAX_SPEED;  // 0.7
        static constexpr double SLOW_BEYBLADE_FACTOR = 0.6 * 10000 / MAX_SPEED;  // 0.35
        static constexpr double TURNING_CONSTANT = 0.5;
        static constexpr double dt = 0.002;
        constexpr static double YAW_TURNING_PROPORTIONAL = -0.02;
        // static constexpr double
    private:  // Private Variables
        tap::Drivers* drivers;
        ThornBots::DrivetrainSubsystem* drivetrainSubsystem;
        ThornBots::GimbalSubsystem* gimbalSubsystem;
        ThornBots::ShooterSubsystem* shooterSubsystem;
        ThornBots::JetsonCommunication* jetsonCommunication;
        double left_stick_horz, left_stick_vert, right_stick_horz, right_stick_vert = 0;
        double leftStickAngle, rightStickAngle, leftStickMagnitude, rightStickMagnitude = 0;
        double wheelValue = 0;
        double driveTrainRPM, yawRPM, yawAngleRelativeWorld = 0.0, imuOffset;
        Remote::SwitchState leftSwitchState, rightSwitchState = Remote::SwitchState::MID;
        // bool useKeyboardMouse = false;
        double yawEncoderCache = 0;
        double desiredYawAngleWorld, desiredYawAngleWorld2, driveTrainEncoder = 0.0;
        double stickAccumulator = 0, targetYawAngleWorld = PI, targetPitchAngleWorld = 0,
               targetDTVelocityWorld = 0;  // changed targetYawAngleWorld from 0 to PI
        bool robotDisabled = false, matchHasStarted;

        Program currentProgram = MANUAL;

        ThornBots::AutoAim autoAim = AutoAim();

        int send_timer = 0;

    public:  // Public Methods
        Robot(tap::Drivers* driver, ThornBots::DrivetrainSubsystem* drivetrainSubsystem, ThornBots::GimbalSubsystem* gimbalSubsystem,
              ThornBots::ShooterSubsystem* shooterSubsystem, ThornBots::JetsonCommunication* JetsonCommunication);

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

        // bool toggleKeyboardAndMouse();

    private:  // Private Methods
        void updateAllInputVariables();

        /*
         * Returns the angle (in radians) x and y form with 0 being straight ahead. atan2(x/y).
         * If x and y are both 0, this is typically undefined, so we assume it is 0.
         * Positive 90 degrees is CCW 90 degrees from 0. Negative 90 degrees is CW 90 degrees from 0
         * 90 degrees is joystick left, -90 is joystick right, and down is +-180 degrees
         */
        inline double getAngle(double x, double y) {
            // error handling to prevent runtime errors in atan2
            if (x == 0 && y == 0) return (double)0.0;

            return atan2(y, x);  // Return (double) [pi, pi] which we want. Doing x/y to rotate the unit
                                 // circle 90 degrees CCW (make 0 straight ahead)
        }

        inline double getMagnitude(double x, double y) { return sqrt(pow(x, 2) + pow(y, 2)); }

        void updateWithCV(bool patrol, bool shoot);
        void updateWithSpin();
        void updateWithController();

        // void updateWithMouseKeyboard();
    };
}  // namespace ThornBots