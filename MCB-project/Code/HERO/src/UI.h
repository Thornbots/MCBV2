#pragma once
#include "tap/algorithms/smooth_pid.hpp"
#include <tap/communication/serial/ref_serial_transmitter.hpp>
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"

#include "PitchController.h"
#include "YawController.h"
#include "drivers_singleton.hpp"
#include <random>


namespace ThornBots {
    tap::communication::serial::RefSerialTransmitter *RefSerialTransmitter;
    class UI {
    public:  // Public Variables
        // constexpr static int YAW_MOTOR_MAX_SPEED = 1000; 

    private:  // Private Variables
        tap::Drivers* drivers;
        // TODO: Check all motor ID's, and verify indexers and flywheels are in the correct direction

        // ThornBots::YawController yawController = YawController();
        // ThornBots::PitchController pitchController = PitchController();



    public:  // Public Methods
        UI(tap::Drivers* driver);
        ~UI() {}  // Intentionally left blank

        /*
         * Call this function once, outside of the main loop.
         * This function will initalize all of the motors, timers, pidControllers, and any other used object.
         * If you want to know what initializing actually does, ping Teaney in discord, or just Google it. It's pretty cool.
         */
        void initialize();

        /*
         * Should be called within the main loop, so called every time in the main loop when you want the described behavior.
         * This will allow the drivetrain to translate with the left stick, and the right stick is for the turret.
         * This function should be called when the right switch is in the Down state.
         * Enabling beyblading (left switch is not down) will override this state, and left stick will control drivetrain translating
         * and right stick will control pitch and yaw of the turret.
         */
        // void turretMove(double desiredYawAngle, double desiredPitchAngle);


        // inline void disable() { robotDisabled = true; }
        // inline void enable() { robotDisabled = false; }


    private:  // Private Methods
        // int getPitchVoltage(double targetAngle, double dt);
    };
}  // namespace ThornBots