#pragma once
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/board/board.hpp"
#include "drivers_singleton.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/motor/dji_motor.hpp"
#include "drivers_singleton.hpp"


namespace ThornBots {
    static tap::arch::PeriodicMilliTimer shooterControllerTimer(2);
    class ShooterController {
        public: //Public Variables
            constexpr static int INDEXER_MOTOR_MAX_SPEED = 6177; //With the 2006, this should give 20Hz
            constexpr static int FLYWHEEL_MOTOR_MAX_SPEED = 8333; //We had 5000 last year, and we can go 30/18 times as fast. So 5000 * 30/18
            constexpr static tap::algorithms::SmoothPidConfig pid_conf_flywheel = { 20, 0, 0, 0, 8000, 1, 0, 1, 0, 0, 0 };
            constexpr static tap::algorithms::SmoothPidConfig pid_conf_index = { 5, 0, 0, 0, 8000, 1, 0, 1, 0, 10, 0 };
        private: //Private Variables
            tap::Drivers* drivers;
            //TODO: Check all motor ID's, and verify indexers and flywheels are in the correct direction
            tap::motor::DjiMotor motor_Indexer1 = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR7, tap::can::CanBus::CAN_BUS2, false, "Left Indexer", 0, 0);
            tap::motor::DjiMotor motor_Indexer2 = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR6, tap::can::CanBus::CAN_BUS2, true, "Right Indexer", 0, 0);

            tap::motor::DjiMotor motor_Flywheel1 = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR3, tap::can::CanBus::CAN_BUS2, false, "Top Flywheel", 0, 0);
            tap::motor::DjiMotor motor_Flywheel2 = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR2, tap::can::CanBus::CAN_BUS2, false, "Bottom Flywheel", 0, 0);
             tap::algorithms::SmoothPid flywheelPIDController1 = tap::algorithms::SmoothPid(pid_conf_flywheel);
            tap::algorithms::SmoothPid flywheelPIDController2 = tap::algorithms::SmoothPid(pid_conf_flywheel);
            tap::algorithms::SmoothPid indexPIDController1 = tap::algorithms::SmoothPid(pid_conf_index);
            tap::algorithms::SmoothPid indexPIDController2 = tap::algorithms::SmoothPid(pid_conf_index);

            double flyWheelVoltage, indexer1Voltage = 0.0, indexer2Voltage = 0.0;


            bool shootingSafety = false;
            bool robotDisabled = false;

        public: //Public Methods
            ShooterController(tap::Drivers* driver);
            ~ShooterController() {} //Intentionally left blank

            /*
            * Call this function once, outside of the main loop.
            * This function will initalize all of the motors, timers, pidControllers, and any other used object.
            * If you want to know what initializing actually does, ping Teaney in discord, or just Google it. It's pretty cool.
            */
            void initialize();
            /*
            * Call this function to convert the desired RPM for all of motors in the TurretController to a voltage level which
            * would then be sent over CanBus to each of the motor controllers to actually set this voltage level on each of the motors.
            * Should be placed inside of the main loop, and called every time through the loop, ONCE
            */
            void setMotorSpeeds();

            void updateSpeeds();
            /*
            * Call this function to set all Turret motors to 0 desired RPM, calculate the voltage level in which to achieve this quickly
            * and packages this information for the motors TO BE SENT over CanBus
            */
            void stopMotors();
            void disable();
            void enable();

            /*
            * Call this function (any number of times) in order to ALLOW shooting. This does NOT mean that the turret WILL shoot.
            * The idea of this function is to allow implementation of AI auto-shooting easily, by "giving control" of the turret to the
            * communicatons received from the JetsonInput.
            * This function is not intended to be used for control when the driver is manually aiming/deciding to shoot or not.
            */
            void enableShooting();

            
            void setIndexer1(double val);
            void setIndexer2(double val);
            void setIndexers(double val);

            /*
            * Call this function (any number of times) in order to DISALLOW shooting. This does NOT mean that the turret WON'T shoot.
            * The idea of this function is to allow implementation of AI auto-shooting easily, by "giving control" of the turret to the 
            * communications received from the JetsonInput.
            * This function is not intended to be used for conrtol when the driver is manually aiming/deciding to shoot or not.
            */
            void disableShooting();

        private: //Private Methods
      
            int getFlywheelVoltage();
            int getIndexer1Voltage();
            int getIndexer2Voltage();
    };
}