#pragma once
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"

#include "drivers_singleton.hpp"

namespace ThornBots {
    static tap::arch::PeriodicMilliTimer shooterControllerTimer(2);
    enum IndexCommand { IDLE, UNJAM, SINGLE, RAPID };
    class ShooterSubsystem {
    public:  // Public Variables
        // constexpr static double PI = 3.14159;
        constexpr static int INDEXER_MOTOR_MAX_SPEED = 18000;   // With the 2006, this should give 20Hz
        constexpr static int FLYWHEEL_MOTOR_MAX_SPEED = 11000;  // We had 5000 last year, and we can go 30/18 times as fast. So 5000 * 30/18
        constexpr static tap::algorithms::SmoothPidConfig pid_conf_flywheel = {20, 0, 0, 0, 13000, 1, 0, 1, 0, 0, 0};
        constexpr static tap::algorithms::SmoothPidConfig pid_conf_index = {5, 0, 0, 0, 8000, 1, 0, 1, 0, 10, 0};

    private:  // Private Variables
        tap::Drivers* drivers;
        // TODO: Check all motor ID's, and verify indexers and flywheels are in the correct direction
        tap::motor::DjiMotor motor_Indexer =
            tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR1, tap::can::CanBus::CAN_BUS2, false, "Indexer", 0, 0);
        tap::motor::DjiMotor motor_LowerFeeder =
            tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR4, tap::can::CanBus::CAN_BUS2, true, "Lower Feeder", 0, 0);
        tap::motor::DjiMotor motor_UpperFeeder =
            tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR7, tap::can::CanBus::CAN_BUS2, false, "Upper Feeder", 0, 0);

        tap::motor::DjiMotor motor_Flywheel1 =
            tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR5, tap::can::CanBus::CAN_BUS2, true, "Flywheel", 0, 0);
        tap::motor::DjiMotor motor_Flywheel2 =
            tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR3, tap::can::CanBus::CAN_BUS2, false, "Flywheel", 0, 0);
        tap::algorithms::SmoothPid flywheelPIDController1 = tap::algorithms::SmoothPid(pid_conf_flywheel);
        tap::algorithms::SmoothPid flywheelPIDController2 = tap::algorithms::SmoothPid(pid_conf_flywheel);
        tap::algorithms::SmoothPid indexPIDController = tap::algorithms::SmoothPid(pid_conf_index);
        tap::algorithms::SmoothPid lowerFeederPIDController = tap::algorithms::SmoothPid(pid_conf_index);
        tap::algorithms::SmoothPid upperFeederPIDController = tap::algorithms::SmoothPid(pid_conf_index);

        using pin = Board::DigitalInPinB12;

        double flyWheelVoltage, indexerVoltage = 0.0, lowerFeederVoltage = 0.0, upperFeederVoltage = 0.0;

        bool beamState = true;
        bool shootingSafety = false;
        bool robotDisabled = false;

        // stuff for hero burst fire
        double startIndexerPosition = 0;
        int numberOfShots;
        bool isRapidStart = true;  // if we need to do the calculations

    public:  // Public Methods
        ShooterSubsystem(tap::Drivers* driver);
        ~ShooterSubsystem() {}  // Intentionally left blank

        /*
         * Call this function once, outside of the main loop.
         * This function will initalize all of the motors, timers, pidControllers, and any other used object.
         * If you want to know what initializing actually does, ping Teaney in discord, or just Google it. It's pretty cool.
         */
        void initialize();
        /*
         * Call this function to convert the desired RPM for all of motors in the GimbalSubsystem to a voltage level which
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

        inline void enableShooting() { this->shootingSafety = true; }

        inline void disableShooting() { this->shootingSafety = false; }
        inline void enable() { this->robotDisabled = false; }
        inline void disable() { this->robotDisabled = true; }

        void index(IndexCommand* cmd);

        void setIndexer(double val);

        void setLowerFeeder(double val);
        void setUpperFeeder(double val);

    private:  // Private Methods
        int getFlywheelVoltage();
        int getIndexerVoltage();
        int getLowerFeederVoltage();
        int getUpperFeederVoltage();
        inline void setAllIndex(double upper, double lower, double index) {
            setIndexer(index);
            setUpperFeeder(upper);
            setLowerFeeder(lower);
        }
        bool readSwitch();
    };
}  // namespace ThornBots