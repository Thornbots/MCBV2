#pragma once
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"

#include "drivers_singleton.hpp"

namespace ThornBots {
    class DrivetrainSubsystem {
    public:  // Public Variables
        // constexpr static double PI = 3.14159; //Everyone likes Pi!
        constexpr static tap::algorithms::SmoothPidConfig pid_conf_dt = {20, 0, 0, 0, 18000, 1, 0, 1, 0, 0, 0};
        constexpr static tap::algorithms::SmoothPidConfig pid_conf_DriveTrainFollowsTurret = {500, 0.5, 0, 0, 18000, 1,
                                                                                              0,   1,   0, 0, 0};  // TODO: Tune this profile
    private:                                                                                                       // Private Variables
        tap::Drivers* drivers;
        tap::motor::DjiMotor motor_one =
            tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR1, tap::can::CanBus::CAN_BUS1, true, "ID1", 0, 0);
        tap::motor::DjiMotor motor_two =
            tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR2, tap::can::CanBus::CAN_BUS1, false, "PURDON'T!", 0, 0);
        tap::motor::DjiMotor motor_three = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR3, tap::can::CanBus::CAN_BUS1,
                                                                true, "Put the possum in his room", 0, 0);
        tap::motor::DjiMotor motor_four =
            tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR4, tap::can::CanBus::CAN_BUS1, false, "ID4", 0, 0);
        tap::algorithms::SmoothPid pidController = tap::algorithms::SmoothPid(pid_conf_dt);
        tap::algorithms::SmoothPid pidControllerDTFollowsT = tap::algorithms::SmoothPid(pid_conf_DriveTrainFollowsTurret);
        bool robotDisabled = false;
        double motorOneRPM, motorTwoRPM, motorThreeRPM, motorFourRPM = 0.0;

        const double HIGH_LIM_INC = 60;      // in watts, however many watts over the current limit given by the ref system
        const double REG_LIM_INC = 0;        // in watts, should be zero
        double limitIncrease = REG_LIM_INC;  // in watts

        const double MIN_BUFFER =
            10;  // in joules, how much should remain unused in the buffer (disables limitIncrease if the buffer is less than this)
        const double VOLT_MAX = 24;      // Volts
        const double RA = 0.194 - 0.01;  // ohms //was 1.03 or 0.194
        const double KB = 0.35 / 19.2;   // volt-rad/s  //0.39
        const double VELO_LOSS = 0.4;   // magic number representing loss from high rpm
        const double IDLE_DRAW = 3;      // watts, measured
        const double DEFAULT_LIMIT = 100;

    public:  // Public Methods
        DrivetrainSubsystem(tap::Drivers* driver);
        ~DrivetrainSubsystem() {}  // Intentionally blank

        /*
         * Call this function once, outside of the main loop.
         * This function will initalize all of the motors, timers, pidControllers, and any other used object.
         * If you want to know what initializing actually does, ping Teaney in discord, or just Google it. It's pretty cool.
         */
        void initialize();

        /*
         * Call this function when you want the Turret to follow the DriveTrain
         * Should be called within the main loop, so called every time in the main loop when you want the described behavior.
         * This will allow the drivetrain to translate with left stick, and turn with the right stick or beyblade depending how this is called.
         */
        void moveDriveTrain(double turnSpeed, double translationSpeed, double translationAngle);

        /*
         * Call this function to convert the desired RPM for all of motors in the DriveTrainController to a voltage level which
         * would then be sent over CanBus to each of the motor controllers to actually set this voltage level on each of the motors.
         * Should be placed inside of the main loop, and called periodically, every
         */
        void setMotorSpeeds();

        /*
         * Call this function to set all DriveTrain motors to 0 desired RPM. CALL setMotorSpeeds() FOR THIS TO WORK
         */
        void stopMotors();

        inline void disable() { robotDisabled = true; }
        inline void enable() { robotDisabled = false; }

        void setHigherPowerLimit();
        void setRegularPowerLimit();

    private:  // Private Methods
        /*
         * Call this function to calculate and OVERWRITE DriveTrain motors' RPMs to translate at the given magnitude and angle
         * Angle should be in radians, with 0 being straight forward relative to the drivetrain and the positive direction being CCW.
         * (i.e. So to translate directly to the left, relative to the drivetrain, you would call this function as:
         * convertTranslationSpeedToMotorSpeeds(0.75, pi/2);)
         */
        void convertTranslationSpeedToMotorSpeeds(double magnitude, double angle);

        /*
         * Call this function to calculate and ADJUST DriveTrain motors' RPMs to rotate at the given turnSpeed. This input is unitless, should be from
         * [0, 1], and simply multiplies it by a constant, adjustable maximum factor of maximum speed.
         */
        void adjustMotorSpeedWithTurnSpeed(double turnSpeed);
    };
}  // namespace ThornBots