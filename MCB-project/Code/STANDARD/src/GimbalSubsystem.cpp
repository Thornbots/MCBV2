#include "GimbalSubsystem.h"

namespace ThornBots {
    GimbalSubsystem::GimbalSubsystem(tap::Drivers* driver) {
        this->drivers = driver;
        // TODO: Complete this
    }
    void GimbalSubsystem::initialize() {
        motor_Pitch.initialize();
        motor_Yaw.initialize();
        // Nothing needs to be done to drivers
        // Nothing needs to be done to the controllers
    }

    void GimbalSubsystem::turretMove(double desiredYawAngle, double desiredPitchAngle, double driveTrainRPM, double yawAngleRelativeWorld,
                                     double yawRPM, double inputVel, double dt) {
        if (turretControllerTimer.execute()) {
            pitchMotorVoltage = getPitchVoltage(desiredPitchAngle - 0.48 * PI, dt);
            yawMotorVoltage = getYawVoltage(driveTrainRPM, yawAngleRelativeWorld, yawRPM, desiredYawAngle, inputVel, dt);
        }
        // TODO: Add flywheels, indexer, and servo
    }

    void GimbalSubsystem::setMotorSpeeds() {
        motor_Pitch.setDesiredOutput(pitchMotorVoltage);
        motor_Yaw.setDesiredOutput(yawMotorVoltage);
    }

    void GimbalSubsystem::stopMotors() {
        motor_Pitch.setDesiredOutput(0);
        motor_Yaw.setDesiredOutput(0);

        drivers->djiMotorTxHandler.encodeAndSendCanData();
        // TODO: Add the other motors
    }

    void GimbalSubsystem::reZeroYaw() {
        // TODO
    }

    int GimbalSubsystem::getYawVoltage(double driveTrainRPM, double yawAngleRelativeWorld, double yawRPM, double desiredAngleWorld, double inputVel,
                                       double dt) {
        if (robotDisabled) return 0;
        return 1000 * yawController.calculate(yawAngleRelativeWorld, yawRPM, 0, desiredAngleWorld, inputVel,
                                              dt);  // 1000 to convert to mV which taproot wants. DTrpm is 0, can calculate and pass in the future
    }

    int GimbalSubsystem::getPitchVoltage(double targetAngle, double dt) {
        if (robotDisabled) return 0;
        return 1000 * pitchController.calculate(getPitchEncoderValue(), getPitchVel(), targetAngle, dt);
    }

}  // namespace ThornBots
