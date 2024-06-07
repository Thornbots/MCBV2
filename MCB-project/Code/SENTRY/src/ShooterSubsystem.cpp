#include "ShooterSubsystem.h"

namespace ThornBots {
    ShooterSubsystem::ShooterSubsystem(tap::Drivers* driver) {
        this->drivers = driver;
        // TODO: Complete this
    }
    void ShooterSubsystem::initialize() {
        motor_Indexer1.initialize();
        motor_Indexer2.initialize();
        motor_Flywheel1.initialize();
        motor_Flywheel2.initialize();
        drivers->pwm.init();  // For the servo we will be using

        // Nothing needs to be done to drivers
        // Nothing needs to be done to the controllers
    }
    void ShooterSubsystem::updateSpeeds() {
        if (shooterControllerTimer.execute()) {
            indexer1Voltage = getIndexer1Voltage();
            indexer2Voltage = getIndexer2Voltage();
            // testIndexerVoltage = getTestIndexerVoltage();
            flyWheelVoltage = getFlywheelVoltage();
        }
    }

    void ShooterSubsystem::setMotorSpeeds() {
        updateSpeeds();
        indexPIDController1.runControllerDerivateError(indexer1Voltage - motor_Indexer1.getShaftRPM(), 1);
        motor_Indexer1.setDesiredOutput(static_cast<int32_t>(indexPIDController1.getOutput()));

        indexPIDController2.runControllerDerivateError(indexer2Voltage - motor_Indexer2.getShaftRPM(), 1);  // was testIndexerVoltage
        motor_Indexer2.setDesiredOutput(static_cast<int32_t>(indexPIDController2.getOutput()));

        flywheelPIDController1.runControllerDerivateError(flyWheelVoltage - motor_Flywheel1.getShaftRPM(), 1);
        motor_Flywheel1.setDesiredOutput(static_cast<int32_t>(flywheelPIDController1.getOutput()));

        flywheelPIDController2.runControllerDerivateError(flyWheelVoltage - motor_Flywheel2.getShaftRPM(), 1);
        motor_Flywheel2.setDesiredOutput(static_cast<int32_t>(flywheelPIDController2.getOutput()));
    }

    void ShooterSubsystem::stopMotors() {
        // indexPIDController.runControllerDerivateError(0 - motor_Indexer.getShaftRPM(), 1);
        motor_Indexer1.setDesiredOutput(0);  // static_cast<int32_t>(indexPIDController.getOutput()));

        motor_Indexer2.setDesiredOutput(0);  // static_cast<int32_t>(indexPIDController.getOutput()));

        // flywheelPIDController1.runControllerDerivateError(0 - motor_Flywheel1.getShaftRPM(), 1);
        motor_Flywheel1.setDesiredOutput(0);  // static_cast<int32_t>(flywheelPIDController1.getOutput()));

        // flywheelPIDController2.runControllerDerivateError(0 - motor_Flywheel2.getShaftRPM(), 1);
        motor_Flywheel2.setDesiredOutput(0);  // static_cast<int32_t>(flywheelPIDController2.getOutput()));

        drivers->djiMotorTxHandler.encodeAndSendCanData();
        // TODO: Add the other motors
    }

    int ShooterSubsystem::getFlywheelVoltage() {
        if (robotDisabled) return 0;
        if (shootingSafety) {
            return FLYWHEEL_MOTOR_MAX_SPEED;
        } else {
            return 0;
        }
    }

    int ShooterSubsystem::getIndexer1Voltage() {
        if (robotDisabled) return 0;
        return indexer1Voltage;
    }
    int ShooterSubsystem::getIndexer2Voltage() {
        if (robotDisabled) return 0;
        return indexer2Voltage;
    }

    void ShooterSubsystem::shoot(double maxFrequency) {
        enableShooting();

        tap::communication::serial::RefSerial::Rx::TurretData turretData = drivers->refSerial.getRobotData().turret;

        double frequency1 = maxFrequency / 2, frequency2 = maxFrequency / 2, latency = 0.4, remaining1 = turretData.heatLimit - turretData.heat17ID1,
               remaining2 = turretData.heatLimit - turretData.heat17ID2;
        // Check if the firing rate should be limited
        if (drivers->refSerial.getRefSerialReceivingData() && (10.0 * frequency1 - turretData.coolingRate) * latency > remaining1) {
            // Set the firing speed to C/10 Hz
            frequency1 = turretData.coolingRate / 10.0;
        }
        if (drivers->refSerial.getRefSerialReceivingData() && (10.0 * frequency2 - turretData.coolingRate) * latency > remaining2) {
            // Set the firing speed to C/10 Hz
            frequency2 = turretData.coolingRate / 10.0;
        }

        setIndexer1(frequency1 / 20.0);
        setIndexer2(frequency2 / 20.0);
    }

    void ShooterSubsystem::setIndexer1(double val) { indexer1Voltage = val * INDEXER_MOTOR_MAX_SPEED; }
    void ShooterSubsystem::setIndexer2(double val) { indexer2Voltage = val * INDEXER_MOTOR_MAX_SPEED; }

}  // namespace ThornBots
