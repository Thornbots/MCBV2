#include "ShooterSubsystem.h"

namespace ThornBots {
    ShooterSubsystem::ShooterSubsystem(tap::Drivers* driver) {
        this->drivers = driver;
        // TODO: Complete this
    }
    void ShooterSubsystem::initialize() {
        motor_Indexer.initialize();
        motor_Flywheel1.initialize();
        motor_Flywheel2.initialize();
        drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER1, 333);  // Timer 1 for C1 Pin

        // Nothing needs to be done to drivers
        // Nothing needs to be done to the controllers
    }
    void ShooterSubsystem::updateSpeeds() {
        if (shooterControllerTimer.execute()) {
            indexerVoltage = getIndexerVoltage();
            flyWheelVoltage = getFlywheelVoltage();
        }

    }

    void ShooterSubsystem::setMotorSpeeds() {
        updateSpeeds();
        indexPIDController.runControllerDerivateError(indexerVoltage - motor_Indexer.getShaftRPM(), 1);
        motor_Indexer.setDesiredOutput(static_cast<int32_t>(indexPIDController.getOutput()));

        flywheelPIDController1.runControllerDerivateError(flyWheelVoltage - motor_Flywheel1.getShaftRPM(), 1);
        motor_Flywheel1.setDesiredOutput(static_cast<int32_t>(flywheelPIDController1.getOutput()));

        flywheelPIDController2.runControllerDerivateError(flyWheelVoltage - motor_Flywheel2.getShaftRPM(), 1);
        motor_Flywheel2.setDesiredOutput(static_cast<int32_t>(flywheelPIDController2.getOutput()));
        
        if(servoTimer.execute()) hopperServo.updateSendPwmRamp();
    }

    void ShooterSubsystem::stopMotors() {
        // indexPIDController.runControllerDerivateError(0 - motor_Indexer.getShaftRPM(), 1);
        motor_Indexer.setDesiredOutput(0);  // static_cast<int32_t>(indexPIDController.getOutput()));

        // flywheelPIDController1.runControllerDerivateError(0 - motor_Flywheel1.getShaftRPM(), 1);
        motor_Flywheel1.setDesiredOutput(0);  // static_cast<int32_t>(flywheelPIDController1.getOutput()));

        // flywheelPIDController2.runControllerDerivateError(0 - motor_Flywheel2.getShaftRPM(), 1);
        motor_Flywheel2.setDesiredOutput(0);  // static_cast<int32_t>(flywheelPIDController2.getOutput()));

        //openServo();  // open when robot is disabled so it is out of the way
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

    int ShooterSubsystem::getIndexerVoltage() {
        if (robotDisabled) return 0;
        return indexerVoltage;
    }

    void ShooterSubsystem::setIndexer(double val) { indexerVoltage = val * INDEXER_MOTOR_MAX_SPEED; }

    void ShooterSubsystem::shoot(double maxFrequency) {
        enableShooting();
        closeServo();  // when shooting resumes close servo to prevent balls from leaving
        tap::communication::serial::RefSerial::Rx::TurretData turretData = drivers->refSerial.getRobotData().turret;

        double latency = 0.4, remaining = turretData.heatLimit - turretData.heat17ID1;
        // Check if the firing rate should be limited
        if (drivers->refSerial.getRefSerialReceivingData() && (10.0 * maxFrequency - turretData.coolingRate) * latency > remaining) {
            // Set the firing speed to C/10 Hz
            maxFrequency = turretData.coolingRate / 10.0;
        }

        setIndexer(maxFrequency / 20.0);
    }

    void ShooterSubsystem::setServo(float val) { if(secondTimer.execute()) hopperServo.setTargetPwm(val); }
}  // namespace ThornBots
