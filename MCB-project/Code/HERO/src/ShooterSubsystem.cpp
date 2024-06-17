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
        motor_LowerFeeder.initialize();
        motor_UpperFeeder.initialize();
        drivers->pwm.init();  // For the servo we will be using
        pin::configure(modm::platform::Gpio::InputType::Floating);

        // Nothing needs to be done to drivers
        // Nothing needs to be done to the controllers
    }
    void ShooterSubsystem::updateSpeeds() {
        if (shooterControllerTimer.execute()) {
            beamState = readSwitch();
            index();
            indexerVoltage = getIndexerVoltage();
            flyWheelVoltage = getFlywheelVoltage();
            lowerFeederVoltage = getLowerFeederVoltage();
            upperFeederVoltage = getUpperFeederVoltage();
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

        lowerFeederPIDController.runControllerDerivateError(lowerFeederVoltage - motor_LowerFeeder.getShaftRPM(), 1);
        motor_LowerFeeder.setDesiredOutput(static_cast<int32_t>(lowerFeederPIDController.getOutput()));

        upperFeederPIDController.runControllerDerivateError(upperFeederVoltage - motor_UpperFeeder.getShaftRPM(), 1);
        motor_UpperFeeder.setDesiredOutput(static_cast<int32_t>(upperFeederPIDController.getOutput()));
    }

    void ShooterSubsystem::stopMotors() {
        motor_Indexer.setDesiredOutput(0);  // static_cast<int32_t>(indexPIDController.getOutput()));

        motor_Flywheel1.setDesiredOutput(0);  // static_cast<int32_t>(flywheelPIDController1.getOutput()));

        motor_Flywheel2.setDesiredOutput(0);  // static_cast<int32_t>(flywheelPIDController2.getOutput()));

        motor_LowerFeeder.setDesiredOutput(0);

        motor_UpperFeeder.setDesiredOutput(0);

        drivers->djiMotorTxHandler.encodeAndSendCanData();
        // TODO: Add the other motors
    }
    bool ShooterSubsystem::readSwitch() {
        // return drivers->digital.read(rxPin);
        return pin::read();
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
    int ShooterSubsystem::getLowerFeederVoltage() {
        if (robotDisabled) return 0;
        return lowerFeederVoltage;
    }
    int ShooterSubsystem::getUpperFeederVoltage() {
        if (robotDisabled) return 0;
        return upperFeederVoltage;
    }

    //statics got moved to be private member variables in the header file as this is more consistent and better practice imho
    void ShooterSubsystem::index() {
        tap::communication::serial::RefSerialData::Rx::TurretData turretData = drivers->refSerial.getRobotData().turret;
        int coolingRate = turretData.coolingRate, heatRemaining = turretData.heatLimit - turretData.heat42;

        switch (cmd) {
            case UNJAM:
                disableShooting();
                isRapidStart = true;
                setAllIndex(-0.4, -0.4, -0.4);
                //set state to idle for next time. If unjam is held this will do nothing
                idle();
                break;

            case RAPID:
                enableShooting();

                if (isRapidStart) {
                    numberOfShots = heatRemaining / 100;
                    
                    numberOfShots = std::floor((numberOfShots / BURST_FIRE_RATE * coolingRate + heatRemaining - LATENCY*coolingRate) / 100);
                    
                    numberOfShots = std::min(numberOfShots, (int)turretData.bulletsRemaining42);
                    startIndexerPosition = motor_Indexer.getEncoderUnwrapped();
                    isRapidStart = false;
                } else {
                    double indexerPosition = motor_Indexer.getEncoderUnwrapped();
                    double traveledDistance = indexerPosition - startIndexerPosition;

                    if (traveledDistance > (8192 * 36 * (numberOfShots - 1) / 5)) {
                        isRapidStart = true;
                        setAllIndex(0, 0, 0);
                        idle();
                        break;
                    }
                }

                setAllIndex(1, 0.8, 0.12);
                break;
            case SINGLE:
                enableShooting();

                isRapidStart = true;
                if (drivers->refSerial.getRefSerialReceivingData() && (heatRemaining < 100)) {
                    // if we don't have ammo to shoot, don't shoot. also make sure we are actually connected to the ref system
                    // this serves as a failsafe
                    idle();
                    // no break intentional, want it to do idle
                } else {
                    if (readSwitch()) {
                        setAllIndex(0.5, 0, 0);
                        idle();

                    } else {
                        setAllIndex(0.2, 0.1, 0.02);
                    }
                    break;
                }
            default:  // case IDLE
                isRapidStart = true;
                if (readSwitch())
                    setAllIndex(0.2, 0.4, 0.15);
                else
                    setAllIndex(0, 0, 0);
                break;
        }
    }

    void ShooterSubsystem::setIndexer(double val) { indexerVoltage = val * INDEXER_MOTOR_MAX_SPEED; }

    void ShooterSubsystem::setLowerFeeder(double val) { lowerFeederVoltage = val * INDEXER_MOTOR_MAX_SPEED; }
    void ShooterSubsystem::setUpperFeeder(double val) { upperFeederVoltage = val * INDEXER_MOTOR_MAX_SPEED; }

}  // namespace ThornBots
