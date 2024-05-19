#include "ShooterController.h"

namespace ThornBots {
    ShooterController::ShooterController(tap::Drivers* driver) {
        this->drivers = driver;
        //TODO: Complete this
    }
    void ShooterController::initialize() {
        motor_Indexer1.initialize();
        motor_Indexer2.initialize();
        motor_Flywheel1.initialize();
        motor_Flywheel2.initialize();
        drivers->pwm.init(); //For the servo we will be using

        //Nothing needs to be done to drivers
        //Nothing needs to be done to the controllers
    }
    void ShooterController::updateSpeeds(){
        if(shooterControllerTimer.execute()) {
            indexerVoltage = getIndexerVoltage();
            testIndexerVoltage = getTestIndexerVoltage();
            flyWheelVoltage = getFlywheelVoltage();
        }
    }

    void ShooterController::setMotorSpeeds() {
        updateSpeeds();
        indexPIDController1.runControllerDerivateError(indexerVoltage - motor_Indexer1.getShaftRPM(), 1);
        motor_Indexer1.setDesiredOutput(static_cast<int32_t>(indexPIDController1.getOutput()));

        indexPIDController2.runControllerDerivateError(testIndexerVoltage - motor_Indexer2.getShaftRPM(), 1);
        motor_Indexer2.setDesiredOutput(static_cast<int32_t>(indexPIDController2.getOutput()));

        flywheelPIDController1.runControllerDerivateError(flyWheelVoltage - motor_Flywheel1.getShaftRPM(), 1);
        motor_Flywheel1.setDesiredOutput(static_cast<int32_t>(flywheelPIDController1.getOutput()));

        flywheelPIDController2.runControllerDerivateError(flyWheelVoltage - motor_Flywheel2.getShaftRPM(), 1);
        motor_Flywheel2.setDesiredOutput(static_cast<int32_t>(flywheelPIDController2.getOutput()));
    }

    void ShooterController::stopMotors() {
             //indexPIDController.runControllerDerivateError(0 - motor_Indexer.getShaftRPM(), 1);
        motor_Indexer1.setDesiredOutput(0);//static_cast<int32_t>(indexPIDController.getOutput()));

        motor_Indexer2.setDesiredOutput(0);//static_cast<int32_t>(indexPIDController.getOutput()));

       // flywheelPIDController1.runControllerDerivateError(0 - motor_Flywheel1.getShaftRPM(), 1);
        motor_Flywheel1.setDesiredOutput(0);//static_cast<int32_t>(flywheelPIDController1.getOutput()));

       // flywheelPIDController2.runControllerDerivateError(0 - motor_Flywheel2.getShaftRPM(), 1);
        motor_Flywheel2.setDesiredOutput(0);//static_cast<int32_t>(flywheelPIDController2.getOutput()));

        drivers->djiMotorTxHandler.encodeAndSendCanData();
        //TODO: Add the other motors
    }

    void ShooterController::enableShooting() {
        this->shootingSafety = true;
    }

    void ShooterController::disableShooting() {
        this->shootingSafety = false;
    }

    int ShooterController::getFlywheelVoltage() {
        if (robotDisabled) return 0;
        if(shootingSafety){
            return FLYWHEEL_MOTOR_MAX_SPEED;
        }else{
            return 0;
        }
    }

    int ShooterController::getIndexerVoltage() {
        if (robotDisabled) return 0;
        if(shootingSafety){
            return indexerVoltage;
        }else{
            return 0;
        }
    }
      int ShooterController::getTestIndexerVoltage() {
        if (robotDisabled) return 0;
        return testIndexerVoltage;

    }

    void ShooterController::setIndexer(double val) {
        indexerVoltage = val*INDEXER_MOTOR_MAX_SPEED;
        
    }
    void ShooterController::setTestIndexer(double val){
        testIndexerVoltage = val*INDEXER_MOTOR_MAX_SPEED;
    }

    void ShooterController::disable(){
        robotDisabled = true;
    }
    void ShooterController::enable(){
        robotDisabled = false;
    }

}
