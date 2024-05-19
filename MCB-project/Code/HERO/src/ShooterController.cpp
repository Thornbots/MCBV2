#include "ShooterController.h"

namespace ThornBots {
    ShooterController::ShooterController(tap::Drivers* driver) {
        this->drivers = driver;
        //TODO: Complete this
    }
    void ShooterController::initialize() {
        motor_Indexer.initialize();
        motor_Flywheel1.initialize();
        motor_Flywheel2.initialize();
        motor_LowerFeeder.initialize();
        motor_UpperFeeder.initialize();
        drivers->pwm.init(); //For the servo we will be using
        pin::configure(modm::platform::Gpio::InputType::Floating);


        //Nothing needs to be done to drivers
        //Nothing needs to be done to the controllers
    }
    void ShooterController::updateSpeeds(){
        if(shooterControllerTimer.execute()) {
            beamState = readSwitch();
            indexerVoltage = getIndexerVoltage();
            flyWheelVoltage = getFlywheelVoltage();
            lowerFeederVoltage = getLowerFeederVoltage();
            upperFeederVoltage = getUpperFeederVoltage();

        }
    }

    void ShooterController::setMotorSpeeds() {
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

    void ShooterController::stopMotors() {
        //indexPIDController.runControllerDerivateError(0 - motor_Indexer.getShaftRPM(), 1);
        motor_Indexer.setDesiredOutput(0);//static_cast<int32_t>(indexPIDController.getOutput()));

       // flywheelPIDController1.runControllerDerivateError(0 - motor_Flywheel1.getShaftRPM(), 1);
        motor_Flywheel1.setDesiredOutput(0);//static_cast<int32_t>(flywheelPIDController1.getOutput()));

       // flywheelPIDController2.runControllerDerivateError(0 - motor_Flywheel2.getShaftRPM(), 1);
        motor_Flywheel2.setDesiredOutput(0);//static_cast<int32_t>(flywheelPIDController2.getOutput()));

        motor_LowerFeeder.setDesiredOutput(0);

        motor_UpperFeeder.setDesiredOutput(0);

        drivers->djiMotorTxHandler.encodeAndSendCanData();
        //TODO: Add the other motors
    }
    bool ShooterController::readSwitch() {
        // return drivers->digital.read(rxPin);
        return pin::read();
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
            return indexerVoltage;

    }
    int ShooterController::getLowerFeederVoltage() {
        if (robotDisabled) return 0;
            return lowerFeederVoltage;

    }    
    int ShooterController::getUpperFeederVoltage() {
        if (robotDisabled) return 0;
            return upperFeederVoltage;

    } 


    void ShooterController::index(IndexCommand * cmd){
        switch(*cmd){
            case UNJAM:
                setAllIndex(0, -0.1, -0.1);
                break;
            case RAPID:
                setAllIndex(1, 1, 1);
                break;
            case SINGLE:
                if(readSwitch()){
                    setAllIndex(0.3, 0, 0);
                    *cmd = IDLE;

                }else{                    
                    setAllIndex(0.3, 0.3, 0.3);
                }
                break;
            default:
                if(readSwitch())
                    setAllIndex(0.3, 0.3, 0.3); 
                else                    
                    setAllIndex(0, 0, 0);
                break;

        }
    }


    void ShooterController::setIndexer(double val) {
        indexerVoltage = val*INDEXER_MOTOR_MAX_SPEED;
        
    }

    void ShooterController::setLowerFeeder(double val){
        lowerFeederVoltage = val*INDEXER_MOTOR_MAX_SPEED;
    }
   void ShooterController::setUpperFeeder(double val){
        upperFeederVoltage = val*INDEXER_MOTOR_MAX_SPEED;
    }

    void ShooterController::disable(){
        robotDisabled = true;
    }
    void ShooterController::enable(){
        robotDisabled = false;
    }

}
