#include "DrivetrainSubsystem.h"

namespace ThornBots {
    static double motorOneSpeed, motorTwoSpeed, motorThreeSpeed, motorFourSpeed = 0;
    DrivetrainSubsystem::DrivetrainSubsystem(tap::Drivers* driver) { this->drivers = driver; }

    void DrivetrainSubsystem::initialize() {
        motor_one.initialize();
        motor_two.initialize();
        motor_three.initialize();
        motor_four.initialize();
        // Nothing needs to be done to drivers
        // Nothing needs to be done to PID controllers
    }

    void DrivetrainSubsystem::moveDriveTrain(double turnSpeed, double translationSpeed, double translationAngle) {
        double angularOffset = 0;  //(motor_one.getShaftRPM()+motor_three.getShaftRPM())/2/14000.0;

        convertTranslationSpeedToMotorSpeeds(translationSpeed, translationAngle + angularOffset + PI / 6);

        adjustMotorSpeedWithTurnSpeed(turnSpeed);
    }

    void DrivetrainSubsystem::setMotorSpeeds() {
        if (robotDisabled) return stopMotors();
        drivers->canRxHandler.pollCanData();

        double powerLimit = DEFAULT_LIMIT;                   // failsafe
        if (drivers->refSerial.getRefSerialReceivingData())  // check for uart disconnected
            powerLimit = drivers->refSerial.getRobotData().chassis.powerConsumptionLimit;
        powerLimit += limitIncrease;  // add limit increase

        // shaft rpms measured from encoders
        motorOneRPM = motor_one.getShaftRPM();
        motorTwoRPM = motor_two.getShaftRPM();
        motorThreeRPM = motor_three.getShaftRPM();
        motorFourRPM = motor_four.getShaftRPM();

        double w1 = motorOneRPM * (2 * M_PI / 60);    // Convert RPM to rad/s
        double w2 = motorTwoRPM * (2 * M_PI / 60);    // Convert RPM to rad/s
        double w3 = motorThreeRPM * (2 * M_PI / 60);  // Convert RPM to rad/s
        double w4 = motorFourRPM * (2 * M_PI / 60);   // Convert RPM to rad/s

        // set the outputs according to the pid controller and get currents adjusted for saturation
        pidController.runControllerDerivateError(motorOneSpeed - motorOneRPM, 1);
        double I1t = std::clamp(static_cast<double>(pidController.getOutput()) / 819.2, -abs(VOLT_MAX - KB * w1) / RA, abs(VOLT_MAX - KB * w1) / RA);

        pidController.runControllerDerivateError(motorTwoSpeed - motorTwoRPM, 1);
        double I2t = std::clamp(static_cast<double>(pidController.getOutput()) / 819.2, -abs(VOLT_MAX - KB * w2) / RA, abs(VOLT_MAX - KB * w2) / RA);

        pidController.runControllerDerivateError(motorThreeSpeed - motorThreeRPM, 1);
        double I3t = std::clamp(static_cast<double>(pidController.getOutput()) / 819.2, -abs(VOLT_MAX - KB * w3) / RA, abs(VOLT_MAX - KB * w3) / RA);

        pidController.runControllerDerivateError(motorFourSpeed - motorFourRPM, 1);
        double I4t = std::clamp(static_cast<double>(pidController.getOutput()) / 819.2, -abs(VOLT_MAX - KB * w4) / RA, abs(VOLT_MAX - KB * w4) / RA);

        // Calculate total power requested
        double totalPowerRequested = 0.8 * RA * (I1t * I1t + I2t * I2t + I3t * I3t + I4t * I4t) + VELO_LOSS * (abs(w1) + abs(w2) + abs(w3) + abs(w4));

        // Scale currents if power limit is exceeded
        double scale = std::max((double)1.0, (totalPowerRequested + IDLE_DRAW) / powerLimit);

        motor_one.setDesiredOutput(static_cast<int32_t>(I1t * 819.2 / scale));
        motor_two.setDesiredOutput(static_cast<int32_t>(I2t * 819.2 / scale));
        motor_three.setDesiredOutput(static_cast<int32_t>(I3t * 819.2 / scale));
        motor_four.setDesiredOutput(static_cast<int32_t>(I4t * 819.2 / scale));

        drivers->djiMotorTxHandler.encodeAndSendCanData();  // Processes these motor speed changes into CAN signal
    }

    void DrivetrainSubsystem::stopMotors() {
        motor_one.setDesiredOutput(0);
        motor_two.setDesiredOutput(0);
        motor_three.setDesiredOutput(0);
        motor_four.setDesiredOutput(0);
        drivers->djiMotorTxHandler.encodeAndSendCanData();
    }

    void DrivetrainSubsystem::convertTranslationSpeedToMotorSpeeds(double translationSpeed, double translationAngle) {
        motorOneSpeed = translationSpeed * sin(translationAngle + (PI / 4));
        motorTwoSpeed = translationSpeed * sin(translationAngle - (PI / 4));
        motorThreeSpeed = translationSpeed * sin(translationAngle - (PI / 4));
        motorFourSpeed = translationSpeed * sin(translationAngle + (PI / 4));
    }

    void DrivetrainSubsystem::adjustMotorSpeedWithTurnSpeed(double turnSpeed) {
        motorOneSpeed += turnSpeed;
        motorTwoSpeed -= turnSpeed;
        motorThreeSpeed += turnSpeed;
        motorFourSpeed -= turnSpeed;
    }

    void DrivetrainSubsystem::setHigherPowerLimit() {
        if (drivers->refSerial.getRobotData().chassis.powerBuffer > MIN_BUFFER)
            limitIncrease = HIGH_LIM_INC;
        else
            setRegularPowerLimit();
    }

    void DrivetrainSubsystem::setRegularPowerLimit() { limitIncrease = REG_LIM_INC; }
}  // namespace ThornBots