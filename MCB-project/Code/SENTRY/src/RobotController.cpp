#include "RobotController.h"

#include <cmath>

namespace ThornBots
{
// double stickLeftHorz, stickLeftVert, stickRightHorz, stickRightVert, stickLeftAngle, stickLeftMagn,
//     stickRightAngle, stickRightMagn = 0.0;
double yawEncoderValue, IMUAngle = 0.0;
/*
 * Constructor for RobotController
 */
RobotController::RobotController(
    tap::Drivers* driver,
    ThornBots::DriveTrainController* driveTrainController,
    ThornBots::TurretController* turretController,
    ThornBots::ShooterController* shooterController):
    drivers(driver),
    driveTrainController(driveTrainController),
    turretController(turretController),
    shooterController(shooterController)
{
    // this->drivers = driver;
    // this->driveTrainController = driveTrainController;
    // this->turretController = turretController;
    // this->shooterController = shooterController;
}

void RobotController::initialize()
{
    Board::initialize();
    drivers->can.initialize();
    drivers->bmi088.initialize(500, 0.0, 0.0);
    drivers->bmi088.requestRecalibration();
    drivers->remote.initialize();
    driveTrainController->initialize();
    turretController->initialize();
    shooterController->initialize();
    modm::delay_ms(
        2500);  // Delay 2.5s to allow the IMU to turn on and get working before we move it around
    // TODO: Finish this (Add creating timers, maybe some code to setup the IMU and make sure it's
    // reading correctly, ect)
}

void RobotController::update()
{

    drivers->canRxHandler.pollCanData();
    updateAllInputVariables();

    toggleKeyboardAndMouse();


    if(drivers->remote.isConnected())
        enableRobot();
    else
        disableRobot();
        

    if (useKeyboardMouse)
    {
        if(robotDisabled) 
            return;
        //shooterController->enableShooting(); 
        updateWithMouseKeyboard();
    }
    else
    {
        if(robotDisabled) 
            return;
        //shooterController->disableShooting();
        updateWithController();
    }


    if (drivers->remote.isConnected())
    {
        if (motorsTimer.execute())
        {
            driveTrainController->setMotorSpeeds();
            turretController->setMotorSpeeds();
            shooterController->setMotorSpeeds();
        }
    }
    else
    {
        shooterController->disableShooting();
        stopRobot();
    }

    // drivers->djiMotorTxHandler.encodeAndSendCanData();  // Processes these motor speed changes
    // into can signal
}

void RobotController::stopRobot()
{
    driveTrainController->stopMotors();
    turretController->stopMotors();
    shooterController->stopMotors();
    robotDisabled = true;
}

void RobotController::disableRobot()
{
    stopRobot();
    driveTrainController->disable();
    turretController->disable();
    shooterController->disable();
}

void RobotController::enableRobot()
{
    robotDisabled = false;
    driveTrainController->enable();
    turretController->enable();
    shooterController->enable();
}

void RobotController::updateAllInputVariables()
{
    drivers->remote.read();  // Reading the remote before we check if it is connected yet or not.
    if (IMUTimer.execute())
    {
        drivers->bmi088.periodicIMUUpdate();
    }

    // START Updating stick values
    // Actually Reading from remote
    rightSwitchState =
        drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH);
    leftSwitchState =
        drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH);
    left_stick_horz =
        drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL);
    left_stick_vert =
        drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
    right_stick_horz =
        drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);
    right_stick_vert =
        drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
    // Turning the remote raw values into values we can use more easily (circular cordinates)
    leftStickAngle = getAngle(left_stick_horz, left_stick_vert);
    rightStickAngle = getAngle(right_stick_horz, right_stick_vert);
    leftStickMagnitude = getMagnitude(left_stick_horz, left_stick_vert);
    rightStickMagnitude = getMagnitude(right_stick_horz, right_stick_vert);
    // STOP Updating stick values

    driveTrainRPM = 0;  // TODO: get this. Either power from DT motors, using yaw encoder and IMU,
                        // or something else
    yawRPM = PI / 180 * drivers->bmi088.getGz();
    yawAngleRelativeWorld = PI / 180 * drivers->bmi088.getYaw();

    wheelValue = drivers->remote.getWheel();

}

double RobotController::getAngle(double x, double y)
{
    // error handling to prevent runtime errors in atan2
    if (x == 0 && y == 0)
    {
        return ((double)0.0);
    }

    return atan2(y, x);  // Return (double) [pi, pi] which we want. Doing x/y to rotate the unit
                         // circle 90 degrees CCW (make 0 straight ahead)
}

double RobotController::getMagnitude(double x, double y) { return sqrt(pow(x, 2) + pow(y, 2)); }

bool RobotController::toggleKeyboardAndMouse()
{
    // only gets set to false the first time this funtion is called
    static bool hasBeenReleased = true;  

    if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::CTRL)
    &&drivers->remote.keyPressed(tap::communication::serial::Remote::Key::SHIFT)
    &&drivers->remote.keyPressed(tap::communication::serial::Remote::Key::R))
    {  
        if (hasBeenReleased)
        {
            hasBeenReleased = false;
            useKeyboardMouse = !useKeyboardMouse;
        }
    }
    else
    {
        hasBeenReleased = true;
    }

    return useKeyboardMouse;
}

void RobotController::updateWithController()
{
    if (updateInputTimer.execute())
    {
        double temp = right_stick_horz * YAW_TURNING_PROPORTIONAL;
        driveTrainEncoder = turretController->getYawEncoderValue();

        switch (leftSwitchState)
        {
            case (tap::communication::serial::Remote::SwitchState::UP):
                // Left Switch is up. So need to beyblade at fast speed, and let right stick control
                // turret yaw and pitch
                targetYawAngleWorld += temp;
                targetDTVelocityWorld = (FAST_BEYBLADE_FACTOR * MAX_SPEED);
                yawEncoderCache = driveTrainEncoder;
                break;
            case (tap::communication::serial::Remote::SwitchState::MID):
                targetYawAngleWorld += temp;
                targetDTVelocityWorld = (SLOW_BEYBLADE_FACTOR * MAX_SPEED);
                yawEncoderCache = driveTrainEncoder;
                // Left Switch is mid. So need to beyblade at slow speed, and let right stick
                // control turret yaw and pitch
                break;
            case (tap::communication::serial::Remote::SwitchState::DOWN):
                // Left Switch is down. So need to not beyblade, and let right stick be decided on
                // the right switch value
                switch (rightSwitchState)
                {
                    case (tap::communication::serial::Remote::SwitchState::MID):
                        // Left switch is down, and right is mid. So move turret independently of
                        // drivetrain
                        targetYawAngleWorld += temp;
                        targetDTVelocityWorld = 0;
                        yawEncoderCache = driveTrainEncoder;

                        break;
                    case (tap::communication::serial::Remote::SwitchState::DOWN):
                        yawEncoderCache = 3 * PI / 4;
                    case (tap::communication::serial::Remote::SwitchState::UP):
                        // Left switch is down, and right is up. So driveTrainFollows Turret
                        targetYawAngleWorld =
                            yawAngleRelativeWorld + (yawEncoderCache - driveTrainEncoder);
                        targetDTVelocityWorld = right_stick_horz * MAX_SPEED * TURNING_CONSTANT;
                        break;
                    default:
                        // Should not be in this state. So if we are, just tell robot to do nothing.
                        stopRobot();
                        break;
                }
                break;
            default:
                // Should not be in this state. So if we are, just tell robot to do nothing.
                stopRobot();
                break;
        }
    }
    if(wheelValue > 0.3){
        shooterController->enableShooting();
        shooterController->setIndexer(0.5);
    } else {
        if(wheelValue < -0.3){
            shooterController->disableShooting();
        }
        shooterController->setIndexer(0);
    }
    targetYawAngleWorld = fmod(targetYawAngleWorld, 2 * PI);
    driveTrainController->moveDriveTrain(
        targetDTVelocityWorld,
        (leftStickMagnitude * MAX_SPEED),
        driveTrainEncoder + leftStickAngle);
    turretController->turretMove(
        targetYawAngleWorld,
        0.1 * PI * right_stick_vert - 0.5 * PI,
        driveTrainRPM,
        yawAngleRelativeWorld,
        yawRPM,
        dt);
    shooterController->setMotorSpeeds();

}

void RobotController::updateWithMouseKeyboard()
{
    if (updateInputTimer.execute())
    {

        //beyblade
        static bool rHasBeenReleased = true; //r sets fast 
        static bool fHasBeenReleased = true; //f sets slow 
        static bool cHasBeenReleased = true; //c sets off 
        static double currentBeybladeFactor = 0;

        if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::R)) { 
            if (rHasBeenReleased){
                rHasBeenReleased = false;
                currentBeybladeFactor = FAST_BEYBLADE_FACTOR;
            }
        } else{
            rHasBeenReleased = true;
        }

        if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::F)){ 
            if (fHasBeenReleased){
                fHasBeenReleased = false;
                currentBeybladeFactor = SLOW_BEYBLADE_FACTOR;
            }
        } else {
            fHasBeenReleased = true;
        }
        
        if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::C)){ 
            if (cHasBeenReleased){
                cHasBeenReleased = false;
                currentBeybladeFactor = 0;
            }
        } else {
            cHasBeenReleased = true;
        }

        if(currentBeybladeFactor!=0)
            targetDTVelocityWorld = (currentBeybladeFactor * MAX_SPEED);
        else{
            targetDTVelocityWorld=0;
            if(drivers->remote.keyPressed(tap::communication::serial::Remote::Key::Q)){ //rotate left
                targetDTVelocityWorld -= (SLOW_BEYBLADE_FACTOR * MAX_SPEED);
            }
            if(drivers->remote.keyPressed(tap::communication::serial::Remote::Key::E)){ //rotate right
                targetDTVelocityWorld += (SLOW_BEYBLADE_FACTOR * MAX_SPEED);
            }
        }


        //movement
        int moveHorizonal = 0;
        int moveVertical = 0;

        if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::W))
            moveVertical++;
        if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::A))
            moveHorizonal--;
        if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::S))
            moveVertical--;
        if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::D))
            moveHorizonal++;

        double moveAngle = getAngle(moveHorizonal, moveVertical);
        double moveMagnitude = getMagnitude(moveHorizonal, moveVertical);

        if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::CTRL))  // slow
            moveMagnitude *= SLOW_SPEED;
        else if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::SHIFT))  // fast
            moveMagnitude *= FAST_SPEED;
        else   // medium
            moveMagnitude *= MED_SPEED;
    
        driveTrainEncoder = turretController->getYawEncoderValue();
        yawEncoderCache = driveTrainEncoder;

        driveTrainController->moveDriveTrain(targetDTVelocityWorld, moveMagnitude, driveTrainEncoder + moveAngle); 

        static int mouseXOffset = drivers->remote.getMouseX(), mouseYOffset = drivers->remote.getMouseY();
        static bool shoot = false;
        static double targetPitch = 0, targetYaw = 0;

        int mouseX = drivers->remote.getMouseX() - mouseXOffset, mouseY = drivers->remote.getMouseY() - mouseYOffset;

        // if(drivers->remote.getMouseL()){
        //     shoot = true;

        // if(drivers->remote.getMouseR()){
        //      if(!jetsonCommunication.hasRead()) {                                
        //         jetsonCommunication.iReadData();
        //         //Need to add the angle from the jetsonCommunicator to our current angle and tell our turret to go to that ()
        //         targetPitch = turretController->getPitchEncoderValue() - 0.5 * PI  + jetsonCommunication.getMsg()->pitch;
        //         targetYaw = turretController->getYawEncoderValue() + jetsonCommunication.getMsg()->yaw;
                
        //     }
        //     shoot = jetsonCommunication.getMsg()->shoot;
        // } else {
        //     shoot = drivers->remote.getMouseL();
        //     targetPitch += mouseY / 10000.0;
        //     targetYaw -= mouseX / 10000.0;
        // }
         if(drivers->remote.getMouseL()){
            shooterController->setIndexer(0.8);
            shooterController->enableShooting();
        } else if(drivers->remote.keyPressed(tap::communication::serial::Remote::Key::Z)){
            shooterController->disableShooting();
            shooterController->setIndexer(-0.1);
        } else {
            shooterController->setIndexer(0);
        }
        shooterController->setMotorSpeeds();

        targetPitch += mouseY / 10000.0;
        targetYaw -= mouseX / 10000.0;


        if(targetPitch > 0.4) targetPitch=0.4;
        if(targetPitch < -0.3) targetPitch=-0.3;
        targetYaw = fmod(targetYaw, 2 * PI);


        // if(shoot){ 
        //     shooterController->setIndexer(0.8);
        //     shooterController->enableShooting();
        // } else if(drivers->remote.keyPressed(tap::communication::serial::Remote::Key::Z)){
        //     shooterController->disableShooting();
        //     shooterController->setIndexer(-0.1);
        // } else {
        //     shooterController->setIndexer(0);
        // }
        //shooterController->setMotorSpeeds();
        // mouse
        turretController->turretMove(targetYaw, targetPitch - 0.5 * PI, driveTrainRPM, yawAngleRelativeWorld, yawRPM, dt);

        
    }
    }
}  // namespace ThornBots 
