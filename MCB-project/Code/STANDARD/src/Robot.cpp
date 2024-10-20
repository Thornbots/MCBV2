#include "Robot.h"

#include <cmath>
namespace ThornBots {

    double currentHeat, maxHeat, theHeatRatio, theLevel = 0.0;
    double yawEncoderValue, IMUAngle = 0.0;
    /*
     * Constructor for Robot
     */
    Robot::Robot(tap::Drivers* driver, DrivetrainSubsystem* driveTrainSubsystem, GimbalSubsystem* gimbalSubsystem,
                 ShooterSubsystem* shooterSubsystem, UI* ui) {
        this->drivers = driver;
        this->drivetrainSubsystem = driveTrainSubsystem;
        this->gimbalSubsystem = gimbalSubsystem;
        this->shooterSubsystem = shooterSubsystem;
        this->ui = ui;
    }

    void Robot::initialize() {
        Board::initialize();
        drivers->can.initialize();
        drivers->bmi088.initialize(500, 0.0, 0.0);
        drivers->bmi088.requestRecalibration();
        drivers->remote.initialize();
        drivers->pwm.init();  // For the servo we will be using
        drivetrainSubsystem->initialize();
        gimbalSubsystem->initialize();
        shooterSubsystem->initialize();
        drivers->refSerial.initialize();

        modm::delay_ms(2500);  // Delay 2.5s to allow the IMU to turn on and get working before we move it around
        // TODO: Finish this (Add creating timers, maybe some code to setup the IMU and make sure it's
        // reading correctly, ect)
        imuOffset = gimbalSubsystem->getYawEncoderValue();
        targetYawAngleWorld += yawAngleRelativeWorld;
    }

    bool Robot::update2(){
        PT_BEGIN();

        PT_WAIT_UNTIL(drivers->refSerial.getRefSerialReceivingData());

        PT_CALL(ui->update());
        PT_YIELD();
        PT_END();
    }
    void Robot::update() {
        update2();
        
        drivers->canRxHandler.pollCanData();
        drivers->refSerial.updateSerial();

        updateAllInputVariables();
        toggleKeyboardAndMouse();

        if (drivers->remote.isConnected())
            enableRobot();
        else
            disableRobot();

        if (robotDisabled) return;

        if (useKeyboardMouse)
            updateWithMouseKeyboard();
        else
            updateWithController();

        if (motorsTimer.execute()) {
            drivetrainSubsystem->setMotorSpeeds();
            gimbalSubsystem->setMotorSpeeds();
            shooterSubsystem->setMotorSpeeds();
        }

        // into can signal
    }

    void Robot::updateAllInputVariables() {
        drivers->remote.read();  // Reading the remote before we check if it is connected yet or not.
        if (IMUTimer.execute()) {
            drivers->bmi088.periodicIMUUpdate();
        }
        // START Updating stick values
        // Actually Reading from remote
        rightSwitchState = drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH);
        leftSwitchState = drivers->remote.getSwitch(Remote::Switch::LEFT_SWITCH);
        left_stick_horz = drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL);
        left_stick_vert = drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL);
        right_stick_horz = drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL);
        right_stick_vert = drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL);
        // Turning the remote raw values into values we can use more easily (circular cordinates)
        leftStickAngle = getAngle(left_stick_horz, left_stick_vert);
        rightStickAngle = getAngle(right_stick_horz, right_stick_vert);
        leftStickMagnitude = getMagnitude(left_stick_horz, left_stick_vert);
        rightStickMagnitude = getMagnitude(right_stick_horz, right_stick_vert);
        // STOP Updating stick values

        driveTrainRPM = 0;  // TODO: get this. Either power from DT motors, using yaw encoder and IMU,
                            // or something else
        yawRPM = PI / 180 * drivers->bmi088.getGz();
        yawAngleRelativeWorld = fmod(PI / 180 * drivers->bmi088.getYaw() - imuOffset, 2 * PI);

        wheelValue = drivers->remote.getChannel(Remote::Channel::WHEEL);
    }

    bool Robot::toggleKeyboardAndMouse() {
        if (keyJustPressed(Remote::Key::B))  // B is furthest. Change later.
            useKeyboardMouse = !useKeyboardMouse;

        return useKeyboardMouse;
    }
    void Robot::updateWithController() {
        if (updateInputTimer.execute()) {
            double temp = right_stick_horz * YAW_TURNING_PROPORTIONAL;
            driveTrainEncoder = gimbalSubsystem->getYawEncoderValue();

            switch (leftSwitchState) {
                case (Remote::SwitchState::UP):
                    // Left Switch is up. So need to beyblade at fast speed, and let right stick control
                    // turret yaw and pitch
                    targetYawAngleWorld += temp;
                    targetDTVelocityWorld = (FAST_BEYBLADE_FACTOR * MAX_SPEED);
                    yawEncoderCache = driveTrainEncoder;
                    break;
                case (Remote::SwitchState::MID):
                    targetYawAngleWorld += temp;
                    targetDTVelocityWorld = (SLOW_BEYBLADE_FACTOR * MAX_SPEED);
                    yawEncoderCache = driveTrainEncoder;
                    // Left Switch is mid. So need to beyblade at slow speed, and let right stick
                    // control turret yaw and pitch
                    break;
                case (Remote::SwitchState::DOWN):
                    // Left Switch is down. So need to not beyblade, and let right stick be decided on
                    // the right switch value
                    switch (rightSwitchState) {
                        case (Remote::SwitchState::MID):
                            // Left switch is down, and right is mid. So move turret independently of
                            // drivetrain
                            targetYawAngleWorld += temp;
                            targetDTVelocityWorld = 0;
                            yawEncoderCache = driveTrainEncoder;

                            break;
                        case (Remote::SwitchState::DOWN):
                            yawEncoderCache = 3 * PI / 4;
                            // no break intentional so if in this case it also runs what is below
                        case (Remote::SwitchState::UP):
                            // Left switch is down, and right is up. So driveTrainFollows Turret
                            targetYawAngleWorld = yawAngleRelativeWorld + (yawEncoderCache - driveTrainEncoder);
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

            if (wheelValue < -0.5) {
                shooterSubsystem->shoot(15);
            } else if (wheelValue > 0.5) {
                shooterSubsystem->unjam();
            } else {
                shooterSubsystem->idle();
            }

            targetYawAngleWorld = fmod(targetYawAngleWorld, 2 * PI);
            drivetrainSubsystem->moveDriveTrain(targetDTVelocityWorld, (leftStickMagnitude * MAX_SPEED), driveTrainEncoder + leftStickAngle);
            gimbalSubsystem->turretMove(targetYawAngleWorld,
                                        0.1 * PI * right_stick_vert,  // was - 0.5 * PI
                                        driveTrainRPM, yawAngleRelativeWorld, yawRPM, temp / dt, dt);
        }
    }

    void Robot::updateWithMouseKeyboard() {
        if (updateInputTimer.execute()) {
            // shooting
            if (drivers->remote.getMouseL()) {
                if (drivers->refSerial.getGameData().gameType == tap::communication::serial::RefSerial::Rx::GameType::ROBOMASTER_RMUL_3V3 &&
                    drivers->refSerial.getRobotData().robotLevel < 3)
                    shooterSubsystem->shoot(10);
                else
                    shooterSubsystem->shoot(15);
            } else if (drivers->remote.keyPressed(Remote::Key::Z)) {
                shooterSubsystem->unjam();
            } else {
                shooterSubsystem->idle();
            }
            // mouse
            static int mouseXOffset = drivers->remote.getMouseX();
            static int mouseYOffset = drivers->remote.getMouseY();
            int mouseX = drivers->remote.getMouseX() - mouseXOffset;
            int mouseY = drivers->remote.getMouseY() - mouseYOffset;
            static double accumulatedMouseY = 0;
            accumulatedMouseY += mouseY / 10000.0 ;

            if (accumulatedMouseY > 0.4) accumulatedMouseY = 0.4;
            if (accumulatedMouseY < -0.3) accumulatedMouseY = -0.3;

            targetYawAngleWorld -= mouseX / 10000.0;

            targetYawAngleWorld = fmod(targetYawAngleWorld, 2 * PI);
            gimbalSubsystem->turretMove(targetYawAngleWorld, (accumulatedMouseY), driveTrainRPM, yawAngleRelativeWorld, yawRPM, -mouseX / 10000.0 / dt, dt);


            // movement

            if (keyJustPressed(Remote::Key::R)) currentBeybladeFactor = FAST_BEYBLADE_FACTOR;

            if (keyJustPressed(Remote::Key::F)) currentBeybladeFactor = SLOW_BEYBLADE_FACTOR;

            if (keyJustPressed(Remote::Key::C)) currentBeybladeFactor = 0;

            if (currentBeybladeFactor != 0)
                targetDTVelocityWorld = (-currentBeybladeFactor * MAX_SPEED);
            else {
                targetDTVelocityWorld = 0;
                if (drivers->remote.keyPressed(Remote::Key::Q)) {  // rotate left
                    targetDTVelocityWorld -= (SLOW_BEYBLADE_FACTOR * MAX_SPEED);
                }
                if (drivers->remote.keyPressed(Remote::Key::E)) {  // rotate right
                    targetDTVelocityWorld += (SLOW_BEYBLADE_FACTOR * MAX_SPEED);
                }
            }

            int moveHorizonal = 0;
            int moveVertical = 0;

            if (drivers->remote.keyPressed(Remote::Key::W)) moveVertical++;
            if (drivers->remote.keyPressed(Remote::Key::A)) moveHorizonal--;
            if (drivers->remote.keyPressed(Remote::Key::S)) moveVertical--;
            if (drivers->remote.keyPressed(Remote::Key::D)) moveHorizonal++;

            double moveAngle = getAngle(moveHorizonal, moveVertical);
            double moveMagnitude = getMagnitude(moveHorizonal, moveVertical);

            if (drivers->remote.keyPressed(Remote::Key::SHIFT)) {  // fast
                moveMagnitude *= FAST_SPEED;
                drivetrainSubsystem->setHigherPowerLimit();
            } else {  // regular
                moveMagnitude *= REG_SPEED;
                drivetrainSubsystem->setRegularPowerLimit();
            }

            driveTrainEncoder = gimbalSubsystem->getYawEncoderValue();
            yawEncoderCache = driveTrainEncoder;

            targetYawAngleWorld = fmod(targetYawAngleWorld, 2 * PI);

            drivetrainSubsystem->moveDriveTrain(targetDTVelocityWorld, moveMagnitude, driveTrainEncoder + moveAngle);
        }
    }
}  // namespace ThornBots