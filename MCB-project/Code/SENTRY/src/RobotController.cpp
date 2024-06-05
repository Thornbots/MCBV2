#include "RobotController.h"

#include <cmath>

#include <bits/stdc++.h>

namespace ThornBots {

    double currentHeat, maxHeat, theHeatRatio, theLevel = 0.0;
    double yawEncoderValue, IMUAngle = 0.0;
    /*
     * Constructor for RobotController
     */
    RobotController::RobotController(tap::Drivers* driver, ThornBots::DriveTrainController* driveTrainController,
                                     ThornBots::TurretController* turretController, ThornBots::ShooterController* shooterController,
                                     ThornBots::JetsonCommunication* jetsonCommunication)
        : drivers(driver),
          driveTrainController(driveTrainController),
          turretController(turretController),
          shooterController(shooterController),
          jetsonCommunication(jetsonCommunication) {
        // this->drivers = driver;
        // this->driveTrainController = driveTrainController;
        // this->turretController = turretController;
        // this->shooterController = shooterController;
        // this->jetsonCommunication = jetsonCommunication;
    }

    void RobotController::initialize() {
        Board::initialize();
        drivers->can.initialize();
        drivers->bmi088.initialize(500, 0.0, 0.0);
        drivers->bmi088.requestRecalibration();
        drivers->remote.initialize();
        driveTrainController->initialize();
        turretController->initialize();
        shooterController->initialize();
        drivers->refSerial.initialize();
        jetsonCommunication->initialize();
        drivers->leds.init();

        modm::delay_ms(2500);  // Delay 2.5s to allow the IMU to turn on and get working before we move it around
        // TODO: Finish this (Add creating timers, maybe some code to setup the IMU and make sure it's
        // reading correctly, ect)
        /// imuOffset = turretController->getYawEncoderValue();
        targetYawAngleWorld += yawAngleRelativeWorld;
    }

    void RobotController::update() {
        drivers->canRxHandler.pollCanData();

        drivers->refSerial.updateSerial();

        // === blinky led code ===
        static int led_timmer = 500;
        if (led_timmer <= 0) {
            static bool led_state = false;
            drivers->leds.set(tap::gpio::Leds::Green, led_state);
            led_state = !led_state;
            led_timmer = 500;
        }
        led_timmer--;
        // =======================

        jetsonCommunication->updateSerial();

        updateAllInputVariables();

        toggleKeyboardAndMouse();

        // to know if match has started
        bool matchHasStarted = drivers->refSerial.getGameData().gameStage == tap::communication::serial::RefSerial::Rx::GameStage::IN_GAME;

        if (drivers->remote.isConnected())
            enableRobot();
        else
            disableRobot();

        if (robotDisabled) return;
        if (useKeyboardMouse) {
            // shooterController->enableShooting();
            updateWithMouseKeyboard();
        } else {
            // shooterController->enableShooting();
            updateWithController();
        }

        if (drivers->remote.isConnected()) {
            if (motorsTimer.execute()) {
                driveTrainController->setMotorSpeeds();
                turretController->setMotorSpeeds();
                shooterController->setMotorSpeeds();
            }
        } else {
            shooterController->disableShooting();
            stopRobot();
        }

        // drivers->djiMotorTxHandler.encodeAndSendCanData();  // Processes these motor speed changes
        // into can signal
    }

    void RobotController::stopRobot() {
        driveTrainController->stopMotors();
        turretController->stopMotors();
        shooterController->stopMotors();
        robotDisabled = true;
    }

    void RobotController::disableRobot() {
        stopRobot();
        driveTrainController->disable();
        turretController->disable();
        shooterController->disable();
    }

    void RobotController::enableRobot() {
        robotDisabled = false;
        driveTrainController->enable();
        turretController->enable();
        shooterController->enable();
    }

    void RobotController::updateAllInputVariables() {
        drivers->remote.read();  // Reading the remote before we check if it is connected yet or not.
        if (IMUTimer.execute()) {
            drivers->bmi088.periodicIMUUpdate();
        }

        // START Updating stick values
        // Actually Reading from remote
        rightSwitchState = drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH);
        leftSwitchState = drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH);
        left_stick_horz = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL);
        left_stick_vert = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
        right_stick_horz = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);
        right_stick_vert = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
        // Turning the remote raw values into values we can use more easily (circular cordinates)
        leftStickAngle = getAngle(left_stick_horz, left_stick_vert);
        rightStickAngle = getAngle(right_stick_horz, right_stick_vert);
        leftStickMagnitude = getMagnitude(left_stick_horz, left_stick_vert);
        rightStickMagnitude = getMagnitude(right_stick_horz, right_stick_vert);
        // STOP Updating stick values

        driveTrainRPM = 0;  // TODO: get this. Either power from DT motors, using yaw encoder and IMU,
                            // or something else
        yawRPM = PI / 180 * drivers->bmi088.getGz();
        yawAngleRelativeWorld = fmod(PI / 180 * drivers->bmi088.getYaw(), 2 * PI);

        wheelValue = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::WHEEL);
    }

    double RobotController::getAngle(double x, double y) {
        // error handling to prevent runtime errors in atan2
        if (x == 0 && y == 0) {
            return ((double)0.0);
        }

        return atan2(y, x);  // Return (double) [pi, pi] which we want. Doing x/y to rotate the unit
                             // circle 90 degrees CCW (make 0 straight ahead)
    }

    double RobotController::getMagnitude(double x, double y) { return sqrt(pow(x, 2) + pow(y, 2)); }

    bool RobotController::toggleKeyboardAndMouse() {
        // only gets set to false the first time this funtion is called
        static bool hasBeenReleased = true;

        if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::B)) {  // B is furthest. Change later.
            if (hasBeenReleased) {
                hasBeenReleased = false;
                useKeyboardMouse = !useKeyboardMouse;
            }
        } else {
            hasBeenReleased = true;
        }

        return useKeyboardMouse;
    }

    void RobotController::updateWithJetson() {
        ThornBots::JetsonCommunication::cord_msg* msg = jetsonCommunication->getMsg();

        constexpr float omega_scaled = 1;
        constexpr float theta_scaled = 1;

        targetYawAngleWorld += msg->omega * omega_scaled;
        targetDTVelocityWorld = 0;
        yawEncoderCache = driveTrainEncoder;

        constexpr double yaw_min = PI * 5 / 6;
        constexpr double yaw_max = PI * 7 / 6;
        targetYawAngleWorld = std::clamp(targetYawAngleWorld, yaw_min, yaw_max);  // TODO: remove
        targetYawAngleWorld = fmod(targetYawAngleWorld, 2 * PI);

        targetPitchAngleWorld += msg->theta * theta_scaled;
        constexpr double min = 0.02;
        constexpr double max = 0.523;
        targetPitchAngleWorld = std::clamp(targetPitchAngleWorld, min, max);

        turretController->turretMove(targetYawAngleWorld, targetPitchAngleWorld, driveTrainRPM, yawAngleRelativeWorld, yawRPM, dt);
    }

    void RobotController::updateWithController() {
        if (updateInputTimer.execute()) {
            double right_stick_scaled = right_stick_horz * YAW_TURNING_PROPORTIONAL;
            driveTrainEncoder = turretController->getYawEncoderValue();

            switch (leftSwitchState) {
                case (tap::communication::serial::Remote::SwitchState::UP):
                    // replace with auto mode: wait until start and then start beyblading

                    // Left Switch is up. So need to beyblade at fast speed, and let right stick control
                    // turret yaw and pitch
                    targetYawAngleWorld += right_stick_scaled;
                    targetDTVelocityWorld = (FAST_BEYBLADE_FACTOR * MAX_SPEED);
                    yawEncoderCache = driveTrainEncoder;
                    break;
                case (tap::communication::serial::Remote::SwitchState::MID):
                    targetYawAngleWorld += right_stick_scaled;
                    targetDTVelocityWorld = (SLOW_BEYBLADE_FACTOR * MAX_SPEED);
                    yawEncoderCache = driveTrainEncoder;
                    // Left Switch is mid. So need to beyblade at slow speed, and let right stick
                    // control turret yaw and pitch
                    break;
                case (tap::communication::serial::Remote::SwitchState::DOWN):
                    // Left Switch is down. So need to not beyblade, and let right stick be decided on
                    // the right switch value
                    switch (rightSwitchState) {
                        case (tap::communication::serial::Remote::SwitchState::MID):
                            // Left switch is down, and right is mid. So move turret independently of
                            // drivetrain
                            {
                                targetYawAngleWorld += right_stick_scaled;
                                targetDTVelocityWorld = 0;
                                yawEncoderCache = driveTrainEncoder;

                                break;
                            }
                        case (tap::communication::serial::Remote::SwitchState::DOWN):
                            updateWithJetson();  // TODO: idk get better system
                            return;
                            yawEncoderCache = 3 * PI / 4;
                            // no break intentional so if in this case it also runs what is below
                        case (tap::communication::serial::Remote::SwitchState::UP):
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
            tap::communication::serial::RefSerialData::Rx::RobotData robotData = drivers->refSerial.getRobotData();
            tap::communication::serial::RefSerialData::Rx::TurretData turretData = robotData.turret;
            double frequency1 = 15, frequency2 = 15, latency = 0.4, remaining1 = turretData.heatLimit - turretData.heat17ID1,
                   remaining2 = turretData.heatLimit - turretData.heat17ID2;
            // Check if the firing rate should be limited
            if ((10.0 * frequency1 - turretData.coolingRate) * latency > remaining1) {
                // Set the firing speed to C/10 Hz
                frequency1 = turretData.coolingRate / 10.0;
            }
            if ((10.0 * frequency2 - turretData.coolingRate) * latency > remaining2) {
                // Set the firing speed to C/10 Hz
                frequency2 = turretData.coolingRate / 10.0;
            }
            if (frequency1 > 15) frequency1 = 15;
            if (frequency2 > 15) frequency2 = 15;
            if (wheelValue < -0.5) {
                shooterController->enableShooting();
                shooterController->setIndexer1(frequency1 / 20.0);
                shooterController->setIndexer2(frequency2 / 20.0);
            } else {
                if (wheelValue > 0.5) {
                    shooterController->disableShooting();
                    shooterController->setIndexers(-0.1);
                } else
                    shooterController->setIndexers(0);
            }

            shooterController->setIndexer2((robotData.allRobotHp.red.standard4) / 400.0);

            targetYawAngleWorld = fmod(targetYawAngleWorld, 2 * PI);

            constexpr double yaw_min = PI * 5 / 6;
            constexpr double yaw_max = PI * 7 / 6;
            targetYawAngleWorld = std::clamp(targetYawAngleWorld, yaw_min, yaw_max);  // TODO: remove

            driveTrainController->moveDriveTrain(targetDTVelocityWorld, (leftStickMagnitude * MAX_SPEED), driveTrainEncoder + leftStickAngle);
            turretController->turretMove(targetYawAngleWorld,
                                         0.1 * PI * right_stick_vert + 0.07 * PI,  // + PI,  //was - 0.5 * PI
                                         driveTrainRPM, yawAngleRelativeWorld, yawRPM, dt);

            // idk if this should still work
            //  tap::communication::serial::RefSerialData::Rx::RobotData robotData = drivers->refSerial.getRobotData();
            //  auto& turretData = robotData.turret;
            //  uint8_t level = robotData.robotLevel;
            //  //turretData.h
            //  double heatRatio = (((double)turretData.heat17ID1)/turretData.heatLimit);

            // currentHeat = turretData.heat17ID1;
            // maxHeat = turretData.heatLimit;
            // theHeatRatio = heatRatio;
            // theLevel = level;

            // shooterController->enableShooting();
            // shooterController->setIndexer(theLevel/10.0);
        }
    }

    void RobotController::updateWithMouseKeyboard() {
        if (updateInputTimer.execute()) {
            tap::communication::serial::RefSerialData::Rx::RobotData robotData = drivers->refSerial.getRobotData();
            tap::communication::serial::RefSerialData::Rx::TurretData turretData = robotData.turret;
            double frequency1 = 15, frequency2 = 15, latency = 50, remaining1 = turretData.heatLimit - (turretData.heat17ID1 + 10),
                   remaining2 = turretData.heatLimit - (turretData.heat17ID2 + 10);
            // Check if the firing rate should be limited
            if ((10.0 * frequency1 - turretData.coolingRate) * latency > remaining1) {
                // Set the firing speed to C/10 Hz
                frequency1 = turretData.coolingRate / 10.0;
            }
            if ((10.0 * frequency2 - turretData.coolingRate) * latency > remaining2) {
                // Set the firing speed to C/10 Hz
                frequency2 = turretData.coolingRate / 10.0;
            }
            if (frequency1 > 15) frequency1 = 15;
            if (frequency2 > 15) frequency2 = 15;
            if (wheelValue < -0.5) {
                shooterController->enableShooting();
                shooterController->setIndexer1(frequency1 / 20.0);
                shooterController->setIndexer2(frequency2 / 20.0);
            } else {
                if (wheelValue > 0.5) {
                    shooterController->disableShooting();
                    shooterController->setIndexers(-0.1);
                } else
                    shooterController->setIndexers(0);
            }

            shooterController->setIndexer2(0);

            // beyblade
            static bool rHasBeenReleased = true;  // r sets fast
            static bool fHasBeenReleased = true;  // f sets slow
            static bool cHasBeenReleased = true;  // c sets off
            static double currentBeybladeFactor = 0;

            if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::R)) {
                if (rHasBeenReleased) {
                    rHasBeenReleased = false;
                    currentBeybladeFactor = FAST_BEYBLADE_FACTOR;
                }
            } else {
                rHasBeenReleased = true;
            }

            if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::F)) {
                if (fHasBeenReleased) {
                    fHasBeenReleased = false;
                    currentBeybladeFactor = SLOW_BEYBLADE_FACTOR;
                }
            } else {
                fHasBeenReleased = true;
            }

            if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::C)) {
                if (cHasBeenReleased) {
                    cHasBeenReleased = false;
                    currentBeybladeFactor = 0;
                }
            } else {
                cHasBeenReleased = true;
            }

            if (currentBeybladeFactor != 0)
                targetDTVelocityWorld = (-currentBeybladeFactor * MAX_SPEED);
            else {
                targetDTVelocityWorld = 0;
                if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::Q)) {  // rotate left
                    targetDTVelocityWorld -= (SLOW_BEYBLADE_FACTOR * MAX_SPEED);
                }
                if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::E)) {  // rotate right
                    targetDTVelocityWorld += (SLOW_BEYBLADE_FACTOR * MAX_SPEED);
                }
            }

            // movement
            int moveHorizonal = 0;
            int moveVertical = 0;

            if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::W)) moveVertical++;
            if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::A)) moveHorizonal--;
            if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::S)) moveVertical--;
            if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::D)) moveHorizonal++;

            double moveAngle = getAngle(moveHorizonal, moveVertical);
            double moveMagnitude = getMagnitude(moveHorizonal, moveVertical);

            if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::CTRL))  // slow
                moveMagnitude *= SLOW_SPEED;
            else if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::SHIFT))  // fast
                moveMagnitude *= FAST_SPEED;
            else  // medium
                moveMagnitude *= MED_SPEED;

            driveTrainEncoder = turretController->getYawEncoderValue();
            yawEncoderCache = driveTrainEncoder;

            driveTrainController->moveDriveTrain(targetDTVelocityWorld, moveMagnitude,
                                                 driveTrainEncoder + moveAngle);  // driveTrainEncoder + moveAngle - 3 * PI / 4);
            // also try targetYawAngleWorld, yawEncoderCache

            // mouse
            static int mouseXOffset = drivers->remote.getMouseX();
            static int mouseYOffset = drivers->remote.getMouseY();
            int mouseX = drivers->remote.getMouseX() - mouseXOffset;
            int mouseY = drivers->remote.getMouseY() - mouseYOffset;
            static double accumulatedMouseY = 0;
            accumulatedMouseY += mouseY / 10000.0;

            if (accumulatedMouseY > 0.4) accumulatedMouseY = 0.4;
            if (accumulatedMouseY < -0.3) accumulatedMouseY = -0.3;

            targetYawAngleWorld -= mouseX / 10000.0;

            targetYawAngleWorld = fmod(targetYawAngleWorld, 2 * PI);
            turretController->turretMove(targetYawAngleWorld,
                                         (accumulatedMouseY) + 0.07 * PI,  // + PI,
                                         driveTrainRPM, yawAngleRelativeWorld, yawRPM, dt);
        }
    }
}  // namespace ThornBots
