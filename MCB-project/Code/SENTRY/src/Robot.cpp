#include "Robot.h"

#include <stdio.h>

#include <cmath>

#include <bits/stdc++.h>

#include "Print.hpp"


namespace ThornBots {

    double currentHeat, maxHeat, theHeatRatio, theLevel = 0.0;
    double yawEncoderValue, IMUAngle = 0.0;
    /*
     * Constructor for Robot
     */
    Robot::Robot(tap::Drivers* driver, DrivetrainSubsystem* driveTrainSubsystem, GimbalSubsystem* gimbalSubsystem, ShooterSubsystem* shooterSubsystem,
                 JetsonCommunication* jetsonCommunication) {
        this->drivers = driver;
        this->drivetrainSubsystem = driveTrainSubsystem;
        this->gimbalSubsystem = gimbalSubsystem;
        this->shooterSubsystem = shooterSubsystem;
        this->jetsonCommunication = jetsonCommunication;
    }

    void Robot::initialize() {
        Board::initialize();
        drivers->can.initialize();
        drivers->bmi088.initialize(500, 0.0, 0.0);
        drivers->bmi088.requestRecalibration();
        drivers->remote.initialize();
        drivetrainSubsystem->initialize();
        gimbalSubsystem->initialize();
        shooterSubsystem->initialize();
        drivers->refSerial.initialize();
        jetsonCommunication->initialize();
        drivers->leds.init();

        modm::delay_ms(2500);  // Delay 2.5s to allow the IMU to turn on and get working before we move it around
        // TODO: Finish this (Add creating timers, maybe some code to setup the IMU and make sure it's
        // reading correctly, ect)
        /// imuOffset = turretController->getYawEncoderValue();
        targetYawAngleWorld += yawAngleRelativeWorld;
    }

    void Robot::update() {
        drivers->canRxHandler.pollCanData();

        drivers->refSerial.updateSerial();

        // === blinky led code ===
        static int led_timmer = 0;
        if (led_timmer <= 0) {
            static bool led_state = false;
            drivers->leds.set(tap::gpio::Leds::Green, led_state);
            led_state = !led_state;
            led_timmer = 800;
        }
        led_timmer--;
        // =======================

        jetsonCommunication->updateSerial();

        updateAllInputVariables();

        // to know if match has started

        if (drivers->remote.isConnected())
            enableRobot();
        else
            disableRobot();

        if (robotDisabled) return;

        switch (currentProgram) {
            case MANUAL:
                updateWithController();
                break;
            case SPIN:
                updateWithSpin();
                break;
            case SHOOT:
                // updateWithSpin();
                updateWithCV();
                break;
        }

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
        yawAngleRelativeWorld = fmod(PI / 180 * drivers->bmi088.getYaw(), 2 * PI);

        wheelValue = drivers->remote.getChannel(Remote::Channel::WHEEL);

        switch (rightSwitchState) {
            case Remote::SwitchState::UP:
                currentProgram = SHOOT;
                break;
            case Remote::SwitchState::DOWN:
                currentProgram = SPIN;
                break;
            default:
                currentProgram = MANUAL;
                break;
        }
    }

    // haha shooty funny
    void Robot::updateWithCV() {
        if (cvTimer.execute()) {
            ThornBots::JetsonCommunication::cord_msg* msg = jetsonCommunication->getMsg();

            AutoAim::GimbalCommand command = autoAim.update(msg->x, msg->y, msg->z, gimbalSubsystem->getPitchEncoderValue(), gimbalSubsystem->getYawEncoderValue());
            if(command.action != -1 && msg->confidence > 0.1){
                targetYawAngleWorld = command.yaw;
                targetPitchAngleWorld = std::clamp(command.pitch, static_cast<double>(-0.3), static_cast<double>(0.3));  // TODO: remove

            }
            if (leftSwitchState == Remote::SwitchState::UP) {
                if (command.action == 1) {
                    shooterSubsystem->shoot(20);
                } else {
                    shooterSubsystem->idle();
                }
            } else {
                shooterSubsystem->disableShooting();
            }
        }
        gimbalSubsystem->turretMove(targetYawAngleWorld, targetPitchAngleWorld, driveTrainRPM, yawAngleRelativeWorld, yawRPM, dt);

    }
    // literally just spin
    void Robot::updateWithSpin() { drivetrainSubsystem->moveDriveTrain(SLOW_BEYBLADE_FACTOR * MAX_SPEED, 0, 0); }

    void Robot::updateWithController() {
        if (updateInputTimer.execute()) {
            double right_stick_scaled = right_stick_horz * YAW_TURNING_PROPORTIONAL;
            driveTrainEncoder = gimbalSubsystem->getYawEncoderValue();

            switch (leftSwitchState) {
                case (Remote::SwitchState::UP):
                    // replace with auto mode: wait until start and then start beyblading

                    // Left Switch is up. So need to beyblade at fast speed, and let right stick control
                    // turret yaw and pitch
                    targetYawAngleWorld += right_stick_scaled;
                    targetDTVelocityWorld = (FAST_BEYBLADE_FACTOR * MAX_SPEED);
                    yawEncoderCache = driveTrainEncoder;
                    break;
                case (Remote::SwitchState::MID):
                    targetYawAngleWorld += right_stick_scaled;
                    targetDTVelocityWorld = (SLOW_BEYBLADE_FACTOR * MAX_SPEED);
                    yawEncoderCache = driveTrainEncoder;
                    // Left Switch is mid. So need to beyblade at slow speed, and let right stick
                    // control turret yaw and pitch
                    break;
                case (Remote::SwitchState::DOWN):
                    // Left Switch is down. So need to not beyblade,

                    targetYawAngleWorld += right_stick_scaled;
                    targetDTVelocityWorld = 0;
                    yawEncoderCache = driveTrainEncoder;

                    break;
                default:
                    // Should not be in this state. So if we are, just tell robot to do nothing.
                    stopRobot();
                    break;
            }

            if (wheelValue < -0.5) {
                shooterSubsystem->shoot(20);
            } else if (wheelValue > 0.5) {
                shooterSubsystem->unjam();
            } else {
                shooterSubsystem->idle();
            }

            targetYawAngleWorld = fmod(targetYawAngleWorld, 2 * PI);

            drivetrainSubsystem->moveDriveTrain(targetDTVelocityWorld, (leftStickMagnitude * MAX_SPEED), driveTrainEncoder + leftStickAngle);
            gimbalSubsystem->turretMove(targetYawAngleWorld,
                                        0.1 * PI * right_stick_vert,  // + PI,  //was - 0.5 * PI
                                        driveTrainRPM, yawAngleRelativeWorld, yawRPM, dt);
        }
    }

}  // namespace ThornBots
