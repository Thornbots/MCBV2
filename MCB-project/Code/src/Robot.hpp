#pragma once

#include "subsystems/drivetrain/DriveTrainSystem.hpp"
#include "subsystems/gimbal/GimbalSystem.hpp"
#include "subsystems/shooter/ShooterSystem.hpp"
#include "subsystems/shooter/HeroShooterSystem.hpp"
#include "subsystems/shooter/ShooterInterface.hpp"

#include "communication/CommunicationsHandler.hpp"

namespace ThornBots{
class Robot
{
private:
    DriveTrainSystem *drivetrainSystem;
    GimbalSystem *gimbalSystem;
    ShooterInterface *shooterSystem;

    CommunicationsHandler *communicationsHandler;

public:
    Robot(/* args */);
    ~Robot();
};

} // namespace ThornBots