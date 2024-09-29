#include "DrivetrainSubsystem.h"
#include "GimbalSubsystem.h"
#include "Robot.h"
#include "ShooterSubsystem.h"
#include "drivers_singleton.hpp"

src::Drivers *drivers;
static tap::arch::PeriodicMicroTimer RunTimer(10);  // Don't ask me why. This only works as a global. #Certified Taproot Moment

int main() {
    src::Drivers *drivers = src::DoNotUse_getDrivers();
    ThornBots::DrivetrainSubsystem *drivetrainSubsystem = new ThornBots::DrivetrainSubsystem(drivers);
    ThornBots::GimbalSubsystem *gimbalSubsystem = new ThornBots::GimbalSubsystem(drivers);
    ThornBots::ShooterSubsystem *shooterSubsystem = new ThornBots::ShooterSubsystem(drivers);
    ThornBots::UI *ui = new ThornBots::UI(drivers);

    ThornBots::Robot *robot = new ThornBots::Robot(drivers, drivetrainSubsystem, gimbalSubsystem, shooterSubsystem, ui);

    robot->initialize();
    while (1) {
        if (RunTimer.execute()) {  // Calling this function every 10 us at max
            robot->update();
        }
    }
}