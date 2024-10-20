#include "UI.h"
using namespace tap::communication::serial;
using namespace tap::control;
namespace ThornBots {
    UI::UI(tap::Drivers* drivers) {
        // roughly from tamu
        this->drivers = drivers;
        this->commandScheduler = &drivers->commandScheduler;
        this->refSerialTransmitter = new tap::communication::serial::RefSerialTransmitter(drivers); //this isn't close to what tamu had
    }

    bool UI::initialize() {
        // Nothing needs to be done to drivers
        // refSerialTransmitter = new tap::communication::serial::RefSerialTransmitter(drivers);

        // if (!this->isRunning())
        // {
        //     // Restart the thread
        //     restart();
        //     // Reset the HUD elements
        //     // this->restartHud();
        // }

        // PT_BEGIN();

        // PT_WAIT_UNTIL(drivers->refSerial.getRefSerialReceivingData());
        // PT_CALL(update());
        // //PT_CALL(update());
        //  //this->ptState = 27; __attribute__((fallthrough)); case 27: auto rfResult = (sendInitial()); if (rfResult.getState() > modm::rf::NestingError) { return true; } rfResult.getResult(); 

        
        // PT_END();

        // We cannot reset the thread from here because there might be locked
        // resources that we need to finish first.
       // restarting = true;
    }

    void UI::restartHud() {
        resetGraphicNameGenerator();
        // booleanHudIndicators.initialize();
        // capBankIndicator.initialize();
        // chassisOrientationIndicator.initialize();
        // positionHudIndicators.initialize();
        // reticleIndicator.initialize();
        // visionHudIndicators.initialize();

        // We can successfully restart the thread
        this->restarting = false;
    }

    void UI::execute() { run(); }

    bool UI::run() {
        // The thread has exited the loop, meaning that there are no locked resources
        if (!this->isRunning()) {
            // Restart the thread
            restart();
            // Reset the HUD elements
            this->restartHud();
        }

        PT_BEGIN();

        PT_WAIT_UNTIL(drivers->refSerial.getRefSerialReceivingData());

        // PT_CALL(booleanHudIndicators.sendInitialGraphics());
        // PT_CALL(capBankIndicator.sendInitialGraphics());
        // PT_CALL(chassisOrientationIndicator.sendInitialGraphics());
        // PT_CALL(positionHudIndicators.sendInitialGraphics());
        // PT_CALL(reticleIndicator.sendInitialGraphics());
        // PT_CALL(visionHudIndicators.sendInitialGraphics());

        // If we try to restart the hud, break out of the loop
        while (!this->restarting) {
            // PT_CALL(booleanHudIndicators.update());
            // PT_CALL(capBankIndicator.update());
            // PT_CALL(chassisOrientationIndicator.update());
            // PT_CALL(positionHudIndicators.update());
            // PT_CALL(reticleIndicator.update());
            // PT_CALL(visionHudIndicators.update());
            PT_YIELD();
        }
        // Breaking out of the loop successfully calls this method,
        // allowing us to know that all execution is over.
        PT_END();
    }

    modm::ResumableResult<bool> UI::sendInitial() {
        //RF_BEGIN(0);  //aruw uses this, but this errs when RF_CALL is used, it also won't build

        //  Example code:
        // Graphic1Message msg;
        // RefSerialTransmitter::configGraphicGenerics(&msg.graphicData, "\x00\x00\x01", RefSerial::GRAPHIC_ADD, 1, YELLOW);
        // RefSerialTransmitter::configLine(4, 100, 100, 200, 200, &msg.graphicData);
        // refSerialTransmitter->sendGraphic(&msg);

        //   For sending graphics, the general schema is to create a `Graphic<n>Message` struct, configure
        //   the individual `GraphicData` structs in the graphic message using the `configGraphicGenerics`
        //   and then `config<Line|Rectangle|Circle|etc.>` functions. Finally, send the graphic message
        //   using `sendGraphic`.

        uint8_t graphicName[3];
        getUnusedGraphicName(graphicName);

        tap::communication::serial::RefSerialTransmitter::Tx::Graphic1Message* msg =
            new tap::communication::serial::RefSerialTransmitter::Tx::Graphic1Message();

        refSerialTransmitter->configGraphicGenerics(&msg->graphicData, graphicName, tap::communication::serial::RefSerialTransmitter::Tx::GRAPHIC_ADD,
                                                    1, tap::communication::serial::RefSerialTransmitter::Tx::GraphicColor::ORANGE);

        refSerialTransmitter->configLine(50, 0, 0, 1920, 1080, &msg->graphicData);
        // refSerialTransmitter->configLine(50, 0, 200, 300, 400, &msg->graphicData);

        //RF_CALL(refSerialTransmitter->sendGraphic(msg));
        // refSerialTransmitter->sendGraphic(msg);

       // RF_END();
    }

    modm::ResumableResult<bool> UI::update() {
        RF_BEGIN(1);
        RF_END();
    }

    bool UI::getUnusedGraphicName(uint8_t graphicName[3]) {
        if (currGraphicName > 0xffffff) return false;

        graphicName[0] = static_cast<uint8_t>((currGraphicName >> 16) & 0xff);
        graphicName[1] = static_cast<uint8_t>((currGraphicName >> 8) & 0xff);
        graphicName[2] = static_cast<uint8_t>(currGraphicName & 0xff);
        currGraphicName++;
        return true;
    }
}  // namespace ThornBots
