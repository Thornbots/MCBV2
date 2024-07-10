#include "UI.h"


namespace ThornBots {
    UI::UI(tap::Drivers* driver) { this->drivers = driver; }

    bool UI::initialize() {
        // Nothing needs to be done to drivers
        refSerialTransmitter = new tap::communication::serial::RefSerialTransmitter(drivers);


        // if (!this->isRunning())
        // {
        //     // Restart the thread
        //     restart();
        //     // Reset the HUD elements
        //     // this->restartHud();
        // }
        
        // PT_BEGIN();

        // PT_WAIT_UNTIL(drivers->refSerial.getRefSerialReceivingData());

        // PT_CALL(sendInitial());

        // PT_END();


        // We cannot reset the thread from here because there might be locked
        // resources that we need to finish first.
        restarting = true;
    }

    

void UI::restartHud()
{
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

bool UI::run()
{
    // The thread has exited the loop, meaning that there are no locked resources
    if (!this->isRunning())
    {
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
    while (!this->restarting)
    {
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


    modm::ResumableResult<bool> UI::sendInitial()
    {
        // RF_BEGIN(0);
        
        //  Example code:
        // Graphic1Message msg;
        // RefSerialTransmitter::configGraphicGenerics(&msg.graphicData, "\x00\x00\x01", RefSerial::GRAPHIC_ADD, 1, YELLOW);
        // RefSerialTransmitter::configLine(4, 100, 100, 200, 200, &msg.graphicData);
        // refSerialTransmitter->sendGraphic(&msg);

        //   For sending graphics, the general schema is to create a `Graphic<n>Message` struct, configure
        //   the individual `GraphicData` structs in the graphic message using the `configGraphicGenerics`
        //   and then `config<Line|Rectangle|Circle|etc.>` functions. Finally, send the graphic message
        //   using `sendGraphic`.

        tap::communication::serial::RefSerialTransmitter::Tx::Graphic1Message* msg =
            new tap::communication::serial::RefSerialTransmitter::Tx::Graphic1Message();

        refSerialTransmitter->configGraphicGenerics(&msg->graphicData, (const uint8_t *)(1),
                                                    tap::communication::serial::RefSerialTransmitter::Tx::GRAPHIC_ADD, 1,
                                                    tap::communication::serial::RefSerialTransmitter::Tx::GraphicColor::ORANGE);

        refSerialTransmitter->configLine(50, 0, 0, 1920, 1080, &msg->graphicData);
        // refSerialTransmitter->configLine(50, 0, 200, 300, 400, &msg->graphicData);
        
        // RF_CALL(refSerialTransmitter->sendGraphic(msg));
        refSerialTransmitter->sendGraphic(msg);
        

        // RF_END();
    }

    modm::ResumableResult<bool> UI::update()
    {
        RF_BEGIN(1);
        RF_END();
    }
}  // namespace ThornBots
