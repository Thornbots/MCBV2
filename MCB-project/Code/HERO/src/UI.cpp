#include "UI.h"

namespace ThornBots {
    UI::UI(tap::Drivers* driver) { this->drivers = driver; }

    void UI::initialize() {
        // Nothing needs to be done to drivers
        RefSerialTransmitter = new tap::communication::serial::RefSerialTransmitter(drivers);

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

        RefSerialTransmitter->configGraphicGenerics(&msg->graphicData, (const uint8_t *)(1),
                                                    tap::communication::serial::RefSerialTransmitter::Tx::GRAPHIC_ADD, 1,
                                                    tap::communication::serial::RefSerialTransmitter::Tx::GraphicColor::WHITE);

        RefSerialTransmitter->configLine(4, 100, 100, 200, 200, &msg->graphicData);
        RefSerialTransmitter->configLine(4, 200, 200, 300, 400, &msg->graphicData);

        RefSerialTransmitter->sendGraphic(msg);
    }

}  // namespace ThornBots
