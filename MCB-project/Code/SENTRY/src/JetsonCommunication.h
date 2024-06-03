#pragma once
#include "drivers_singleton.hpp"
#include "tap/communication/serial/dji_serial.hpp"
#include "tap/communication/serial/uart.hpp"

struct cord_msg;

namespace tap
{
class Drivers;
}

namespace ThornBots {
class JetsonCommunication: public tap::communication::serial::DJISerial{
    private:
        bool hasBeenRead = false;
        // src::Drivers* drivers;
        bool led_state = false;


    public:
        // using tap::communication::serial::DJISerial::DJISerial;

        JetsonCommunication(
            tap::Drivers *drivers,
            tap::communication::serial::Uart::UartPort port=tap::communication::serial::Uart::Uart1,
            bool isRxCRCEnforcementEnabled = true):
             DJISerial(drivers,port,isRxCRCEnforcementEnabled){};

        struct cord_msg
        {
            float theta=0; // rad
            float omega=0; // rad
            float confidence; // 0.0 to 1.0
            float panel_dist; // meters
        };


        /*
        run this to read from the uart queue buffer to get the coordinates of the panal
        */
        void messageReceiveCallback(const ReceivedSerialMessage &completeMessage)
        {
            message = (cord_msg*)completeMessage.data;
            hasBeenRead = false;
            drivers->leds.set(tap::gpio::Leds::Blue, true);
            led_state = !led_state;
        };

        cord_msg* getMsg(){
            if (hasBeenRead) return &empty_msg;
            drivers->leds.set(tap::gpio::Leds::Blue, false);
            hasBeenRead = true;
            return message;
        };

        // void sendToJetson(uint8_t){


        // }

    private:
        cord_msg *message;
        cord_msg empty_msg = {0.0,0.0,0.0,0.0};
    };

}
