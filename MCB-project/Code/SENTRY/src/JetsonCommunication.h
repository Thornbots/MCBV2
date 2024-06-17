#pragma once
#include "tap/communication/serial/dji_serial.hpp"
#include "tap/communication/serial/uart.hpp"

#include "drivers_singleton.hpp"

struct cord_msg;

namespace tap {
    class Drivers;
}

namespace ThornBots {
    class JetsonCommunication : public tap::communication::serial::DJISerial {
    private:
        bool hasBeenRead = false;
        bool led_state = false;

    public:
        // using tap::communication::serial::DJISerial::DJISerial;

        JetsonCommunication(tap::Drivers *drivers, tap::communication::serial::Uart::UartPort port = tap::communication::serial::Uart::Uart1,
                            bool isRxCRCEnforcementEnabled = true)
            : DJISerial(drivers, port, isRxCRCEnforcementEnabled){};

        struct cord_msg
        {
            float x=0; // meters
            float y=0; // meters
            float z=0; // meters
            float v_x=0; // m/s
            float v_y=0; // m/s
            float v_z=0; // m/s
            float a_x=0; // m/s^2
            float a_y=0; // m/s^2
            float a_z=0; // m/s^2
            float confidence=0; // 0.0 to 1.0
        }; //TODO: use modm_packed?

        //TODO: make better scheme
        struct encoder_msg{
            uint8_t header = 0xA5;
            uint16_t data_len = 0; //little endian
            // uint8_t data_type = 0;
            double current_pitch;
            uint8_t newline = 0x0A; // newline '\n'
        } modm_packed;


        /*
        run this to read from the uart queue buffer to get the coordinates of the panal
        */
        void messageReceiveCallback(const ReceivedSerialMessage &completeMessage) {
            cord_msg *m = (cord_msg *)completeMessage.data;
            memcpy((void*)&message, (void*)m, sizeof(cord_msg));
            // message = *(cord_msg*)completeMessage.data;
            hasBeenRead = false;
            // drivers->leds.set(tap::gpio::Leds::Blue, true);
            led_state = !led_state;
        };

        //TODO: try with value copy?
        cord_msg *getMsg() { 
            if (hasBeenRead) return &empty_msg;
            // drivers->leds.set(tap::gpio::Leds::Blue, false);
            hasBeenRead = true;
            return &message;
        };
        bool newMessage(){
            return !hasBeenRead;
        }

        void send(struct encoder_msg* msg){
            tap::communication::serial::Uart::UartPort port = tap::communication::serial::Uart::Uart1;
            
            uint8_t *msg_to_send = reinterpret_cast<uint8_t*>(msg);
            // drivers->uart.write(port, 0xA5);
            drivers->uart.write(port, msg_to_send, sizeof(encoder_msg));
            // drivers->uart.write(port, '\n');
            // drivers->uart.write(port, '\0');

            // constexpr int msg_len = 7;
            // char buf[msg_len];
            // sprintf(buf,"theta:");
            // uint8_t* print_msg = reinterpret_cast<uint8_t*>(buf);
            // drivers->uart.write(tap::communication::serial::Uart::Uart1, print_msg, msg_len);
        }

    private:
        cord_msg message = {0.0, 0.0, 0.0,0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0};
        cord_msg empty_msg = {0.0, 0.0, 0.0,0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0};
    };

}  // namespace ThornBots
