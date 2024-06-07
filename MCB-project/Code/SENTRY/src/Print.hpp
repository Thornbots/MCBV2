#pragma once

#include "tap/communication/serial/uart.hpp"

#include "drivers_singleton.hpp"

#define RX_BUF_SIZE 256

namespace tap {
    class Drivers;
}  // namespace tap

namespace ThornBots {
    class UartPrint {
    private:
        tap::Drivers* drivers;
        char buff[RX_BUF_SIZE];

    public:
        UartPrint(tap::Drivers* drivers) : drivers(drivers) { drivers->uart.init<tap::communication::serial::Uart::UartPort::Uart1, 115200>(); };
        void printf(const char* __restrict __fmt, ...) {
            // sprintf(this->buff,__fmt,);
            strcpy(this->buff, __fmt);
            uint8_t* msg = reinterpret_cast<uint8_t*>(this->buff);
            drivers->uart.write(tap::communication::serial::Uart::Uart1, msg, strlen(__fmt));
        };
    };

}  // namespace ThornBots
