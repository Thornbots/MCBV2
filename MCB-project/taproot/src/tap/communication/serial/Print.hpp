#ifndef TAPROOT_PRINT_HPP_
#define TAPROOT_PRINT_HPP_

// #include "drivers_singleton.hpp"
#include "tap/drivers.hpp"
#include "tap/communication/serial/uart.hpp"
#include <cstdint>

#define RX_BUF_SIZE 256

namespace tap
{
    class Drivers;
} // namespace tap



namespace tap::communication::serial::print
{
class UartPrint{
private:
    char buff[RX_BUF_SIZE];
public:
    UartPrint(Drivers* drivers): drivers(drivers){};
    ~UartPrint() = default;

    void init();
    // void printf(const char *__restrict __fmt, ...);
    void printf(const char *__fmt);

private:
    Drivers* drivers;

};
    
} // namespace ThronBots

#endif  // TAPROOT_PRINT_HPP_