#include "tap/drivers.hpp"
#include "Print.hpp"

namespace tap::communication::serial::print{
    void UartPrint::init(){
        drivers->uart.init<tap::communication::serial::Uart::UartPort::Uart1, 115200>();

    }
    void UartPrint::printf(const char *__fmt){//const char *__restrict __fmt, ...){
        // sprintf(this->buff,__fmt,);
        strcpy(this->buff, __fmt);
        uint8_t* msg = reinterpret_cast<uint8_t*>(this->buff);
        drivers->uart.write(tap::communication::serial::Uart::Uart1,msg,strlen(__fmt));
    }
}