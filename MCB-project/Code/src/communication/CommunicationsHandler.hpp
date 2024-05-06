#pragma once
#include "ControllerInput.hpp"
#include "KeyboardInput.hpp"
#include "JetsonInput.hpp"

namespace ThornBots{
class CommunicationsHandler
{
private:
    KeyboardInput *keyboard;
    ControllerInput *controller;
    JetsonInput *jetson;
public:
    CommunicationsHandler(/* args */);
    ~CommunicationsHandler();
};
} // namespace ThornBots