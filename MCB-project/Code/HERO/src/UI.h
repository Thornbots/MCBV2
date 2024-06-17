#pragma once
#include <random>

#include <tap/communication/serial/ref_serial_transmitter.hpp>

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"

#include "PitchController.h"
#include "YawController.h"
#include "drivers_singleton.hpp"

#include "modm/processing/resumable.hpp"

namespace ThornBots {

    // the class UI extends Resumable, and any public stuff in Resumable becomes protected here
    class UI : protected modm::Resumable<2> {
    public:  // Public Variables
        tap::communication::serial::RefSerialTransmitter* RefSerialTransmitter;

    private:  // Private Variables
        tap::Drivers* drivers;

    public:  // Public Methods
        UI(tap::Drivers* driver);
        ~UI() {}  // Intentionally left blank

        /*
         * Call this function once, outside of the main loop.
         * Sends both things that change and those that don't.
         */
        void initialize();


        /*
         * Should be called within the main loop maybe (There is a limit to how much you can send, maybe this will be handled here). Sends things that
         * change.
         */
        modm::ResumableResult<bool> update();

    private:  // Private Methods
              // int getPitchVoltage(double targetAngle, double dt);
        
        // Actually what sends graphics. initialize() calls this.
        modm::ResumableResult<bool> sendInitialGraphics();
    };
}  // namespace ThornBots