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
#include "modm/processing/protothread.hpp"
#include <cstdint>

#include "tap/architecture/timeout.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"



namespace ThornBots {

    // the class UI extends Resumable, and any public stuff in Resumable becomes protected here
    // it also extends RefSerialData
    class UI : protected modm::Resumable<2>, protected tap::communication::serial::RefSerialData, ::modm::pt::Protothread {
    public:  // Public Variables
        tap::communication::serial::RefSerialTransmitter* refSerialTransmitter;

    private:  // Private Variables
        tap::Drivers* drivers;
        bool restarting;
        int nextName = 0;

    public:  // Public Methods
        UI(tap::Drivers* driver);
        ~UI() {}  // Intentionally left blank

        /*
         * Call this function once, outside of the main loop.
         * Sends both things that change and those that don't.
         */
        bool initialize();

        /*
         * Call this function once, outside of the main loop.
         * Sends both things that change and those that don't.
         */
        void restartHud();

        /**
         * Resets the graphic name generator so the next time it is queried via `getUnusedGraphicName`,
         * the function returns {0, 0, 0}.
         */
        inline void resetGraphicNameGenerator() {nextName = 0;};

        inline const uint8_t * getNextGrapicName() {(const uint8_t *) nextName++;};


        /*
         * Should be called within the main loop maybe (There is a limit to how much you can send, maybe this will be handled here). Sends things that
         * change.
         */
        modm::ResumableResult<bool> update();

        
        //these were overrides, not sure what overriding
        void execute();

        void end(bool) {};

        bool isFinished() const { return false; };

    private:  // Private Methods
              // int getPitchVoltage(double targetAngle, double dt);
        
        // initialize() calls this. Sends the grapics for the first time.
        modm::ResumableResult<bool> sendInitial();

        

        
        bool run();

        
        // bool run();
    };
}  // namespace ThornBots