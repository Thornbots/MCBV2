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
    class UI : protected modm::Resumable<2> { //protected tap::communication::serial::RefSerialData,
    public:  // Public Variables

    private:  // Private Variables
        tap::Drivers* drivers;
        tap::control::CommandScheduler* commandScheduler;
        tap::communication::serial::RefSerialTransmitter* refSerialTransmitter;
        bool restarting;
        int nextName = 0;
        uint32_t currGraphicName;

    public:  // Public Methods
        UI(tap::Drivers* drivers);
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
        inline void resetGraphicNameGenerator() {currGraphicName = 0;};

        /**
         * Graphics must have unique 3 byte names. Utility function for getting a graphic name that is
         * currently unused. Use this function exclusively to avoid graphic name clashes.
         *
         * If no list names are available (all are in use), won't set the graphicName and will return false.
         *
         * @param[out] graphicName Array to put an unused list name in.
         */
        bool getUnusedGraphicName(uint8_t graphicName[3]);


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