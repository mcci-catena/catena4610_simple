/*

Module: Catena4610_cMeasurementLoop.cpp

Function:
    Class for transmitting accumulated measurements.

Copyright:
    See accompanying LICENSE file for copyright and license information.

Author:
    Rama subbu, MCCI Corporation   june 2021

*/

#include <Catena_TxBuffer.h>

#include "Catena4610_cMeasurementLoop.h"

#include <arduino_lmic.h>

using namespace McciCatena;
using namespace McciCatena4610;

/*

Name:   McciCatena4610::cMeasurementLoop::fillTxBuffer()

Function:
    Prepare a messages in a TxBuffer with data from current measurements.

Definition:
    void McciCatena4610::cMeasurementLoop::fillTxBuffer(
            cMeasurementLoop::TxBuffer_t& b
            );

Description:
    A format 0x22 message is prepared from the data in the cMeasurementLoop
    object.

*/

void
cMeasurementLoop::fillTxBuffer(
    cMeasurementLoop::TxBuffer_t& b, Measurement const &mData
    )
    {
    gLed.Set(McciCatena::LedPattern::Measuring);


    // initialize the message buffer to an empty state
    b.begin();

    // insert format byte
    b.put(kMessageFormat);

    // the flags in Measurement correspond to the over-the-air flags.
    b.put(std::uint8_t(this->m_data.flags));

    // send Vbat
    if (((this->m_data.flags) & (Flags::FlagVbat))!= Flags(0)) 
        {
        float Vbat = mData.Vbat;
        gCatena.SafePrintf("Vbat:    %d mV\n", (int) (Vbat * 1000.0f));
        b.putV(Vbat);
        }

    // send Vdd if we can measure it.

    // Vbus is sent as 5000 * v
    float Vbus = mData.Vbus;
    gCatena.SafePrintf("Vbus:    %d mV\n", (int) (Vbus * 1000.0f));

    // send boot count
    if (((this->m_data.flags) & (Flags::FlagBoot))!= Flags(0))
        {
        b.putBootCountLsb(mData.BootCount);
        }

    if (((this->m_data.flags) & (Flags::FlagTPH))!= Flags(0))
        {
        gCatena.SafePrintf(
                "BME280:  T: %d P: %d RH: %d\n",
                (int) mData.env.Temperature,
                (int) mData.env.Pressure,
                (int) mData.env.Humidity
                );
        b.putT(mData.env.Temperature);
        b.putP(mData.env.Pressure);
        // no method for 2-byte RH, directly encode it.
        b.put2uf((mData.env.Humidity / 100.0f) * 65535.0f);
        }

    // put light
    if (((this->m_data.flags) & (Flags::FlagLux))!= Flags(0))
        {
        gCatena.SafePrintf(
                "Si1133:  %u White\n",
                mData.light.White
                );

        b.putLux(mData.light.White);
        }

    gLed.Set(McciCatena::LedPattern::Off);
    }
