/*

Name:   catena4610_simple.h

Function:
    Global linkage for catena4610_simple.ino

Copyright:
    See accompanying LICENSE file for copyright and license information.

Author:
    Rama subbu, MCCI Corporation   june 2021

*/

#ifndef _catena4610_simple_h_
# define _catena4610_simple_h_

#pragma once

#include <Catena.h>
#include <Catena_BootloaderApi.h>
#include <Catena_Download.h>
#include <Catena_Led.h>
#include <Catena_Mx25v8035f.h>
#include <Catena_Timer.h>
#include <SPI.h>
#include "Catena4610_cMeasurementLoop.h"

// the global clock object

extern  McciCatena::Catena                      gCatena;
extern  McciCatena::cTimer                      ledTimer;
extern  McciCatena::Catena::LoRaWAN             gLoRaWAN;
extern  McciCatena::StatusLed                   gLed;

extern  SPIClass                                gSPI2;
extern  McciCatena4610::cMeasurementLoop        gMeasurementLoop;

//   The flash
extern  McciCatena::Catena_Mx25v8035f           gFlash;

// the bootloader
extern  McciCatena::cBootloaderApi              gBootloaderApi;

// the downloaer
extern  McciCatena::cDownload                   gDownload;

#endif // !defined(_catena4610_simple_h_)
