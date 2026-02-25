#pragma once
#include <Arduino.h>
#include "pins.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "USB.h"
#include "USBCDC.h"

#include "uvc.h"


// System file for EAGLE
// This file must expose `sys_begin()`, which will create all freeRTOS threads needed for the board.

/* Begin all system functions, including init. */
[[noreturn]] void sys_begin();

struct EAGLESystems {
    USBCDC* serial;




};