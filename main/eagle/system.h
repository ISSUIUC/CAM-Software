#pragma once
#include <Arduino.h>

#include "USB.h"
#include "USBCDC.h"

// System file for EAGLE
// This file must expose `sys_begin()`, which will create all freeRTOS threads needed for the board.

/* Begin all system functions, including init. */
[[noreturn]] void sys_begin();

struct EAGLESystems {
    USBCDC* serial;
};