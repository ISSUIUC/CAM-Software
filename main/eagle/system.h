#pragma once
#include <Arduino.h>

#include "USB.h"
#include "USBCDC.h"
#include "cam_radio/cam_radio.h"
#include "eagle/uvc.h"

#define USE_USB_DEBUG

// System file for EAGLE
// This file must expose `sys_begin()`, which will create all freeRTOS threads needed for the board.

/* Begin all system functions, including init. */
[[noreturn]] void sys_begin();

struct EAGLESystems {
    CAMRadio radio;
    #ifdef USE_USB_DEBUG
    USBCDC* serial;
    #endif
};