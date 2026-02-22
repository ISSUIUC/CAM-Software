#pragma once
#include <Arduino.h>
#include <cam/tvpcontroller.h>
#include <cam/camera.h>
#include <cam/b2b.h>

#include "USB.h"
#include "USBCDC.h"

// System file for CAM
// This file must expose `sys_begin()`, which will create all freeRTOS threads needed for the board.

/* Begin all system functions, including init. */
[[noreturn]] void sys_begin();

struct CAMSystems {
    TVPController tvp;
    Cameras cameras;
    B2BHandler b2b;
    USBCDC* serial;
};