#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <pins.h>


static void STARTUP_SONG(void *)
{
    
    vTaskDelete(nullptr);
}