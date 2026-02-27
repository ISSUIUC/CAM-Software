#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <pins.h>
#include <SPI.h>
#include <Wire.h>

#ifdef IS_CAM
#include <cam/system.h>
#endif

#ifdef IS_EAGLE
#include <eagle/system.h>
#endif

#define CORE_0 0
#define CORE_1 1

void setup()
{
    #ifdef IS_CAM
    Wire.begin(I2C_SDA, I2C_SCL);
    #endif

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    pinMode(LED_RED, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_ORANGE, OUTPUT);

    sys_begin();
}

void loop()
{}