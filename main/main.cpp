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



void setup()
{

    tone(BUZZER_PIN, 2700, 60);
    delay(80);
    tone(BUZZER_PIN, 2700, 60);

    #ifdef IS_CAM
    Wire.setPins(I2C_SDA, I2C_SCL);
    Wire.begin();
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
