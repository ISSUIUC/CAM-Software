#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "startmusic.h"

#include <pins.h>
#include <SPI.h>
#include <Wire.h>

#ifdef IS_CAM
#include <cam/system.h>
#endif

#ifdef IS_EAGLE
#include <eagle/system.h>
#endif

// note lengths (milliseconds for 113 BPM)
#define sixteenth 133
#define eighth 266
#define dotted_eighth 399
#define quarter 532
#define dotted_quarter 798
#define half 1064
#define whole 2128

// note frequencies (Hz)
#define A4_flat 415
#define B4_flat 466
#define C5      523
#define D5_flat 554
#define E5_flat 622
#define F5      699


void setup()
{

    // tone(BUZZER_PIN, 2700, 60);
    // delay(80);
    // tone(BUZZER_PIN, 2700, 60);

    // Rick Astley
    tone(BUZZER_PIN, A4_flat, sixteenth);
    tone(BUZZER_PIN, B4_flat, sixteenth);
    tone(BUZZER_PIN, D5_flat, sixteenth);
    tone(BUZZER_PIN, B4_flat, sixteenth);
    tone(BUZZER_PIN, F5, dotted_eighth);
    tone(BUZZER_PIN, F5, dotted_eighth);
    tone(BUZZER_PIN, E5_flat, dotted_quarter);
    tone(BUZZER_PIN, A4_flat, sixteenth);
    tone(BUZZER_PIN, B4_flat, sixteenth);
    tone(BUZZER_PIN, C5, sixteenth);
    tone(BUZZER_PIN, A4_flat, sixteenth);
    tone(BUZZER_PIN, E5_flat, dotted_eighth);
    tone(BUZZER_PIN, E5_flat, dotted_eighth);
    tone(BUZZER_PIN, D5_flat, dotted_eighth);
    tone(BUZZER_PIN, C5, sixteenth);
    tone(BUZZER_PIN, B4_flat, eighth);


    #ifdef IS_CAM
    Wire.setPins(I2C_SDA, I2C_SCL);
    Wire.begin();
    #endif

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    pinMode(LED_RED, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_ORANGE, OUTPUT);

    pinMode(BUZZER_PIN, OUTPUT);

    if (xTaskCreate(STARTUP_SONG, "STARTUP_SONG", 1024, nullptr, 1, nullptr) != pdPASS)
    {
        digitalWrite(LED_RED, HIGH);
    }

    sys_begin();
}

void loop()
{}
