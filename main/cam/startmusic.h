#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <pins.h>

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

static void STARTUP_SONG(void *)
{
    // Rick Astley
    tone(BUZZER_PIN, A4_flat, sixteenth);
    vTaskDelay(pdMS_TO_TICKS(sixteenth));
    tone(BUZZER_PIN, B4_flat, sixteenth);
    vTaskDelay(pdMS_TO_TICKS(sixteenth));
    tone(BUZZER_PIN, D5_flat, sixteenth);
    vTaskDelay(pdMS_TO_TICKS(sixteenth));
    tone(BUZZER_PIN, B4_flat, sixteenth);
    vTaskDelay(pdMS_TO_TICKS(sixteenth));
    tone(BUZZER_PIN, F5, dotted_eighth);
    vTaskDelay(pdMS_TO_TICKS(dotted_eighth));
    tone(BUZZER_PIN, F5, dotted_eighth);
    vTaskDelay(pdMS_TO_TICKS(dotted_eighth));
    tone(BUZZER_PIN, E5_flat, dotted_quarter);
    vTaskDelay(pdMS_TO_TICKS(dotted_quarter));
    tone(BUZZER_PIN, A4_flat, sixteenth);
    vTaskDelay(pdMS_TO_TICKS(sixteenth));
    tone(BUZZER_PIN, B4_flat, sixteenth);
    vTaskDelay(pdMS_TO_TICKS(sixteenth));
    tone(BUZZER_PIN, C5, sixteenth);
    vTaskDelay(pdMS_TO_TICKS(sixteenth));
    tone(BUZZER_PIN, A4_flat, sixteenth);
    vTaskDelay(pdMS_TO_TICKS(sixteenth));
    tone(BUZZER_PIN, E5_flat, dotted_eighth);
    vTaskDelay(pdMS_TO_TICKS(dotted_eighth));
    tone(BUZZER_PIN, E5_flat, dotted_eighth);
    vTaskDelay(pdMS_TO_TICKS(dotted_eighth));
    tone(BUZZER_PIN, D5_flat, dotted_eighth);
    vTaskDelay(pdMS_TO_TICKS(dotted_eighth));
    tone(BUZZER_PIN, C5, sixteenth);
    vTaskDelay(pdMS_TO_TICKS(sixteenth));
    tone(BUZZER_PIN, B4_flat, eighth);
    vTaskDelay(pdMS_TO_TICKS(eighth));

    vTaskDelete(nullptr);
}