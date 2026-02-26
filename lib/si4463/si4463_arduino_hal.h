#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <pins.h>
#include <si4463.h>

using si4463_result_t = uint8_t;

static const SPISettings SI_SPI_SETTINGS(8000000, MSBFIRST, SPI_MODE0);  // 8 MHz (SI4463 max is 10MHz)
static SPIClass* SI_SPI = nullptr;

inline void si_cs_low() { digitalWrite(SI4463_CS, LOW); }
inline void si_cs_high() { digitalWrite(SI4463_CS, HIGH); }
inline void si_sdn_low() { digitalWrite(SI4463_SDN, LOW); }
inline void si_sdn_high() { digitalWrite(SI4463_SDN, HIGH); }
inline uint8_t si_nirq_asserted() { return digitalRead(SI4463_INT) == LOW ? 1 : 0; }
inline void si_delay_ms(uint32_t ms) { delay(ms); }
inline void si_delay_us(uint32_t us) { delayMicroseconds(us); }
inline uint8_t si_cts() { return digitalRead(SI4463_GPIO1) == HIGH ? 1 : 0; }
inline uint8_t si_tx_active() { return digitalRead(SI4463_GPIO0) == HIGH ? 1 : 0; }

inline void si_spi_xfer(const uint8_t* tx, uint8_t* rx, const uint16_t n) {
  SI_SPI->beginTransaction(SI_SPI_SETTINGS);
  for (size_t i = 0; i < n; i++) {
    uint8_t out = tx ? tx[i] : 0x00;
    uint8_t in  = SI_SPI->transfer(out);
    if (rx) rx[i] = in;
  }
  SI_SPI->endTransaction();
}

inline void si_hal_arduino_init() {
  // Inits the SI pins for arduino.
  pinMode(SI4463_CS, OUTPUT);
  pinMode(SI4463_SDN, OUTPUT);
  pinMode(SI4463_INT, INPUT_PULLUP);
  pinMode(SI4463_GPIO0, INPUT);
  pinMode(SI4463_GPIO1, INPUT);

  digitalWrite(SI4463_SDN, LOW);
  digitalWrite(SI4463_CS, HIGH);
}

inline void si_hal_bind_arduino(si4463_t* si) {
  si->Select = si_cs_low;
  si->Deselect = si_cs_high;
  si->WriteRead = si_spi_xfer;
  si->ClearShutdown = si_sdn_low;
  si->SetShutdown = si_sdn_high;
  si->DelayMs = si_delay_ms;
  si->DelayUs = si_delay_us;
  si->IsClearToSend = si_cts;
  si->IsNirqAsserted = si_nirq_asserted;
  si->IsTxActive = si_tx_active;
}

inline void si_hal_set_spi(SPIClass& spi) {
    SI_SPI = &spi;
}