#include <Wire.h>
#include <Arduino.h>

#define CONFIG_ADDR 0x0
#define ADC_CONFIG_ADDR 0x1
#define VBUS_ADDR 0x5
#define TEMP_ADDR 0x6
#define CURRENT_ADDR 0x7
#define POWER_ADDR 0x8

#define VBUS_CONVERSION_FACTOR 3.125 // 3.125 mV per LSB
#define CURRENT_CONVERSION_FACTOR 1.2 // 1.2 mA per LSB
#define POWER_CONVERSION_FACTOR 240 // 240 µW per LSB

class ina745b {
public:
    ina745b(uint8_t addr);
    void init();

    int read_raw_voltage();
    int read_raw_current();
    int read_raw_power();

    double read_voltage();
    double read_current();
    double read_power();

private:
    uint8_t _i2c_addr;

    int read_register(uint8_t reg_addr, uint8_t bytes);
    int write_register(uint8_t reg_addr, uint8_t * data, uint8_t len);

};