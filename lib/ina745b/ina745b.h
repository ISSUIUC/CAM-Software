#include <Wire.h>
#include <Arduino.h>

#define CONFIG_ADDR 0x0
#define VBUS_ADDR 0x5
#define TEMP_ADDR 0x6
#define CURRENT_ADDR 0x7
#define POWER_ADDR 0x8

class ina745b {
public:
    ina745b(uint8_t addr, TwoWire *i2c);
    void init();

    int read_voltage();

private:
    TwoWire* _i2c;
    uint8_t _i2c_addr;

    int read_register(uint8_t register_addr);

};