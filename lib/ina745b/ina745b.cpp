#include "ina745b.h"

ina745b::ina745b(uint8_t addr)
{
    _i2c_addr = addr;
}

void ina745b::init() {
    uint8_t reset_word = [0x80, 0x00];
    write_register(CONFIG_ADDR, &reset_word, 2);
}

int ina745b::read_raw_voltage() {
    return read_register(VBUS_ADDR, 2);
}

int ina745b::read_raw_current() {
    return read_register(CURRENT_ADDR, 2);
}

int ina745b::read_raw_power() {
    return read_register(POWER_ADDR, 3);
}


double ina745b::read_voltage() {
    return VBUS_CONVERSION_FACTOR * read_raw_voltage();
}

double ina745b::read_current() {
    return CURRENT_CONVERSION_FACTOR * read_raw_current();
}

double ina745b::read_power() { 
    return POWER_CONVERSION_FACTOR * read_raw_power();
}


int ina745b::read_register(uint8_t reg_addr, uint8_t bytes) {
    Wire.beginTransmission(_i2c_addr);
    Wire.write(reg_addr);
    if(Wire.endTransmission()){
        Serial.println("Current sensor: I2C Error");
    }
    Wire.requestFrom(_i2c_addr, bytes);
    int val = 0;
    for(int i = 0; i < bytes; i++) {
        int v = Wire.read();
        if(v == -1) Serial.println("Current sensor: I2C Read Error");
        val = (val << 8) | v;
    }
    return val;
}

// @brief Writes data to a register on the INA745B device
// @param reg_addr: The address of the register to write to
// @param data: Pointer to the data to write
// @param len: Length of the data to write
void ina745b::write_register(uint8_t reg_addr, uint8_t * data, uint8_t len){
    Wire.beginTransmission(_i2c_addr);
    Wire.write(reg_addr);
    for(int i = len-1; i >= 0; i++){
        Wire.write(data[i]);
    }
    
    if(Wire.endTransmission()){
        Serial.println("Current sensor: I2C Write Error");
    }
}