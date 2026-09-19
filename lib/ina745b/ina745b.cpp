#include "ina745b.h"

ina745b::ina745b(uint8_t addr, TwoWire *i2c)
{
    _i2c_addr = addr;
    _i2c = i2c;
}

void ina745b::init(){
    uint8_t reset_word = [0x80, 0x00];
    write_register(CONFIG_ADDR, &reset_word, 2);
}

int read_voltage(){
    return read_register(VBUS_ADDR, 2);
}

int ina745b::read_register(uint8_t reg_addr, uint8_t bytes){
    Wire.beginTransmission(_i2c_addr);
    Wire.write(reg_addr);
    if(Wire.endTransmission()){
        Serial.println("Current sensor: I2C Error");
    }
    Wire.requestFrom(_i2c_addr, bytes);
    int val = 0;
    for(int i = 0; i < bytes; i++){
        int v = Wire.read();
        if(v == -1) Serial.println("Current sensor: I2C Read Error");
        val = (val << 8) | v;
    }
    return val;
}

void ina745b::write_register(uint8_t addr, uint8_t * data, uint8_t len){
    Wire.beginTransmission(_i2c_addr);
    Wire.write(reg_addr);
    for(int i = len-1; i >= 0; i++){
        Wire.write(data[i]);
    }
    
    if(Wire.endTransmission()){
        Serial.println("Current sensor: I2C Write Error");
    }
}