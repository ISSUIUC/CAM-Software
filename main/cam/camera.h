#pragma once

#include <HardwareSerial.h>
#include <pins.h>
#include <errors.h>

#define CAM_STATE_ON true
#define CAM_STATE_OFF false

struct read_mem_cap_data_return {
    uint8_t buf[32];
    size_t mem_size;
    uint8_t status;
};

struct Runcam {
    private:
    uint8_t _pwr_pin;
    uint8_t _cam_tx;
    uint8_t _cam_rx;
    HardwareSerial* _uart;
    bool _cam_on;
    
    public:
    Runcam(HardwareSerial* uart, uint8_t power_pin, uint8_t rx, uint8_t tx) {
        _pwr_pin = power_pin;
        _cam_tx = tx;
        _cam_rx = rx;
        _uart = uart;

        pinMode(_pwr_pin, OUTPUT);
        digitalWrite(_pwr_pin, LOW);
    }

    size_t write(const uint8_t* buf, size_t len) { if(_cam_on) { return _uart->write(buf, len); } return 0; };
    void flush() { if(_cam_on) { _uart->flush(); } };
    int read(uint8_t* buf, size_t size) { if(_cam_on) { return _uart->read(buf, size); } return 0; };
    int available() { if(_cam_on) { return _uart->available(); } return 0; }
    void set_state(bool power_on);
};

struct Cameras {
    Runcam cam1{&Serial1, CAM1_ON_OFF, CAM1_RX, CAM1_TX};
    Runcam cam2{&Serial2, CAM2_ON_OFF, CAM2_RX, CAM2_TX};
};

void camera_on_off(Runcam& camera);
void start_recording(Runcam& camera);
void stop_recording(Runcam& camera);
uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a);
uint8_t generate_crc(uint8_t* buf, unsigned int buf_len);
bool check_crc(uint8_t* buf, unsigned int buf_len, uint8_t expected_crc);
struct read_mem_cap_data_return read_mem_cap_data(Runcam& camera);
