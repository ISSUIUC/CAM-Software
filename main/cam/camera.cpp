#ifdef IS_CAM

#include <HardwareSerial.h>
#include <cam/camera.h>

//https://support.runcam.com/hc/en-us/articles/360014537794-RunCam-Device-Protocol

int Cameras::init() {
    cam1->begin(115200, SERIAL_8N1, CAM1_RX, CAM1_TX);
    cam2->begin(115200, SERIAL_8N1, CAM2_RX, CAM2_TX);

    pinMode(CAM1_ON_OFF, OUTPUT);
    digitalWrite(CAM1_ON_OFF, LOW);
    pinMode(CAM2_ON_OFF, OUTPUT);
    digitalWrite(CAM2_ON_OFF, LOW);

    return CAM_OK;
}

void camera_on_off(HardwareSerial& camera) {
    uint8_t arr[4] = {0xCC, 0x01, 0x01, 0xE7};
    camera.write(arr, 4);
    camera.flush();
}

void start_recording(HardwareSerial& camera) {
    uint8_t arr[4] = {0xCC, 0x01, 0x03, 0x98};
    camera.write(arr, 4);
    camera.flush();
}

void stop_recording(HardwareSerial& camera) {
    uint8_t arr[4] = {0xCC, 0x01, 0x04, 0xCC};
    camera.write(arr, 4);
    camera.flush();
}

uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a) {
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

uint8_t generate_crc(uint8_t* buf, unsigned int buf_len) {
    uint8_t crc = 0x00;
    for(unsigned i = 0; i < buf_len; i++) {
        crc = crc8_dvb_s2(crc, buf[i]);
    }
    return crc;
}

bool check_crc(uint8_t* buf, unsigned int buf_len, uint8_t expected_crc) {
    return generate_crc(buf, buf_len) == expected_crc;
}

static size_t read_memory_number_from_buf(uint8_t* buf) {
    uint8_t* ptr = buf + 0;
    size_t value = 0;
    while (*ptr != '/' && *ptr != '\0') {
        if (*ptr >= '0' && *ptr <= '9') {
            value = value * 10 + (*ptr - '0');
        }
        ptr++;
    }
    return value;
}

struct read_mem_cap_data_return read_mem_cap_data(HardwareSerial& camera) {
    uint8_t get_setting_raw[4] = {0xCC, 0x11, 0x03, 0x00};
    uint8_t get_setting[5] = {0xCC, 0x11, 0x03, 0x00, generate_crc(get_setting_raw, 4)};
    camera.write(get_setting, 5);
    delay(750);

    struct read_mem_cap_data_return toReturn;
    toReturn.status = 0;

    if(camera.available()) {
        camera.read(toReturn.buf, 4);
        uint8_t msg_len = toReturn.buf[2] - 1;

        camera.read(toReturn.buf, msg_len);

        if(msg_len >= 7 && msg_len <= 17) {
            toReturn.mem_size = read_memory_number_from_buf(toReturn.buf);
            toReturn.status = 1;
            return toReturn;
        }
    }
    return toReturn;
}

#endif
