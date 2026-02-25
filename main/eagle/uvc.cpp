#include "uvc.h"

#ifdef IS_EAGLE


const char *TAG = "uvc_test";

// ---------------------------------------------------------------------------
// Video state
// ---------------------------------------------------------------------------
volatile unsigned frame_num = 0;
volatile unsigned tx_busy = 0;
unsigned interval_ms = 1000 / FRAME_RATE;

// ---------------------------------------------------------------------------
// Frame buffer and color bar
// ---------------------------------------------------------------------------
uint8_t frame_buffer[FRAME_SIZE_BYTES];



// ---------------------------------------------------------------------------
// TinyUSB descriptor callbacks — override weak versions in esp_tinyusb
// ---------------------------------------------------------------------------
extern "C" {

uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&desc_device;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
#if TUD_OPT_HIGH_SPEED
    if (tud_speed_get() == TUSB_SPEED_HIGH) {
        return get_hs_cfg();
    }
#endif
    return (uint8_t const *)&desc_fs_configuration;
}

#if TUD_OPT_HIGH_SPEED
uint8_t const *tud_descriptor_device_qualifier_cb(void) {
    return (uint8_t const *)&desc_device_qualifier;
}

uint8_t const *tud_descriptor_other_speed_configuration_cb(uint8_t index) {
    (void)index;
    if (tud_speed_get() == TUSB_SPEED_HIGH) {
        return (uint8_t const *)&desc_fs_configuration;
    } else {
        return get_hs_cfg();
    }
}
#endif

static uint16_t _desc_str[33];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    size_t chr_count;
    if (index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) return NULL;
        const char *str = string_desc_arr[index];
        if (!str) return NULL;
        chr_count = strlen(str);
        if (chr_count > 32) chr_count = 32;
        for (size_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];
        }
    }
    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}

// ---------------------------------------------------------------------------
// Device mount/umount callbacks — override weak versions in esp_tinyusb
// ---------------------------------------------------------------------------
void tud_mount_cb(void) {
    ESP_LOGI(TAG, "USB mounted!");
}

void tud_umount_cb(void) {
    ESP_LOGI(TAG, "USB unmounted!");
}

} // extern "C"



void fill_color_bar(uint8_t *buffer, unsigned start_position) {
    static const uint8_t bar_color[8][4] = {
        { 235, 128, 235, 128 },
        { 219,  16, 219, 138 },
        { 188, 154, 188,  16 },
        { 173,  42, 173,  26 },
        {  78, 214,  78, 230 },
        {  63, 102,  63, 240 },
        {  32, 240,  32, 118 },
        {  16, 128,  16, 128 },
    };
    uint8_t *p;
    uint8_t *end = &buffer[FRAME_WIDTH * 2];
    unsigned idx = (FRAME_WIDTH / 2 - 1) - (start_position % (FRAME_WIDTH / 2));
    p = &buffer[idx * 4];
    for (unsigned i = 0; i < 8; ++i) {
        for (int j = 0; j < FRAME_WIDTH / (2 * 8); ++j) {
            memcpy(p, &bar_color[i], 4);
            p += 4;
            if (end <= p) p = buffer;
        }
    }
    p = &buffer[FRAME_WIDTH * 2];
    for (unsigned i = 1; i < FRAME_HEIGHT; ++i) {
        memcpy(p, buffer, FRAME_WIDTH * 2);
        p += FRAME_WIDTH * 2;
    }
}



// ---------------------------------------------------------------------------
// TinyUSB Video callbacks
// ---------------------------------------------------------------------------
extern "C" {

int tud_video_commit_cb(uint_fast8_t ctl_idx, uint_fast8_t stm_idx,
                        video_probe_and_commit_control_t const *parameters) {
    (void)ctl_idx; (void)stm_idx;
    interval_ms = parameters->dwFrameInterval / 10000;
    if (interval_ms == 0) interval_ms = 1000 / FRAME_RATE;
    ESP_LOGI(TAG, "Video commit: interval=%lu ms", (unsigned long)interval_ms);
    return VIDEO_ERROR_NONE;
}

void tud_video_frame_xfer_complete_cb(uint_fast8_t ctl_idx, uint_fast8_t stm_idx) {
    (void)ctl_idx; (void)stm_idx;
    tx_busy = 0;
    frame_num++;
}

}  // extern "C"

// ---------------------------------------------------------------------------
// TinyUSB device task (replaces esp_tinyusb's task)
// ---------------------------------------------------------------------------
void usb_device_task(void *param) {
    (void)param;

    // Init USB PHY for High-Speed UTMI
    usb_phy_config_t phy_conf = {
        .controller = USB_PHY_CTRL_OTG,
        .target = USB_PHY_TARGET_UTMI,
        .otg_mode = USB_OTG_MODE_DEVICE,
        .otg_speed = USB_PHY_SPEED_HIGH,
        .otg_io_conf = NULL,
    };
    usb_phy_handle_t phy_hdl = NULL;
    esp_err_t ret = usb_new_phy(&phy_conf, &phy_hdl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "USB PHY init failed: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "USB PHY initialized (High-Speed UTMI)");

    // Init TinyUSB stack directly (port 1 = HS on P4)
    tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_HIGH,
    };
    if (!tusb_rhport_init(1, &dev_init)) {
        ESP_LOGE(TAG, "tusb_rhport_init() failed!");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "TinyUSB stack initialized on port 1 (HS)");

    // Signal setup() that we're ready
    xTaskNotifyGive((TaskHandle_t)param);

    // Process USB events forever
    while (1) {
        tud_task();
    }
}


#endif // IS_EAGLE

