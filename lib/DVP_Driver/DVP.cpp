#include "DVP.h"

esp_video* DVP_init()
{

    Serial.println("Cam dvp controller init");

    static const esp_video_init_dvp_config_t s_dvp_config = {
        // .sccb_config = {

        // },
        .reset_pin = GPIO_NUM_NC,
        .pwdn_pin = GPIO_NUM_NC,
        .dvp_pin = {
            .data_width = CAM_CTLR_DATA_WIDTH_8,
            .data_io = {
                GPIO_NUM_23,
                GPIO_NUM_22,
                GPIO_NUM_21,
                GPIO_NUM_20,
                GPIO_NUM_13,
                GPIO_NUM_12,
                GPIO_NUM_11,
                GPIO_NUM_10,
            },
            .vsync_io = GPIO_NUM_46,
            .de_io = GPIO_NUM_45,
            .pclk_io = GPIO_NUM_2,
            .xclk_io = GPIO_NUM_NC,
        },
        .xclk_freq = 0,
    };

    static const esp_video_init_config_t s_cam_config = {
        .dvp = &s_dvp_config,

        //.usb_uvc  = &s_usb_uvc_config,
    };

    const esp_video_init_config_t *cam_config_ptr = &s_cam_config;

    Serial.println("Before esp_video_init");

    esp_err_t err = esp_video_init(cam_config_ptr);
    if (err != ESP_OK)
    {
        Serial.println("Video init error");
        Serial.println(err);
        while (1)
            ;
    }
    Serial.println("After esp_video_init");

    Serial.println("Before esp video open");
    esp_video *video = NULL;
    err = esp_video_open("DVP", &video);
    if (err != ESP_OK)
    {
        Serial.println("Video open error");
        Serial.println(err, HEX);
        while (1)
            ;
    }
    Serial.println("After esp video open");

    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = 720;
    fmt.fmt.pix.height = 480;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
    fmt.fmt.pix.field = V4L2_FIELD_INTERLACED_TB;

    Serial.println("Before esp_video_set_format");
    err = esp_video_set_format(video, &fmt);

    if (err != ESP_OK)
    {
        Serial.println("Video set format error");
        Serial.println(err);
        while (1)
            ;
    }
    Serial.println("After esp_video_set_format");

    Serial.println("Before esp_video_setup_buffer");
    esp_video_setup_buffer(video, V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP, 2);
    Serial.println("After esp_video_setup_buffer");

    Serial.println("Before esp_video_queue_element_index");
    err = esp_video_queue_element_index(video, V4L2_BUF_TYPE_VIDEO_CAPTURE, 0);
    if (err != ESP_OK)
    {
        Serial.println("esp_video_queue_element_index error");
        Serial.println(err);
        while (1)
            ;
    }
    err = esp_video_queue_element_index(video, V4L2_BUF_TYPE_VIDEO_CAPTURE, 1);
    if (err != ESP_OK)
    {
        Serial.println("esp_video_queue_element_index error");
        Serial.println(err);
        while (1)
            ;
    }
    Serial.println("After esp_video_queue_element_index");


    Serial.println("Finished DVP init");

    return video;
}


void start_dvp_capture(esp_video *video)
{
    esp_err_t err = esp_video_start_capture(video, V4L2_BUF_TYPE_VIDEO_CAPTURE);
    if (err != ESP_OK)
    {
        Serial.println("Video start capture error");
        Serial.println(err);
        while (1)
            ;
    }
    Serial.println("DVP capture started");
}

