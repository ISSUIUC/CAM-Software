#pragma once
// ============================================================================
// EAGLE: Minimal UVC test pattern — bypasses esp_tinyusb, inits USB directly
// ============================================================================
#ifdef IS_EAGLE

#include "esp_log.h"
#include "esp_private/usb_phy.h"
#include "tusb.h"
#include "class/video/video.h"
#include "class/video/video_device.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"




// ---------------------------------------------------------------------------
// Frame configuration
// ---------------------------------------------------------------------------
#define FRAME_WIDTH   720
#define FRAME_HEIGHT  480
#define FRAME_RATE    5
#define FRAME_SIZE_BYTES  (FRAME_WIDTH * FRAME_HEIGHT * 2)  // YUY2: 2 bytes/pixel

// UVC entity IDs
#define UVC_ENTITY_CAP_INPUT_TERMINAL  0x01
#define UVC_ENTITY_CAP_OUTPUT_TERMINAL 0x02

#define EPNUM_VIDEO_IN  0x81
#define UVC_CLOCK_FREQUENCY 27000000

extern volatile unsigned frame_num;
extern volatile unsigned tx_busy;
extern unsigned interval_ms;
extern uint8_t *frame_buffer;
extern const char *TAG;


// ---------------------------------------------------------------------------
// Descriptor structs (same as TinyUSB video_capture example)
// ---------------------------------------------------------------------------
enum {
    ITF_NUM_VIDEO_CONTROL = 0,
    ITF_NUM_VIDEO_STREAMING,
    ITF_NUM_TOTAL
};

typedef struct TU_ATTR_PACKED {
    tusb_desc_interface_t                          itf;
    tusb_desc_video_control_header_1itf_t          header;
    tusb_desc_video_control_camera_terminal_t      camera_terminal;
    tusb_desc_video_control_output_terminal_t      output_terminal;
} uvc_control_desc_t;

typedef struct TU_ATTR_PACKED {
    tusb_desc_interface_t                          itf;
    tusb_desc_video_streaming_input_header_1byte_t header;
    tusb_desc_video_format_uncompressed_t          format;
    tusb_desc_video_frame_uncompressed_continuous_t frame;
    tusb_desc_video_streaming_color_matching_t     color;
    tusb_desc_endpoint_t                           ep;
} uvc_streaming_desc_t;

typedef struct TU_ATTR_PACKED {
    tusb_desc_configuration_t   config;
    tusb_desc_interface_assoc_t iad;
    uvc_control_desc_t          video_control;
    uvc_streaming_desc_t        video_streaming;
} uvc_cfg_desc_t;

// ---------------------------------------------------------------------------
// Descriptors — matching TinyUSB video_capture example exactly
// ---------------------------------------------------------------------------
const tusb_desc_device_t desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0x303A,
    .idProduct          = 0x8000,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = 3,
    .bNumConfigurations = 1
};

extern const char *string_desc_arr[];

const uvc_cfg_desc_t desc_fs_configuration = {
    .config = {
        .bLength             = sizeof(tusb_desc_configuration_t),
        .bDescriptorType     = TUSB_DESC_CONFIGURATION,
        .wTotalLength        = sizeof(uvc_cfg_desc_t),
        .bNumInterfaces      = ITF_NUM_TOTAL,
        .bConfigurationValue = 1,
        .iConfiguration      = 0,
        .bmAttributes        = TU_BIT(7),
        .bMaxPower           = 100 / 2
    },
    .iad = {
        .bLength           = sizeof(tusb_desc_interface_assoc_t),
        .bDescriptorType   = TUSB_DESC_INTERFACE_ASSOCIATION,
        .bFirstInterface   = ITF_NUM_VIDEO_CONTROL,
        .bInterfaceCount   = 2,
        .bFunctionClass    = TUSB_CLASS_VIDEO,
        .bFunctionSubClass = VIDEO_SUBCLASS_INTERFACE_COLLECTION,
        .bFunctionProtocol = VIDEO_ITF_PROTOCOL_UNDEFINED,
        .iFunction         = 0
    },
    .video_control = {
        .itf = {
            .bLength            = sizeof(tusb_desc_interface_t),
            .bDescriptorType    = TUSB_DESC_INTERFACE,
            .bInterfaceNumber   = ITF_NUM_VIDEO_CONTROL,
            .bAlternateSetting  = 0,
            .bNumEndpoints      = 0,
            .bInterfaceClass    = TUSB_CLASS_VIDEO,
            .bInterfaceSubClass = VIDEO_SUBCLASS_CONTROL,
            .bInterfaceProtocol = VIDEO_ITF_PROTOCOL_15,
            .iInterface         = 4
        },
        .header = {
            .bLength           = sizeof(tusb_desc_video_control_header_1itf_t),
            .bDescriptorType   = TUSB_DESC_CS_INTERFACE,
            .bDescriptorSubType = VIDEO_CS_ITF_VC_HEADER,
            .bcdUVC            = VIDEO_BCD_1_50,
            .wTotalLength      = sizeof(uvc_control_desc_t) - sizeof(tusb_desc_interface_t),
            .dwClockFrequency  = UVC_CLOCK_FREQUENCY,
            .bInCollection     = 1,
            .baInterfaceNr     = { ITF_NUM_VIDEO_STREAMING }
        },
        .camera_terminal = {
            .bLength           = sizeof(tusb_desc_video_control_camera_terminal_t),
            .bDescriptorType   = TUSB_DESC_CS_INTERFACE,
            .bDescriptorSubType = VIDEO_CS_ITF_VC_INPUT_TERMINAL,
            .bTerminalID       = UVC_ENTITY_CAP_INPUT_TERMINAL,
            .wTerminalType     = VIDEO_ITT_CAMERA,
            .bAssocTerminal    = 0,
            .iTerminal         = 0,
            .wObjectiveFocalLengthMin = 0,
            .wObjectiveFocalLengthMax = 0,
            .wOcularFocalLength      = 0,
            .bControlSize      = 3,
            .bmControls        = { 0, 0, 0 }
        },
        .output_terminal = {
            .bLength           = sizeof(tusb_desc_video_control_output_terminal_t),
            .bDescriptorType   = TUSB_DESC_CS_INTERFACE,
            .bDescriptorSubType = VIDEO_CS_ITF_VC_OUTPUT_TERMINAL,
            .bTerminalID       = UVC_ENTITY_CAP_OUTPUT_TERMINAL,
            .wTerminalType     = VIDEO_TT_STREAMING,
            .bAssocTerminal    = 0,
            .bSourceID         = UVC_ENTITY_CAP_INPUT_TERMINAL,
            .iTerminal         = 0
        }
    },
    .video_streaming = {
        .itf = {
            .bLength            = sizeof(tusb_desc_interface_t),
            .bDescriptorType    = TUSB_DESC_INTERFACE,
            .bInterfaceNumber   = ITF_NUM_VIDEO_STREAMING,
            .bAlternateSetting  = 0,
            .bNumEndpoints      = 1,
            .bInterfaceClass    = TUSB_CLASS_VIDEO,
            .bInterfaceSubClass = VIDEO_SUBCLASS_STREAMING,
            .bInterfaceProtocol = VIDEO_ITF_PROTOCOL_15,
            .iInterface         = 5
        },
        .header = {
            .bLength           = sizeof(tusb_desc_video_streaming_input_header_1byte_t),
            .bDescriptorType   = TUSB_DESC_CS_INTERFACE,
            .bDescriptorSubType = VIDEO_CS_ITF_VS_INPUT_HEADER,
            .bNumFormats       = 1,
            .wTotalLength      = sizeof(uvc_streaming_desc_t)
                                 - sizeof(tusb_desc_interface_t)
                                 - sizeof(tusb_desc_endpoint_t),
            .bEndpointAddress  = EPNUM_VIDEO_IN,
            .bmInfo            = 0,
            .bTerminalLink     = UVC_ENTITY_CAP_OUTPUT_TERMINAL,
            .bStillCaptureMethod = 0,
            .bTriggerSupport   = 0,
            .bTriggerUsage     = 0,
            .bControlSize      = 1,
            .bmaControls       = { 0 }
        },
        .format = {
            .bLength           = sizeof(tusb_desc_video_format_uncompressed_t),
            .bDescriptorType   = TUSB_DESC_CS_INTERFACE,
            .bDescriptorSubType = VIDEO_CS_ITF_VS_FORMAT_UNCOMPRESSED,
            .bFormatIndex      = 1,
            .bNumFrameDescriptors = 1,
            .guidFormat        = { TUD_VIDEO_GUID_YUY2 },
            .bBitsPerPixel     = 16,
            .bDefaultFrameIndex = 1,
            .bAspectRatioX     = 0,
            .bAspectRatioY     = 0,
            .bmInterlaceFlags  = 0,
            .bCopyProtect      = 0
        },
        .frame = {
            .bLength           = sizeof(tusb_desc_video_frame_uncompressed_continuous_t),
            .bDescriptorType   = TUSB_DESC_CS_INTERFACE,
            .bDescriptorSubType = VIDEO_CS_ITF_VS_FRAME_UNCOMPRESSED,
            .bFrameIndex       = 1,
            .bmCapabilities    = 0,
            .wWidth            = FRAME_WIDTH,
            .wHeight           = FRAME_HEIGHT,
            .dwMinBitRate      = FRAME_WIDTH * FRAME_HEIGHT * 16 * 1,
            .dwMaxBitRate      = FRAME_WIDTH * FRAME_HEIGHT * 16 * FRAME_RATE,
            .dwMaxVideoFrameBufferSize = FRAME_SIZE_BYTES,
            .dwDefaultFrameInterval    = 10000000 / FRAME_RATE,
            .bFrameIntervalType = 0,
            .dwFrameInterval   = {
                10000000 / FRAME_RATE,
                10000000,
                10000000 / FRAME_RATE
            }
        },
        .color = {
            .bLength           = sizeof(tusb_desc_video_streaming_color_matching_t),
            .bDescriptorType   = TUSB_DESC_CS_INTERFACE,
            .bDescriptorSubType = VIDEO_CS_ITF_VS_COLORFORMAT,
            .bColorPrimaries   = VIDEO_COLOR_PRIMARIES_BT709,
            .bTransferCharacteristics = VIDEO_COLOR_XFER_CH_BT709,
            .bMatrixCoefficients      = VIDEO_COLOR_COEF_SMPTE170M
        },
        .ep = {
            .bLength          = sizeof(tusb_desc_endpoint_t),
            .bDescriptorType  = TUSB_DESC_ENDPOINT,
            .bEndpointAddress = EPNUM_VIDEO_IN,
            .bmAttributes     = { .xfer = TUSB_XFER_BULK },
            .wMaxPacketSize   = 64,
            .bInterval        = 1
        }
    }
};

// HS version: mutable copy with 512-byte bulk EP
extern uvc_cfg_desc_t desc_hs_configuration;

uint8_t const *get_hs_cfg(void);

const tusb_desc_device_qualifier_t desc_device_qualifier = {
    .bLength            = sizeof(tusb_desc_device_qualifier_t),
    .bDescriptorType    = TUSB_DESC_DEVICE_QUALIFIER,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .bNumConfigurations = 1,
    .bReserved          = 0
};




void usb_device_task(void *param);

void fill_color_bar(uint8_t *buffer, unsigned start_position);




#endif 