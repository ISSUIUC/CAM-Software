# CAM Software

A complete embedded system for capturing and transmitting live video from high-altitude rockets. This repository contains firmware for the **CAM-MK3** board (video capture and transmission) and the **EAGLE** receiver board.

## Project Overview

The CAM-MK3 system converts analog video from rocket cameras into compressed digital video and transmits it wirelessly to a ground station. This is a hybrid esp-idf/Arduino framework project built with PlatformIO.

### What is CAM?

**CAM (Camera Module)** is the transmitter board located in the rocket's avionics bay. It:
- Captures analog NTSC video from Runcam Split 4S cameras via a TVP5151 ADC
- Converts analog video to digital YUV422 format using the DVP (Digital Video Port) interface
- Encodes the video as JPEG frames using hardware acceleration on the ESP32-P4
- Applies stride length and byte offset corrections in software
- Fragments and transmits compressed frames wirelessly via LR2021 FSK transceiver (~877 Kbps)
- Implements Reed-Solomon error correction for reliable transmission

### What is EAGLE?

**EAGLE** is the receiver board at the ground station. It:
- Receives fragmented JPEG packets from CAM via the LR2021 transceiver
- Reassembles fragmented packets back into complete JPEG frames
- Transmits decoded video frames to the ground station computer via USB-CDC (USBCDC) serial
- Provides visual feedback through status LEDs and buzzer

### The Video Pipeline

```
Runcam NTSC Video → TVP5151 ADC → ESP32-P4 DVP Interface → YUV422 Buffer
    ↓
Stride Correction → Byte Offset Correction → JPEG Encoding
    ↓
Packet Fragmentation (255-byte max) → Reed-Solomon EC → LR2021 FSK Transmission
    ↓
EAGLE Reception → Packet Reassembly → USB Output to Ground Station
```

## Installation / Toolchain

This project uses platformio to build (like most of our projects). Unlike most of our projects, we use a hybrid esp-idf / arduino framework build in platformio to allow for the usage of espidf components (namely esp_h264), which requires a custom platformio build toolchain.

To be able to build this project, download platformio (either core or through the vscode marketplace), and ensure you are able to run the build command: `pio run`. This will NOT be succesful, so just ensure you are able to run it.

You will first need to download the `arduino` component by cloning the git submodule:

```bash
$ git submodule update --init --recursive
```

This will take a while, and will clone the esp32_arduino repository to `components/arduino`.

#### Build errors:

> *I am running into issues due to spaces in my path...*

Unfortunately this is a pretty debilitating issue, and is caused by your system being unable to resolve the `.platformio` directory in your user folder. A script is present in this repository (`bf_safe.py`) which will move the directory to a path-safe directory (`C:/dev/.platformio`) and symbolically link `~/.platformio` to that directory.

To be able to use this new symlinked version, you will need to set the `PLATFORMIO_CORE_DIR` environment variable to the new core location, likely `C:/dev/.platformio`. The script will give you the command to run to perform this, or you can edit the environment variable as you see fit.

> *The components/arduino folder is empty...*

You likely forgot to init the submodule, read above and run the `git submodule` command listed.

> *My flash fails even though it looks like the board flashed*

This is normal behavior and comes from a known bug in the toolchain. If you see the flashing dialog, it is likely the board flashed successfully. Usually it is a watch dog error.

## Workspace Structure

```
CAM-Software/
├── main/                          # Main firmware source code
│   ├── cam/                      # CAM-specific code
│   │   ├── system.cpp/h          # CAM system initialization and main control loop
│   │   ├── camera.cpp/h          # DVP video capture driver
│   │   ├── tvpcontroller.h       # TVP5151 video ADC controller
│   │   ├── b2b.cpp/h             # Board-to-board (I2C) communication handler
│   │
│   ├── eagle/                    # EAGLE-specific code
│   │   ├── system.cpp/h          # EAGLE system initialization
│   │   ├── uvc.cpp/h             # USB Video Class streaming handler
│   │
│   ├── cam_radio.cpp/h           # Shared LR2021 radio driver (both boards)
│   ├── main.cpp                  # Entry point (selects CAM or EAGLE via IS_CAM/IS_EAGLE)
│   ├── pins.h                    # Pin definitions for both boards
│   ├── errors.h                  # Error definitions
│   └── CMakeLists.txt            # Firmware build configuration
│
├── lib/                          # Reusable driver libraries
│   ├── DVP_Driver/               # Digital Video Port driver for video capture
│   ├── H264_Driver/              # H.264 codec driver (unused, JPEG preferred)
│   ├── JPEG_Driver/              # JPEG encoder driver
│   ├── LR2021/                   # LR2021 transceiver driver
│   ├── tvp5151/                  # TVP5151 video ADC driver
│   ├── RS/                       # Reed-Solomon error correction library
│   ├── RadioHead/                # Radio communication library
│   └── RadioLib/                 # Alternative radio library
│
├── components/                   # ESP-IDF components (external dependencies)
│   ├── arduino/                  # Arduino-ESP32 core (git submodule)
│   ├── esp_video/                # Espressif video processing component
│   ├── esp_tinyusb/              # USB driver component
│   ├── arduino_tinyusb/          # Arduino USB wrapper
│   └── ... (other managed components)
│
├── managed_components/           # Auto-downloaded IDF components
│   ├── espressif__esp_h264/      # H.264 hardware encoder
│   ├── espressif__cjson/         # JSON parsing library
│   └── ... (other dependencies)
│
├── CAMReader/                    # Ground station Python utilities
│   ├── read_jpeg_stream.py       # Reads JPEG frames from USB serial & displays them
│   ├── combine_images.py         # Combines tiled JPEG frames
│   ├── pyproject.toml            # Python project configuration
│   └── commands.txt              # Debugging commands
│
├── build_scripts/                # Build automation scripts
│   └── bf_safe.py                # Fixes PlatformIO path issues
│
├── CMakeLists.txt                # Root build configuration
├── platformio.ini                # PlatformIO project settings
├── README.md                     # This file
└── sdkconfig.*                   # ESP-IDF SDK configurations
```

## Key Files Explained

### Firmware Source (main/)

| File | Purpose |
|------|---------|
| `main.cpp` | Entry point; initializes hardware, calls `sys_begin()` based on IS_CAM or IS_EAGLE macro |
| `pins.h` | GPIO/peripheral pin definitions for ESP32-P4 |
| `errors.h` | Error codes and handling definitions |

### CAM-Specific (main/cam/)

| File | Purpose |
|------|---------|
| `system.cpp/h` | Manages CAM subsystems: video capture, encoding, transmission; runs FreeRTOS tasks |
| `camera.cpp/h` | DVP driver for capturing video frames from the TVP5151 |
| `tvpcontroller.h` | I2C controller for TVP5151 configuration (resolution, color space, etc.) |
| `b2b.cpp/h` | I2C communication with MIDAS flight computer |

### EAGLE-Specific (main/eagle/)

| File | Purpose |
|------|---------|
| `system.cpp/h` | Manages EAGLE subsystems: radio reception, packet reassembly, USB output |
| `uvc.cpp/h` | USB Video Class driver for streaming decoded JPEG frames to ground station |

### Shared Code (main/)

| File | Purpose |
|------|---------|
| `cam_radio.cpp/h` | LR2021 transceiver driver (packet fragmentation, transmission, reassembly) |

### Driver Libraries (lib/)

| Library | Purpose |
|---------|---------|
| `DVP_Driver/` | Low-level interface to ESP32-P4's Digital Video Port for DMA video capture |
| `JPEG_Driver/` | Hardware JPEG encoder interface |
| `LR2021/` | LR2021 LoRa transceiver control and packet protocol |
| `tvp5151/` | TVP5151 NTSC video ADC initialization and control |
| `RS/` | Reed-Solomon forward error correction (256-byte blocks) |

### CAMReader (Python Ground Station)

| Script | Purpose |
|--------|---------|
| `read_jpeg_stream.py` | Connects to EAGLE via USB serial, decodes JPEG frames, displays live video with Flask web server on port 5002 |
| `combine_images.py` | Utility to recombine split JPEG frames during debugging |

## Installation & Build Setup

### Prerequisites

1. **PlatformIO** - Install via VSCode marketplace or command line:
   ```bash
   pip install platformio
   ```

2. **Clone submodules** - Download the Arduino component (required):
   ```bash
   git submodule update --init --recursive
   ```
   This populates `components/arduino/` (~2GB, takes several minutes).

3. **ESP32-P4 toolchain** - PlatformIO downloads this automatically on first build.

### Building the Firmware

```bash
# Build for CAM (default)
pio run

# Build for EAGLE
pio run -e eagle

# Flash CAM to device
pio run -t upload

# Flash EAGLE to device
pio run -e eagle -t upload

# Monitor serial output
pio device monitor --baud 115200
```

### Selecting CAM or EAGLE

The firmware automatically detects which board to build based on the environment in `platformio.ini`:
- `[env:default]` = CAM
- `[env:eagle]` = EAGLE

The corresponding `-DPROJECT_CAM` or `-DPROJECT_EAGLE` flags enable the correct code paths in `main/main.cpp`.

## Running the Ground Station

### Prerequisites (Python)

```bash
cd CAMReader
pip install -r requirements.txt  # or use: pip install pyserial pillow flask flask-cors
```

### Starting the Ground Station

```bash
python read_jpeg_stream.py
```

The script will:
1. Connect to EAGLE via USB serial 
2. Decode incoming JPEG frames
3. Save latest frame to `output.jpg`
4. Stream continuous video to `output_stream.mjpg`
5. Optionally display live preview (if OpenCV is installed)
6. Optionally serve web interface at `http://localhost:5002/video_feed`

## Workflow & Data Flow

### During Flight

1. **CAM captures video** - TVP5151 continuously converts analog NTSC to YUV422 (720×480 @ ~30fps)
2. **DVP DMA transfer** - Video frames automatically transferred to ESP32-P4 memory via DMA
3. **Encoding** - JPEG encodes video frames (stride correction, byte-offset handling)
4. **Fragmentation** - JPEG frames (typically 15-25 KB) are split into 255-byte packets
5. **Error correction** - Reed-Solomon codes added for FEC
6. **Transmission** - Packets transmitted via LR2021 at 1 Mbps FSK (~8 fps achieved)

### On the Ground

1. **EAGLE receives packets** - Reassembles fragmented packets via radio
2. **Error recovery** - Reed-Solomon corrects corrupted bytes
3. **Frame reconstruction** - Complete JPEG frames sent to USB host
4. **Ground station decodes** - `read_jpeg_stream.py` displays real-time video

## Troubleshooting

### Build Issues

#### "The components/arduino folder is empty"
You forgot to initialize the git submodule. Run:
```bash
git submodule update --init --recursive
```

#### "I am running into issues due to spaces in my path..."
PlatformIO cannot resolve `.platformio` directory paths with spaces. Use the included script:
```bash
python bf_safe.py
```
Then set the environment variable:
```bash
export PLATFORMIO_CORE_DIR=/path/to/safe/location/.platformio
```

#### "My flash fails even though it looks like the board flashed"
This is a known bug in the esp-idf toolchain. If you see a flashing dialog, the board likely flashed successfully. Check device behavior before assuming failure.

#### "USB serial port not found"
- Ensure USB cable is connected and board is in bootloader mode
- Check port name: `pio device list`
- Update `read_jpeg_stream.py` with correct port if needed

### Runtime Issues

#### "No video frames received on EAGLE"
- Check LR2021 radio connection (SPI pins, interrupt lines)
- Verify both CAM and EAGLE are configured for same frequency (434 MHz default)
- Monitor CAM via USB: `pio device monitor` and look for transmission errors
- Verify antenna connections

#### "Corrupted video frames (purple/green images)"
This was caused by byte offset errors in YUV422 → JPEG conversion. The firmware now includes automatic byte-offset detection and correction.

#### "Video stutters or has low frame rate"
- Check JPEG quality settings in `main/cam/system.cpp`
- Verify JPEG encoder is not bottlenecked (use terminal output statistics)
- Check radio link quality (interference, distance)
- Look for Reed-Solomon FEC corrections (indicates packet loss)

## Hardware Components

- **MCU**: ESP32-P4 (dual-core, 400 MHz, DVP interface)
- **Video ADC**: TVP5151 (NTSC → YUV422 converter)
- **Radio**: LR2021 LoRa transceiver (434 MHz FSK/FLRC)
- **USB**: CH340 serial-to-USB adapter
- **Video Encoders**: Hardware JPEG + software stride correction
- **Power**: 1.8V, 3.3V, 5.5V rails with buck converters

## Development Notes

- The code uses FreeRTOS tasks for concurrent video capture, encoding, and transmission
- DVP uses DMA for efficient video frame transfers
- JPEG encoding uses hardware acceleration on ESP32-P4
- Reed-Solomon FEC uses 256-byte blocks with adjustable redundancy
- The system achieves ~877 Kbps transmission rates with ~8 fps video delivery

