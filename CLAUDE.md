# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

CAM-Software is embedded firmware for the ESP32-P4 (RISC-V) microcontroller, part of a camera system that captures analog video, encodes it to JPEG, and transmits it over a Si4463 RF radio link. The project uses a hybrid ESP-IDF/Arduino framework via PlatformIO.

Two build configurations exist:
- **CAMmk3** (`-DIS_CAM`): Full camera system with video input, JPEG encoding, radio TX, and dual RunCam camera control
- **EAGLE** (`-DIS_EAGLE`): Simplified radio-only configuration without camera capture

## Build Commands

```bash
# Initialize submodules (required first time)
git submodule update --init --recursive

# Build for CAMmk3
pio run -e CAMmk3

# Build for EAGLE
pio run -e EAGLE

# Upload firmware
pio run -e CAMmk3 -t upload
pio run -e EAGLE -t upload

# Serial monitor (115200 baud)
pio device monitor
```

**Note:** Flash failures that still show the flashing dialog are a known toolchain bug — the board likely flashed successfully. On Windows, spaces in paths require running `build_scripts/bf_safe.py` and setting `PLATFORMIO_CORE_DIR`.

## Architecture

### Source Layout

Source code lives in `main/` (configured via `platformio.ini`'s `src_dir`). Compile-time `#ifdef IS_CAM` / `#ifdef IS_EAGLE` guards select which subsystem is active.

- `main/main.cpp` — Entry point (`setup()`/`loop()`)
- `main/pins.h` — All hardware pin definitions for both board variants
- `main/errors.h` — Error code enum
- `main/cam/` — CAM subsystem: video capture, camera control, B2B comms
- `main/eagle/` — EAGLE subsystem: simplified radio-only mode
- `main/cam_radio/` — Si4463 radio abstraction layer (shared)

### FreeRTOS Threading (CAM)

The CAM system spawns 4 FreeRTOS tasks:

| Task | Core | Priority | Role |
|------|------|----------|------|
| `cmd_thread` | 1 | 10 | Processes B2B I2C commands (camera power, mux switching) |
| `poll_thread` | 1 | 5 | Monitors RunCam recording state via UART |
| `video_thread` | 0 | 5 | DVP capture → field merge → JPEG encode → radio TX (32KB stack) |
| `comm_thread` | 1 | 8 | I2C health monitor with 3s recovery / 180s fallback shutdown |

### Hardware Peripherals

- **TVP5151**: Analog video decoder (I2C address 0x5C), 8-bit Y output via DVP
- **Si4463**: RF radio module on SPI, supports auto-fragmented packets >50KB
- **RunCam**: Two cameras controlled via UART protocol
- **B2B I2C**: Slave-mode board-to-board communication for command/state exchange
- **Reed-Solomon**: Error correction applied to radio transmissions

### Custom Libraries (`lib/`)

- `DVP_Driver` — Digital Video Port hardware driver
- `JPEG_Driver` — Real-time JPEG encoder with interlaced field merging
- `tvp5151` — TVP5151 video decoder driver
- `si4463_nuke` — Custom Si4463 radio driver with large packet support
- `RS` — Reed-Solomon error correction codec
- `RadioHead` — RF communication library

### Companion Tools

- `CAMReader/read_jpeg_stream.py` — Python script for reading JPEG streams from the radio link
