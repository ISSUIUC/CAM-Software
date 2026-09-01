# CAM Software

A complete embedded system for capturing and transmitting live video from high-altitude rockets. This repository contains firmware for the **CAM-MK3** board (video capture and transmission) and the **EAGLE** receiver board.

## Branch Conventions (follow these to the best of your abilities)

`feature/my-feature`: for something new

`bugfix/my-bugfix`: for fixing something old

`launch/my-launch`: for a launch milestone

`docs/some-docs`: for documentation

`misc/something`: for anything else

## How to setup and run python script

**Clone submodules** - Download the Arduino component (required):
   ```bash
   git submodule update --init --recursive
   ```
   This populates `components/arduino/` (~2GB, takes several minutes).


Method 1:

```bash
cd CAMReader
python -m venv .venv
# For windows:
.venv\scripts\activate
# For mac:
. .venv/bin/activate

#once you see a (camreader) thing in your terminal, run

pip install -r requirements.txt  # or use: pip install pyserial pillow flask flask-cors
```

Method 2 (if you have uv):

```
uv sync
```


### Running the ground scripts

```bash
python read_jpeg_stream.py
```

or 

```bash
uv run read_jpeg_stream.py
```

The script will:
1. Connect to EAGLE via USB serial 
2. Decode incoming JPEG frames
3. Save latest frame to `output.jpg`
4. Stream continuous video to `output_stream.mjpg`
5. Optionally display live preview (if OpenCV is installed)
6. Optionally serve web interface at `http://localhost:5002/video_feed`


# How does everything work? 

Great question

Here are some diagrams that break down both systems fairly well. Feel free to ask us any questions on any specific portion.

![Camera Software](<docs/CAM Software Archi.drawio.png>)
![EAGLE Software](<docs/EAGLE Software Arch.drawio.png>)