# Dual Hikrobot Camera Capture App

Bushing Washer inspection app with two Hikrobot camera feeds and image capture.

| Camera | Model | Connection | View |
|--------|-------|------------|------|
| 1 | MV-CA032-10GM | GigE (Ethernet) | Side View |
| 2 | MV-CA035 | USB 3.0 | Top View |

## Prerequisites

1. **Hikrobot MVS SDK** – Download and install from [Hikrobot](https://www.hikrobotics.com/en/machinevision/service/download).
   - During installation, make sure **Python development** is selected.
   - The installer places `MvCameraControl_class.py` and `MvCameraControl_header.py` plus the native DLLs in:
     ```
     C:\Program Files (x86)\Common Files\MVS\Development\Samples\Python\MvImport\
     ```
   - Either copy those files into this project folder **or** add that path to your `PYTHONPATH`.

2. **Python 3.8+**

3. **Python packages**:
   ```
   pip install -r requirements.txt
   ```

## Setup

### GigE Camera (MV-CA032-10GM)
- Plug the Ethernet cable from the camera into your PC's Ethernet port.
- **You do NOT need to manually change your adapter IP or subnet mask.**  
  The app automatically detects if the camera is on a different subnet and uses `ForceIP` to assign a compatible address before opening the camera.

### USB Camera (MV-CA035)
- Plug the USB 3.0 cable from the camera into a USB 3.0 port on your PC.
- No extra configuration needed.

## Running

```
python camera_app.py
```

## Usage

1. Click **Start Stream** – the app enumerates cameras, auto-configures the GigE IP if needed, and starts live feeds.
2. Both camera feeds appear side-by-side (70% of the screen).
3. Click **Capture** to grab the current frame from both cameras simultaneously.
   - Side view images → `side view/` folder
   - Top view images  → `top view/` folder
   - Files are timestamped BMP images.
4. Click **Stop Stream** to disconnect.

## Folder Structure

```
image_capture/
├── camera_app.py          # Main application
├── requirements.txt
├── README.md
├── side view/             # Captured side-view images (auto-created)
└── top view/              # Captured top-view images (auto-created)
```

## Troubleshooting

| Issue | Fix |
|-------|-----|
| "No cameras found" | Close MVS Studio (it holds exclusive access). Check cables/power. |
| GigE camera not detected | Make sure only one network adapter is active, or check that Windows Firewall isn't blocking GigE traffic. |
| Import error for `MvCameraControl_class` | Copy the MVS Python files into this folder or set `PYTHONPATH`. |
| Black/corrupt image | Camera may be sending a pixel format not handled — check `enPixelType` in debug. |
