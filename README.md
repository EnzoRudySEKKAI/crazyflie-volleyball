# Volleyball POC (Crazyflie 2.1)

Python 2.0 was used in this project, we recommed doing the same.

## Requirements
You should start by installing all the libraries.

```
python -m pip install -r requirements.txt
```

## Usage

1. Calibrate your camera by running the script calibration.py, we have provided pictures for the following cameras: Logitech C270 and C920. If you have a different camera, you should use it to take pictures from different angles and modify the variable ```CAMERA_MODEL = 'C920'``` the calibration.py.
2. You can start the volleyball game by running the script game_controller.py.

### Nota Bene
You should modify the following variables in the script ```game_controller.py``` 

FIRST_PLAYER = {
    "origin_x": DRONE ORIGIN,
    "uri": YOUR RADIO URI,
    # Offsets if there are some
    "x_offset": 0.0,
    "y_offset": 0.0,
    "z_offset": 0.0
}
