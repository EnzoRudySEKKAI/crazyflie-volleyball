# Volleyball POC (Crazyflie 2.1)

## Important:

Please check the code's [documentation](https://marouone.github.io/index.html) before reading this.

## Introduction

Final master degree project. This project makes UAVs (Crazyflie drones) play VolleyBall with an augmented reality ball and field.
<p align="center">
<img src="https://github.com/EnzoRudySEKKAI/PFECam/blob/main/gif/2drone_ex00002054_AdobeExpress.gif" width="600">
</p>

## Getting Started

Python 3.9 was used in this project, we recommend doing the same.

### Requirements
You should start by installing all the libraries.

```
python -m pip install -r requirements.txt
```

### Calibration

First of all, you will need to calibrate your camera. You can find the script in the folder ```calibration```.

1. Calibrate your camera by running the script calibration.py, we have provided pictures for the following cameras: Logitech C270 and C920. If you have a different camera, you should use it to take pictures from different angles with a chessboard and modify the variable ```CAMERA_MODEL``` in calibration.py.
2. We have provided a chessboard for the calibration, you can find it in the folder ```calibration```. The file is called ```chessboard.png```. You should print it and use it to calibrate your camera.
3. You can start the volleyball game by running the script main.py.

### LPS

You need to setup the Loco Positioning System to make it work with Crazyflie. (If you want to print the anchor's stand, the STL file is in the folder "lps-anchor-stand". \
Adding a Flow Deck improves the stability and precision of the Crazyflie. We recommend you to add it.

### Aruco Marker

An aruco marker (4x4) need to be placed at the origin (0,0,0) of the LPS.

## Usage

### UAVs
You should modify the following variables in json file ```drones.json```
```
[
    {
        "origin_x": 1.0,
        "origin_y": 0.0,
        "min_x": 0.3,
        "max_x": 1,
        "min_y": 0.5,
        "max_y": -0.5,
        "x_offset": 0.08,
        "y_offset": 0.03,
        "z_offset": -0.15,
        "uri": "radio://0/100/2M/E7E7E7E701",
        "cache": "./cache"
     }
]
```

This is an example of one drone, if you want to use more drones, use the same dict format and append them to the list.
These settings will be used to initialize your drones.

The meaning of each field can be found in our [documentation](https://marouone.github.io/index.html).

### Aruco markers

You can modify the following variables in the script ```game_controller.py``` :

```ARUCO_SIZE``` : The size of the aruco marker in meters.

### Ball
You can modify the following variables in the script ```game_controller.py``` :

```NB_TRAJECTORIES``` : The numbers of intermediate trajectories from a start point to an end point. 
The greater the number, the slower the ball will be. The lower the number, the faster the ball will be.
1 intermediate trajectory will be read at each frame. If the camera reads at 30 frames per second, 
30 intermediate trajectories will be read.

```BALL_COLOR``` : The color of the ball in RGB system.

```BALL_SIZE``` : The size of the ball.

### Field boundaries
You can modify the following variables in the script ```game_controller.py``` :

```MIN_Y``` : The minimum y value of the play field.

```MAX_Y``` : The maximum y value of the play field.

```MIN_X``` : The minimum x value of the play field.

```MAX_X``` : The maximum x value of the play field.

```MIN_Z``` : The minimum z value of the play field. (Also the maximum height of the ball for its parabolic trajectory).

```MAX_Z``` : The maximum z value of the play field. (Also the minimum height of the ball for its parabolic trajectory).

After doing all of the above, you just need to do ```python main.py``` and enjoy the game.

If you're interested in learning more about cflib and what it's capable of, you can check more usage examples [here](https://github.com/bitcraze/crazyflie-lib-python/tree/master/examples).

## Authors

* **Marouane OURICHE**
* **Enzo Rudy SEKKAÏ**
