# Volleyball POC (Crazyflie 2.1)

## Introduction

Final Master degree project. This project make UAV play VolleyBall with an augmented reality ball and field.
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

1. Calibrate your camera by running the script calibration.py, we have provided pictures for the following cameras: Logitech C270 and C920. If you have a different camera, you should use it to take pictures from different angles with a chessboard and modify the variable ```CAMERA_MODEL = 'C920'``` in calibration.py.
2. We have provided a chessboard for the calibration, you can find it in the folder ```calibration```. The file is called ```chessboard.png```. You should print it and use it to calibrate your camera.
3. You can start the volleyball game by running the script main.py.

### LPS

You need to setup LPS to make it working with crazyflie.
Also adding a flow deck module can improve the stability and precision of the UAV's. We recommend you to add it.

### Aruco Marker

An aruco marker (4x4) need to be placed at the origin (0,0,0) of the LPS.

## Usage

### UAVs
You should modify the following variables in the script ```game_controller.py``` 
```
FIRST_PLAYER = {
    "origin_x": DRONE ORIGIN,
    "uri": YOUR RADIO URI,
    # Offsets if there are some
    "x_offset": 0.0,
    "y_offset": 0.0,
    "z_offset": 0.0
}

Same for SECOND_PLAYER.

These variables will be used to initialize your drones.
```

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

## Authors

* **Marouane OURICHE**
* **Enzo Rudy SEKKAÏ**
