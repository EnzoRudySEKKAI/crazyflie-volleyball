import math
import random

import numpy as np
import cv2 as cv

from threading import Thread
from drone_controller import DroneController
from position import Position



class GameController:
    
    CALIBRATION_FILE = 'calibration/calibration.npz'
    
    # Ball variables
    BALL_COLOR = (0, 255, 255)
    BALL_SIZE = 1.2
    BALL_POS = (0.0, 0.0, 0.5)

    # Aruco variables
    LIST_RVEC = []
    LIST_TVEC = []
    CPT = 0
    NEED_CALIBRATION = True
    RVEC = None
    TVEC = None

    
    # Number of intermediate positions for the trajectory the greater the number the smoother the trajectory and
    # the slower the ball is.
    NB_TRAJECTORIES = 60
    
    ARUCO_SIZE = 0.265
    
    # Boundaries of axis Y and Z
    MIN_Y = -0.1
    MAX_Y = 0.1
    
    MIN_Z = 0.4
    MAX_Z = 1
    
    # Circle transparency
    ALPHA = 0.5
    
    FIRST_PLAYER = {
        "origin_x": 1.0,
        "uri": 'radio://0/100/2M/E7E7E7E701',
        "x_offset": 0.06,
        "y_offset": 0.0,
        "z_offset": 0.0
    }
    
    SECOND_PLAYER = {
        "origin_x": -1.0,
        "uri": 'radio://0/100/2M/E7E7E7E702',
        "x_offset": 0.06,
        "y_offset": 0.03,
        "z_offset": 0.0,
        "cache": "./cache1"
    }
    
    def __init__(self):
        self.first_player = DroneController(
            **self.FIRST_PLAYER
        )
        self.second_player = DroneController(
            **self.SECOND_PLAYER
        )
        self.next_is_first_player = True

    @staticmethod
    def get_translation_matrix(tvec):
        tr = np.identity(n=4)
        tr[0:3, 3] = tvec
        return tr

    @staticmethod
    def parabolic_trajectory(start, end, max_height, min_height, frames):
        # Calculate the displacement between start and end points
        displacement = [end[i] - start[i] for i in range(3)]

        distance = math.hypot(displacement[0], displacement[1], displacement[2])

        # Calculate the acceleration needed to reach the max height in half of the total distance
        acceleration = (max_height - start[2]) / (distance / 2) ** 2
        # Calculate the vertical velocity needed to reach the max height at half of the total distance
        vert_velocity = acceleration * (distance / 2)
        # Calculate the time interval between each position
        interval = 1 / frames * distance
        # Calculate the x and z components of the velocity
        horiz_velocity = 1  #
        x_velocity = displacement[0] / distance * horiz_velocity
        z_velocity = displacement[1] / distance * horiz_velocity
        # Calculate the positions for each time interval
        positions = []
        for i in range(frames):
            t = i * interval
            x = start[0] + x_velocity * t
            y = start[2] + vert_velocity * t - 0.5 * acceleration * t ** 2 - 0.01
            z = start[1] + z_velocity * t
            if displacement[2] != 0:
                y_correction = displacement[2] * (t / distance)
                y += y_correction
            # Check if the y position is below the minimum height
            if y < min_height:
                y = min_height
            # Check if the y position is above the maximum height
            elif y > max_height:
                y = max_height
            positions.append((x, z, y))
        positions.append((end[0], end[1], end[2]))
        return positions

    def get_transform_matrix(self, rvec, tvec):
        mat = self.get_translation_matrix(tvec)
        mat[:3, :3] = cv.Rodrigues(rvec)[0]
        return mat

    @staticmethod
    def relative_transform_matrix(rotation, translation):
        x_c, x_s = np.cos(rotation[0]), np.sin(rotation[0])
        y_c, y_s = np.cos(rotation[1]), np.sin(rotation[1])
        z_c, z_s = np.cos(rotation[2]), np.sin(rotation[2])
        d_x = translation[0]
        d_y = translation[1]
        d_z = translation[2]
        translate_matrix = np.array([[1, 0, 0, d_x],
                                    [0, 1, 0, d_y],
                                    [0, 0, 1, d_z],
                                    [0, 0, 0, 1]], dtype=np.float32)
        rotate_x_matrix = np.array([[1, 0, 0, 0],
                                    [0, x_c, -x_s, 0],
                                    [0, x_s, x_c, 0],
                                    [0, 0, 0, 1]])
        rotate_y_matrix = np.array([[y_c, 0, y_s, 0],
                                    [0, 1, 0, 0],
                                    [-y_s, 0, y_c, 0],
                                    [0, 0, 0, 1]])
        rotate_z_matrix = np.array([[z_c, -z_s, 0, 0],
                                    [z_s, z_c, 0, 0],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]])
        return np.dot(rotate_z_matrix, np.dot(rotate_y_matrix, np.dot(rotate_x_matrix, translate_matrix)))

    def start_drones(self):
        Thread(target=self.first_player.main).start()
        Thread(target=self.second_player.main).start()
        
    def stop_drones(self):
        # TODO: To fix
        self.first_player.land_now = True
        self.second_player.land_now = True

    def main(self):
        
        # Aruco detection
        dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
        parameters = cv.aruco.DetectorParameters()
        detector = cv.aruco.ArucoDetector(dictionary, parameters)
        
        start_pos = ball_pos = self.BALL_POS
        
        end_pos = (
            round(random.uniform(0.2, self.first_player.origin_x), 2),
            round(random.uniform(self.MIN_Y, self.MAX_Y), 2),
            round(random.uniform(self.MIN_Z, self.MAX_Z), 2)
        )

        need_of_new_trajectory = True

        trajectory_positions = None

        # Get the camera calibration data
        with np.load(self.CALIBRATION_FILE) as X:
            mtx, dist, _, _ = [X[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]

        # Start the video capture
        cap = cv.VideoCapture(0)
        cap.set(cv.CAP_PROP_AUTOFOCUS, 0)
        
        self.start_drones()

        try:
            while True:
                                
                _, frame = cap.read()
                
                if cv.waitKey(1) & 0xFF == ord('q'):
                    self.stop_drones()
                    break

                # Check if the frame is valid
                if isinstance(frame, np.ndarray) and frame.any():
                    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

                    # Detect the markers
                    corners, ids, __ = detector.detectMarkers(gray)

                    if np.all(ids is not None):

                        if self.NEED_CALIBRATION:
                            if self.CPT < 5:
                                # Estimate the pose of the markers
                                rvec, tvec, _ = cv.aruco.estimatePoseSingleMarkers(corners, self.ARUCO_SIZE, mtx, dist)
                                # Get the first marker found
                                self.RVEC = rvec[0][0]
                                self.TVEC = tvec[0][0]
                                self.LIST_RVEC.append(self.RVEC)
                                self.LIST_TVEC.append(self.TVEC)
                                self.CPT += 1
                            else:
                                self.RVEC = np.mean(self.LIST_RVEC, axis=0)
                                self.TVEC = np.mean(self.LIST_TVEC, axis=0)
                                self.NEED_CALIBRATION = False


                        # Get the original marker position in 4x4 matrix representation
                        transform_matrix = self.get_transform_matrix(self.RVEC, self.TVEC)

                        # Get the transform matrix we want to apply to the obtained marker position
                        # E.g. rotate 180 deg (PI) along Y and then translate -10 cm along X

                        relative_transform_matrix_ = self.relative_transform_matrix(
                            [0, 0, 0], [ball_pos[0], ball_pos[1], ball_pos[2]])

                        # Now apply the transform to the original matrix by simply dot multiplying them
                        transform_matrix = np.dot(transform_matrix, relative_transform_matrix_)

                        # Extract rotation matrix and translation vector out of result and then display
                        rmat_relative = transform_matrix[:3, :3]
                        tmat_relative = transform_matrix[:3, 3:]
                        
                        # Perspective projection equations with depth
                        # 3D point in the marker coordinate system
                        p = np.array([0, 0, 0]).reshape(3, 1)
                        
                        # 3D point in the camera coordinate system
                        p_camera = np.dot(rmat_relative, p) + tmat_relative
                        
                        # 2D point in the image plane
                        p_image = np.dot(mtx, p_camera)
                        
                        # Normalized 2D point
                        p_image_normalized = p_image / p_image[2]

                        # Scale the ball size depending on the distance
                        ball_scale = int(self.BALL_SIZE / p_camera[2] * 200)

                        overlay = frame.copy()

                        # Draw the ball
                        cv.circle(overlay, (int(p_image_normalized[0]), int(p_image_normalized[1])), ball_scale, self.BALL_COLOR, -1)

                        cv.drawFrameAxes(frame, mtx, dist, rmat_relative, tmat_relative, 0.1)

                        frame = cv.addWeighted(overlay, self.ALPHA, frame, 1 - self.ALPHA, 0)

                        # Trajectory and movement of the ball
                        if need_of_new_trajectory:
                            trajectory_positions = self.parabolic_trajectory(start_pos, end_pos, self.MAX_Z+0.5, self.MIN_Z, self.NB_TRAJECTORIES)
                            need_of_new_trajectory = False
                        ball_pos = trajectory_positions[0]
                        if len(trajectory_positions) > 1:
                            trajectory_positions.pop(0)
                        else:
                            need_of_new_trajectory = True
                            start_pos = end_pos
                            if self.next_is_first_player:
                                end_pos = (
                                    round(random.uniform(0.2, self.first_player.origin_x), 2),
                                    round(random.uniform(0.2, self.MAX_Y), 2),
                                    round(random.uniform(self.MIN_Z, 1), 2)
                                )
                                self.next_is_first_player = False
                            else:
                                end_pos = (
                                    round(random.uniform(-0.4, self.second_player.origin_x), 2),
                                    round(random.uniform(self.MIN_Y, self.MAX_Y), 2),
                                    round(random.uniform(self.MIN_Z, self.MAX_Z), 2)
                                )
                                self.next_is_first_player = True
                        
                        position_to_visit = Position(end_pos[0], end_pos[1], end_pos[2])
                        
                        # First player starts first
                        if not self.next_is_first_player:
                            self.first_player.position_to_visit = position_to_visit 
                        else:
                            if position_to_visit.x < 0.0:
                                self.second_player.position_to_visit = position_to_visit

                    cv.imshow('frame', frame)
            cap.release()
            cv.destroyAllWindows()
            
        except Exception as err:
            print(str(err))
            self.stop_drones()

if __name__ == '__main__':
    GameController().main()
