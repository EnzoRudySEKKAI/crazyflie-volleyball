import math
import random

import numpy as np
import cv2 as cv


def get_translation_matrix(tvec):
    tr = np.identity(n=4)
    tr[0:3, 3] = tvec
    return tr

def parabolic_trajectory(start, end, speed, max_height, min_height, num_positions=80):
    # Calculate the displacement between start and end points
    displacement = [end[i] - start[i] for i in range(3)]

    distance = math.hypot(displacement[0], displacement[1], displacement[2])

    # Calculate the time it would take to travel the displacement at the given speed
    time = distance / speed
    # Calculate the acceleration needed to reach the max height in half of the total time
    acceleration = (max_height - start[2]) / (time / 2) ** 2
    # Calculate the vertical velocity needed to reach the max height at half of the total time
    vert_velocity = acceleration * (time / 2)
    # Calculate the time interval between each position
    interval = 1 / num_positions * time
    # Calculate the x and z components of the velocity
    horiz_velocity = distance / time  # (time/2)
    x_velocity = displacement[0] / distance * horiz_velocity
    z_velocity = displacement[1] / distance * horiz_velocity
    # Calculate the positions for each time interval
    positions = []
    for i in range(num_positions):
        t = i * interval
        x = start[0] + x_velocity * t
        y = start[2] + vert_velocity * t - 0.5 * acceleration * t ** 2
        z = start[1] + z_velocity * t - 0.02
        if displacement[1] != 0:
            y_correction = displacement[2] * (t / time)
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

def get_transform_matrix(rvec, tvec):
    mat = get_translation_matrix(tvec)
    mat[:3, :3] = cv.Rodrigues(rvec)[0]
    return mat


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


if __name__ == "__main__":

    # aruco detection
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
    parameters = cv.aruco.DetectorParameters()
    detector = cv.aruco.ArucoDetector(dictionary, parameters)

    # ball
    ball_speed_m_s = 1
    ball_color = (0, 255, 255)
    ball_size = 3

    # camera
    cam_fps = 30

    # aruco
    aruco_size = 0.265

    # Variable for the movement

    # Varibale to know if the ball is going to a side or another
    joueur1 = False
    joueur2 = True

    # Boundaries of X axis
    max_x = 0.5
    min_x = -0.5

    # Boundaries of Y axis
    max_y = 1
    min_y = -1

    # Boundaries of Z axis
    max_z = 1.5
    min_z = 0.2

    duration = 1

    # Initial position and destination of the ball
    destination_j1 = (round(random.uniform(min_x, max_x), 2),
                      round(random.uniform(min_y, max_y), 2),
                      0.00)

    destination_j2 = (round(random.uniform(min_x, max_x), 2),
                      round(random.uniform(min_y, max_y), 2),
                      0.00)

    # Initial position of the ball
    ball_pos = (0.0, 0.0, 0.4)

    start_pos = ball_pos
    end_pos = (0.4, 0.4, 0.4)

    test = True

    pos = None

    # Get the camera calibration data
    with np.load('calibration/calibration.npz') as X:
        mtx, dist, _, _ = [X[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]

    # Define the axis
    axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]]).reshape(-1, 3)

    # Start the video capture
    cap = cv.VideoCapture(0)

    while True:
        _, frame = cap.read()

        # Check if the frame is valid
        if isinstance(frame, np.ndarray) and frame.any():

            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

            # Detect the markers
            corners, ids, rejectedImgPoints = detector.detectMarkers(gray)

            if np.all(ids is not None):
                # Estimate the pose of the markers
                rvec, tvec, _ = cv.aruco.estimatePoseSingleMarkers(corners, aruco_size, mtx, dist)

                # Get the first marker found
                rvec = rvec[0][0]
                tvec = tvec[0][0]

                # Get the original marker position in 4x4 matrix representation
                transform_matrix = get_transform_matrix(rvec, tvec)

                # Get the transform matrix we want to apply to the obtained marker position
                # E.g. rotate 180 deg (PI) along Y and then translate -10 cm along X

                # relative_transform_matrix_ = relative_transform_matrix([0, np.pi, 0], [axis_x, axis_y, axis_z])
                relative_transform_matrix_ = relative_transform_matrix([0, 0, 0],
                                                                       [ball_pos[0], ball_pos[1], ball_pos[2]])

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
                ball_scale = int(ball_size / p_camera[2] * 200)

                overlay = frame.copy()
                alpha = 0.5

                # Draw the ball
                cv.circle(overlay, (int(p_image_normalized[0]), int(p_image_normalized[1])), ball_scale, ball_color, -1)

                cv.drawFrameAxes(frame, mtx, dist, rmat_relative, tmat_relative, 0.1)

                frame = cv.addWeighted(overlay, alpha, frame, 1 - alpha, 0)

                # Définit les points de départ et d'arrivée, ainsi que la hauteur de la trajectoire
                height = max_z

                # Définit le framerate de la caméra et la durée totale de la trajectoire
                fps = 30
                duration = 5  # Durée totale de la trajectoire en secondes

                if test:
                    pos = parabolic_trajectory(start_pos, end_pos, ball_speed_m_s, max_z, min_z)
                    test = False
                    print(pos)
                    print(ball_pos)
                    print(end_pos)
                ball_pos = pos[0]
                if len(pos) > 1:
                    pos.pop(0)
                else:
                    test = True
                    start_pos = end_pos
                    if joueur1:
                        end_pos = (round(random.uniform(min_x, max_x), 2),
                                   round(random.uniform(0.2, max_y), 2),
                                   round(random.uniform(min_z, 1), 2))
                        joueur1 = False
                        joueur2 = True
                    else:
                        end_pos = (round(random.uniform(min_x, max_x), 2),
                                   round(random.uniform(min_y, -0.2), 2),
                                   round(random.uniform(min_z, 1), 2))
                        joueur1 = True
                        joueur2 = False

            cv.imshow('frame', frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()
