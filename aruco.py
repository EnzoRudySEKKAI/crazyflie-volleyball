import numpy as np
import cv2 as cv


def get_translation_matrix(tvec):
    t = np.identity(n=4)
    t[0:3, 3] = tvec
    return t


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
                                 [0, 0, 0, 1]])
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


def draw(img, corners, imgpts):
    print(corners)
    corner = tuple(corners[0].ravel())
    imgpts0 = tuple(imgpts[0].ravel())
    imgpts1 = tuple(imgpts[1].ravel())
    imgpts2 = tuple(imgpts[2].ravel())
    tuplecorner = (int(corner[0]), int(corner[1]))
    tuplecorner2 = (int(corner[4]), int(corner[5]))
    if (int(corner[0]) - int(corner[4])) > 0:
        test = int(corner[0]) - int(corner[4])
    else:
        test = int(corner[4]) - int(corner[0])
    img = cv.circle(img, tuplecorner, test, (0, 0, 255), -1)


if __name__ == "__main__":

    # aruco detection
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
    parameters = cv.aruco.DetectorParameters()
    detector = cv.aruco.ArucoDetector(dictionary, parameters)

    # Variable pour le déplacement
    plus = True
    plus_z = False

    max_x = 0.2
    min_x = -0.2

    max_y = 0.2
    min_y = -0.2

    max_z = -0.1
    min_z = -0.05

    axis_x = 0
    axis_y = 0
    axis_z = -0.1

    # camera calibration
    with np.load('calibration/calibration.npz') as X:
        mtx, dist, _, _ = [X[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]

    axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]]).reshape(-1, 3)

    cap = cv.VideoCapture(0)
    
    while True:
        _, frame = cap.read()
        
        if isinstance(frame, np.ndarray) and frame.any():

            # frame = cv.resize(frame, None, fx=0.7, fy=0.7, interpolation=cv.INTER_AREA)
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            # aruco detection
            
            corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
            if np.all(ids is not None):
                rvec, tvec, _ = cv.aruco.estimatePoseSingleMarkers(corners, 0.1, mtx, dist)

                rvec = rvec[0][0]
                tvec = tvec[0][0]

                # Get the original marker position in 4x4 matrix representation
                transform_matrix = get_transform_matrix(rvec, tvec)

                # Get the transform matrix we want to apply to the obtained marker position
                # E.g. rotate 180 deg (PI) along Y and then translate -10 cm along X
                relative_transform_matrix_ = relative_transform_matrix([0, np.pi, 0], [axis_x, axis_y, axis_z])

                # Now apply the transform to the original matrix by simply dot multiplying them
                transform_matrix = np.dot(transform_matrix, relative_transform_matrix_)

                # Extract rotation matrix and translation vector out of result and then display
                rmat_relative = transform_matrix[:3, :3]
                tmat_relative = transform_matrix[:3, 3:]
                
                # cv.circle(img,(447,63), 63, (0,0,255), -1)
                
                imgpts, jac = cv.projectPoints(axis, rmat_relative, tmat_relative, mtx, dist)
                print(frame.shape)
                draw(frame, corners, imgpts)

                cv.drawFrameAxes(frame, mtx, dist, rmat_relative, tmat_relative, 0.1)

            # Déplacement en y
            if axis_y >= max_y and plus:
                plus = False
            if axis_y <= min_y and not plus:
                plus = True
            if plus:
                axis_y += 0.01
            else:
                axis_y -= 0.01

            # Déplacement en Z
            if axis_z >= min_z and not plus_z:
                plus_z = True
            if axis_z <= max_z and plus_z:
                plus_z = False

            if plus_z:
                axis_z -= 0.0025
            else:
                axis_z += 0.0025

            cv.imshow('frame', frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()
