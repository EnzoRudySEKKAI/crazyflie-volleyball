from math import cos, sin

import numpy as np
import cv2 as cv



# aruco detection
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
parameters = cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(dictionary, parameters)

# camera calibration
with np.load('calibration.npz') as X:
    mtx, dist, _, _ = [X[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]


def getTranslationMatrix(tvec):
    T = np.identity(n=4)
    T[0:3, 3] = tvec
    return T

def getTransformMatrix(rvec, tvec):
    mat = getTranslationMatrix(tvec)
    mat[:3, :3] = cv.Rodrigues(rvec)[0]
    return mat

def relativeTransformMatrix(rotation, translation):
    xC, xS = np.cos(rotation[0]), np.sin(rotation[0])
    yC, yS = np.cos(rotation[1]), np.sin(rotation[1])
    zC, zS = np.cos(rotation[2]), np.sin(rotation[2])
    dX = translation[0]
    dY = translation[1]
    dZ = translation[2]
    Translate_matrix = np.array([[1, 0, 0, dX],
                                 [0, 1, 0, dY],
                                 [0, 0, 1, dZ],
                                 [0, 0, 0, 1]])
    Rotate_X_matrix = np.array([[1, 0, 0, 0],
                                [0, xC, -xS, 0],
                                [0, xS, xC, 0],
                                [0, 0, 0, 1]])
    Rotate_Y_matrix = np.array([[yC, 0, yS, 0],
                                [0, 1, 0, 0],
                                [-yS, 0, yC, 0],
                                [0, 0, 0, 1]])
    Rotate_Z_matrix = np.array([[zC, -zS, 0, 0],
                                [zS, zC, 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])
    return np.dot(Rotate_Z_matrix, np.dot(Rotate_Y_matrix, np.dot(Rotate_X_matrix, Translate_matrix)))


axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]]).reshape(-1, 3)

cap = cv.VideoCapture(0)
cpt = 0
test = -1
axisx = 0.2
axisy = 0
axisz =-0.2
while True:
    _, frame = cap.read()



    # frame = cv.resize(frame, None, fx=0.7, fy=0.7, interpolation=cv.INTER_AREA)
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    # aruco detection
    corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
    if np.all(ids != None):
        rvec, tvec, _ = cv.aruco.estimatePoseSingleMarkers(corners, 0.1, mtx, dist)
        print(ids)

        rvec = rvec[0][0]
        tvec = tvec[0][0]

        # Get the original marker position in 4x4 matrix representation
        transformMatrix = getTransformMatrix(rvec, tvec)

        # Get the transform matrix we want to apply to the obtained marker position
        # E.g. rotate 180 deg (PI) along Y and then translate -10 cm along X
        relativeTransformMatrice = relativeTransformMatrix([0, np.pi, 0], [axisx, axisy, axisz])

        # Now apply the transform to the original matrix by simply dot multiplying them
        transformMatrix = np.dot(transformMatrix, relativeTransformMatrice)

        # Extract rotation matrix and translation vector out of result and then display
        rmat = transformMatrix[:3, :3]
        tmat = transformMatrix[:3, 3:]
        print(tmat)

        cv.drawFrameAxes(frame, mtx, dist, rmat, tmat, 0.1)
        # imgpts, jac = cv.projectPoints(axis, rvec[i], tvec[i], mtx, dist)
        # print(frame.shape)
        # draw(frame, corners, imgpts)


    cv.imshow('frame', frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
