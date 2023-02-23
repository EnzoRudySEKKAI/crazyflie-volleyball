import numpy as np
import cv2 as cv



# aruco detection
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
parameters = cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(dictionary, parameters)

# camera calibration
with np.load('calibration.npz') as X:
    mtx, dist, _, _ = [X[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]

axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]]).reshape(-1, 3)

cap = cv.VideoCapture(0)
axisx = 0
axisy = 0
axisz = 0
while True:
    # X = 0,05 = 10cm
    # Y = 0,05 = 10cm
    # Z = 0,05 = 10cm
    _, frame = cap.read()
    commade = input("Appuyer sur une touche pour continuer")
    if commade == "q":
        axisx += -0.05
    if commade == "d":
        axisx += 0.05
    if commade == "z":
        axisy += -0.05
    if commade == "s":
        axisy += 0.05
    if commade == "a":
        axisz += -0.05
    if commade == "e":
        axisz += 0.05

    # frame = cv.resize(frame, None, fx=0.7, fy=0.7, interpolation=cv.INTER_AREA)
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    # aruco detection
    corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
    if np.all(ids != None):
        rvec, tvec, _ = cv.aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
        print(ids)
        for i in range(0, ids.size):
            print(ids[i])
            print(rvec[i], tvec[i])
            tvec[i][0][0] = tvec[i][0][0]+axisx
            tvec[i][0][1] = tvec[i][0][1]+axisy
            tvec[i][0][2] = tvec[i][0][2]+axisz

            cv.drawFrameAxes(frame, mtx, dist, rvec[i], tvec[i], 0.05)

    cv.imshow('frame', frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
