import numpy as np
import cv2 as cv

with np.load('calibration.npz') as X:
    mtx, dist, _, _ = [X[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]


def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    print(tuple(imgpts[0].ravel()))
    img = cv.line(img, (int(corner[0]), int(corner[1])),
                  (int(tuple(imgpts[0].ravel())[0]), int(tuple(imgpts[0].ravel())[1])), (255, 0, 0), 5)
    img = cv.line(img, (int(corner[0]), int(corner[1])),
                  (int(tuple(imgpts[1].ravel())[0]), int(tuple(imgpts[1].ravel())[1])), (0, 255, 0), 5)
    img = cv.line(img, (int(corner[0]), int(corner[1])),
                  (int(tuple(imgpts[2].ravel())[0]), int(tuple(imgpts[2].ravel())[1])), (0, 0, 255), 5)
    return img


criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((6 * 7, 3), np.float32)
objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)
axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]]).reshape(-1, 3)

cap = cv.VideoCapture(0)
cap.set(3, 1280)
cap.set(4, 720)

while True:
    _, frame = cap.read()
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # undistort
    h, w = gray.shape[:2]
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    dst = cv.undistort(gray, mtx, dist, None, newcameramtx)
    # crop the image
    x, y, w, h = roi
    dst = dst[y:y + h, x:x + w]
    frame = dst
    # frame=gray

    ret, corners = cv.findChessboardCorners(frame, (7, 6), None)
    if ret:
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        ret, rvecs, tvecs = cv.solvePnP(objp, corners2, mtx, dist)
        imgpts, jac = cv.projectPoints(axis, rvecs, tvecs, mtx, dist)
        frame = draw(frame, corners2, imgpts)
    cv.imshow('frame', frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
