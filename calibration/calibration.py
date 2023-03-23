import numpy as np
import cv2 as cv
import glob

CAMERA_MODEL = 'C270'

def calibrate():
    obj_points = []
    img_points = []

    # Chessboard size 10*7
    chessboard_size = (9, 6)
    # Chessboard size 9*7
    #chessboard_size = (8, 6)
    
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Chessboard size 10*7
    objp = np.zeros((6 * 9, 3), np.float32)
    # Chessboard size 9*7
    #objp = np.zeros((6 * 8, 3), np.float32)

    # Chessboard size 10*7
    objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)
    # Chessboard size 9*7
    #objp[:, :2] = np.mgrid[0:8, 0:6].T.reshape(-1, 2)
    
    images = glob.glob(CAMERA_MODEL+'/*.jpg')
    
    for fname in images:
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        ret, corners = cv.findChessboardCorners(gray, chessboard_size, None)
        print(ret)
        if ret:
            obj_points.append(objp)
            corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            img_points.append(corners2)
            cv.drawChessboardCorners(img, chessboard_size, corners2, ret)
            cv.imshow('img', img)
            cv.waitKey(500)
    
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)
    mean_error = 0
    for i in range(len(obj_points)):
        imgpoints2, _ = cv.projectPoints(obj_points[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv.norm(img_points[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
        mean_error += error

    print("total error: {}".format(mean_error / len(obj_points)))
    
    np.savez('calibration.npz', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)


if __name__ == "__main__":
    calibrate()
