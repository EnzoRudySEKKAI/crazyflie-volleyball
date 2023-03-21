import numpy as np
import cv2 as cv
import glob

CAMERA_MODEL = 'C920'


# open camera and take picture of chessboard every 5 seconds for 30 seconds and show the webcam live
def take_pictures():
    cap = cv.VideoCapture(0)
    # deactivates autofocus
    cap.set(cv.CAP_PROP_AUTOFOCUS, 0)
    cv.waitKey(5000)
    for i in range(0, 10):
        ret, frame = cap.read()
        cv.imshow('frame', frame)
        cv.waitKey(5000)
        cv.imwrite(CAMERA_MODEL + '/image' + str(i) + '.jpg', frame)
    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    take_pictures()