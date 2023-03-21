# open the webcam and display the live video

import cv2
import numpy as np

cap = cv2.VideoCapture(0)
# deactivates autofocus
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)

while(True):
    ret, frame = cap.read()
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


