import cv2
import numpy as np

# Définition des paramètres du marqueur ArUco
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

with np.load('calibration.npz') as X:
    mtx, dist, _, _ = [X[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]

# Définition de la couleur de la balle (ici rouge)
ball_color = (0, 0, 255)

# Ouverture de la webcam
cap = cv2.VideoCapture(0)

while True:
    # Lecture d'une image depuis la webcam
    ret, frame = cap.read()

    # Détection des marqueurs ArUco dans l'image
    corners, ids, rejected = detector.detectMarkers(frame)

    # Si au moins un marqueur a été détecté, dessiner une balle sur celui-ci
    if ids is not None:
        # Récupération des coins du marqueur détecté
        marker_corners = corners[0][0]

        # Calcul de la position et de l'orientation du marqueur
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers([marker_corners], 0.05, mtx, dist)
        rotM = cv2.Rodrigues(rvec)[0]

        # Dessiner un cercle dans l'espace 3D sur le marqueur détecté
        center_3d = np.array([[0, 0, 0]], dtype=np.float32)
        center_3d[0][1] = -0.025  # La balle est située légèrement en dessous du marqueur
        center_3d = np.matmul(center_3d, rotM.T) + tvec
        center_2d, _ = cv2.projectPoints(center_3d, rvec, tvec, mtx, dist)
        center_2d = center_2d.astype(int)[0]
        radius = int(min(marker_corners[1, 0] - marker_corners[0, 0], marker_corners[3, 1] - marker_corners[0, 1]) / 2)

        # Vérifier que center_2d est une liste/tuple de longueur 2
        if len(center_2d) == 2:
            cv2.circle(frame, tuple(center_2d), radius, ball_color, -1)

    # Afficher l'image résultante dans une fenêtre
    cv2.imshow('Frame', frame)

    # Attendre une touche pour quitter
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Fermer la webcam et détruire les fenêtres
cap.release()
cv2.destroyAllWindows()
