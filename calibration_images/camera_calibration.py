import numpy as np
import cv2
import glob


class CameraCalibration:
    def __init__(self, chessboard_size, frame_size):
        self.chessboard_size = chessboard_size
        self.frame_size = frame_size
        self.camera_matrix = None
        self.dist = None
        self.rvecs = None
        self.tvecs = None

    def calibrate(self, images_path):
        objp = np.zeros(
            (self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32
        )
        objp[:, :2] = np.mgrid[
            0 : self.chessboard_size[0], 0 : self.chessboard_size[1]
        ].T.reshape(-1, 2)

        objpoints = []
        imgpoints = []

        images = glob.glob(images_path)
        if not images:
            print("Error: No calibration images found. Check the path and file format.")
            return False

        for fname in images:
            img = cv2.imread(fname)
            if img is None:
                print(f"Warning: Unable to load image {fname}. Skipping.")
                continue

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)

            if ret:
                objpoints.append(objp)
                imgpoints.append(corners)

        if not objpoints:
            print("Error: No valid calibration images with chessboard patterns found.")
            return False

        ret, self.camera_matrix, self.dist, self.rvecs, self.tvecs = (
            cv2.calibrateCamera(objpoints, imgpoints, self.frame_size, None, None)
        )
        return ret

    def get_camera_matrix(self):
        return self.camera_matrix

    def get_distortion_coefficients(self):
        return self.dist

    def get_rotation_vectors(self):
        return self.rvecs

    def get_translation_vectors(self):
        return self.tvecs
