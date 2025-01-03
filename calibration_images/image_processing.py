import cv2
import numpy as np


class ImageProcessing:
    def __init__(self):
        pass

    def detect_keypoints(self, image_path):
        img = cv2.imread(image_path)
        if img is None:
            print("Error: Image not found or unable to load.")
            return None, None

        img = cv2.resize(img, (0, 0), fx=0.25, fy=0.25)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners = cv2.goodFeaturesToTrack(gray, 100, 0.5, 10)

        if corners is None:
            print("No corners detected. Adjust parameters or check the image.")
            return None, None

        corners = np.int32(corners)
        return corners, img

    def extract_pixel_coordinates(self, corners):
        return [tuple(corner[0]) for corner in corners]
