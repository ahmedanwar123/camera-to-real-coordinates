import cv2
import numpy as np
from typing import Optional, Tuple, List


class CameraCalibration:
    def __init__(
        self,
        chessboard_size: Tuple[int, int] = (9, 6),
        frame_size: Tuple[int, int] = (640, 480),
    ):
        self.chessboard_size: Tuple[int, int] = chessboard_size
        self.frame_size: Tuple[int, int] = frame_size
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None

    def set_camera_intrinsics(self, fx: float, fy: float, cx: float, cy: float) -> None:
        """Set the camera matrix with intrinsic parameters."""
        self.camera_matrix = np.array(
            [[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64
        )
        print("Camera matrix set manually:\n", self.camera_matrix)

    def calibrate_camera_from_images(
        self, image_paths: List[str], grid_sizes: List[Tuple[int, int]]
    ) -> None:
        """
        Calibrate the camera using multiple images of chessboards with varying grid sizes.

        Args:
            image_paths: List of paths to the chessboard images.
            grid_sizes: List of grid sizes (width, height) for each image.
        """
        if len(image_paths) != len(grid_sizes):
            raise ValueError(
                "The number of image paths must match the number of grid sizes."
            )

        objpoints: list = []  # 3D points in real world space
        imgpoints: list = []  # 2D points in image plane

        for image_path, chessboard_size in zip(image_paths, grid_sizes):
            # Prepare object points for the current grid size
            objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
            objp[:, :2] = np.mgrid[
                0 : chessboard_size[0], 0 : chessboard_size[1]
            ].T.reshape(-1, 2)

            # Load the image
            image = cv2.imread(image_path)
            if image is None:
                print(f"Error: Unable to load image at {image_path}.")
                continue

            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

            if ret:
                objpoints.append(objp)
                imgpoints.append(corners)
                cv2.drawChessboardCorners(image, chessboard_size, corners, ret)
                cv2.imshow("Chessboard", image)
                cv2.waitKey(1500)
            else:
                print(f"Chessboard corners not found in the image: {image_path}")

        cv2.destroyAllWindows()

        if len(objpoints) > 0:
            # Calibrate the camera using all detected points
            ret, self.camera_matrix, self.dist_coeffs = cv2.calibrateCamera(
                objpoints, imgpoints, self.frame_size, None, None
            )

            if ret:
                print("Camera matrix after calibration:\n", self.camera_matrix)
                print("Distortion coefficients:\n", self.dist_coeffs)
            else:
                print("Calibration failed.")
        else:
            print("No valid chessboard images found for calibration.")

    def calibrate_camera_from_single_image(self, image_path: str) -> None:
        """Calibrate the camera using a single image of a chessboard."""
        objp = np.zeros(
            (self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32
        )
        objp[:, :2] = np.mgrid[
            0 : self.chessboard_size[0], 0 : self.chessboard_size[1]
        ].T.reshape(-1, 2)

        objpoints: list = []  # 3d points in real world space
        imgpoints: list = []  # 2d points in image plane

        image = cv2.imread(image_path)
        if image is None:
            print(f"Error: Unable to load image at {image_path}.")
            return

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)

        if ret:
            objpoints.append(objp)
            imgpoints.append(corners)
            cv2.drawChessboardCorners(image, self.chessboard_size, corners, ret)
            cv2.imshow("Chessboard", image)
            cv2.waitKey(1500)
            cv2.destroyAllWindows()

            # Calibrate the camera
            ret, self.camera_matrix, self.dist_coeffs, rvecs, tvecs = (
                cv2.calibrateCamera(objpoints, imgpoints, self.frame_size, None, None)
            )

            if ret:
                print("Camera matrix after calibration:\n", self.camera_matrix)
                print("Distortion coefficients:\n", self.dist_coeffs)
            else:
                print("Calibration failed.")
        else:
            print("Chessboard corners not found in the image.")

    def detect_marker(self, image_path: str) -> Optional[Tuple[float, float]]:
        """Detect the marker in the provided image."""
        image = cv2.imread(image_path)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        detector = cv2.SimpleBlobDetector_create()
        keypoints = detector.detect(gray)

        im_with_keypoints = cv2.drawKeypoints(
            image,
            keypoints,
            np.array([]),
            (0, 0, 255),
            cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
        )

        cv2.imshow("Keypoints", im_with_keypoints)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        if len(keypoints) > 0:
            pixel_coords = keypoints[0].pt
            print("Pixel coordinates of the marker:", pixel_coords)
            return pixel_coords
        else:
            print("No marker detected.")
            return None

    def pixel_to_world(
        self, pixel_coords: Tuple[float, float], distance: float
    ) -> Optional[Tuple[float, float, float]]:
        """Convert pixel coordinates to real-world coordinates."""
        if self.camera_matrix is None:
            print(
                "Error: Camera calibration not done. Please calibrate the camera first."
            )
            return None

        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        u, v = pixel_coords

        x_norm = (u - cx) / fx
        y_norm = (v - cy) / fy

        X = distance * x_norm
        Y = distance * y_norm
        Z = distance

        print("Real-world coordinates (X, Y, Z):", X, Y, Z)
        return X, Y, Z
