from camera_calibration import CameraCalibration
from image_processing import ImageProcessing
from coordinate_mapping import CoordinateMapping
from robot_control import RobotControl
from robot_simulation import RobotSimulation
import cv2


def test_chessboard_detection(image_path, chessboard_size):
    img = cv2.imread(image_path)
    if img is None:
        print("Error: Image not found or unable to load.")
        return

    # Resize image to a reasonable size (e.g., width of 800 pixels)
    scale = 800 / img.shape[1]
    width = int(img.shape[1] * scale)
    height = int(img.shape[0] * scale)
    img = cv2.resize(img, (width, height))

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        print("Chessboard detected!")
        cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
        cv2.imshow("Chessboard", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("Chessboard not detected. Check the image and chessboard size.")


def main():
    # Test chessboard detection
    test_chessboard_detection("calibration_images/1pattern.png", (9, 6))

    # Camera calibration
    chessboard_size = (9, 6)
    frame_size = (640, 480)
    calibration = CameraCalibration(chessboard_size, frame_size)
    if not calibration.calibrate("calibration_images/*.jpg"):
        print("Calibration failed. Exiting.")
        return

    camera_matrix = calibration.get_camera_matrix()
    dist = calibration.get_distortion_coefficients()
    rvecs = calibration.get_rotation_vectors()
    tvecs = calibration.get_translation_vectors()

    # Image processing
    image_processor = ImageProcessing()
    corners, img = image_processor.detect_keypoints(
        "calibration_images/20250103_182327.jpg"
    )

    if corners is not None:
        pixel_coords = image_processor.extract_pixel_coordinates(corners)

        # Coordinate mapping
        mapper = CoordinateMapping(camera_matrix, dist, rvecs[0], tvecs[0])
        world_coords = []

        # Robot control
        initial_position = [0, 0]  # Changed to 2D coordinates
        robot = RobotControl(initial_position)

        # Robot simulation
        sim = RobotSimulation()

        for coord in pixel_coords:
            world_coord = mapper.pixel_to_world(coord)
            world_coords.append(world_coord)
            print(f"Converted coordinates: {world_coord}")
            robot.move_to_position(world_coord)
            sim.update_arm(world_coord)

        sim.show()


if __name__ == "__main__":
    main()
