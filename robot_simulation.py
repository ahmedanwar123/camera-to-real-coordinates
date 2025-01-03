import pybullet as p
import pybullet_data
import numpy as np
import time
from camera_calibration import CameraCalibration
from utils import normalize_vector, calculate_distance
from typing import Optional


class RobotSimulation:
    def __init__(self) -> None:
        self.robotId: Optional[int] = None
        self.robot_speed: float = 0.2
        self.debug_lines: list = []
        self.target_position: np.ndarray = np.array([1.0, 1.0, 1.0])

        # Create camera calibration instance
        calibrator = CameraCalibration()

        # Set the camera matrix using the new intrinsic parameters
        fx = 3000  # Focal length in x (pixels)
        fy = 3000  # Focal length in y (pixels)
        cx = 2000  # Principal point x (pixels)
        cy = 1500  # Principal point y (pixels)

        calibrator.set_camera_intrinsics(fx, fy, cx, cy)

        # Set the distortion coefficients with the new values
        calibrator.dist_coeffs = np.array(
            [
                [-0.1, 0.01, 0.001, -0.001, 0.005]  # k1, k2, p1, p2, k3
            ]
        )

        # Detect marker and get coordinates
        marker_image_path = "calibration_images/istockphoto-1394093629-612x612.jpg"
        pixel_coords = calibrator.detect_marker(marker_image_path)

        if pixel_coords:
            # Convert to real world coordinates
            distance = 2.0  # Increased distance for farther target
            X, Y, Z = calibrator.pixel_to_world(pixel_coords, distance)
            self.target_position = np.array([X, Y, Z])
            print(f"Using detected target position: {self.target_position}")
        else:
            print("Warning: No marker detected, using default position")
            self.target_position = np.array([1.0, 2.0, 0.0])

    def setup_simulation(self) -> bool:
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        p.resetDebugVisualizerCamera(
            cameraDistance=5.0,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0],
        )
        p.setGravity(0, 0, -9.81)

        try:
            self.planeId = p.loadURDF("plane.urdf")
            self.robotId = p.loadURDF("simple_robot.urdf", [0, 0, 0.1])
            print("Successfully loaded robot URDF")
        except p.error as e:
            print(f"Failed to load robot URDF: {e}")
            return False

        start_position = [0, 0, 0.1]
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        p.resetBasePositionAndOrientation(
            self.robotId, start_position, start_orientation
        )
        self.visualize_target()
        return True

    def visualize_target(self) -> None:
        visual_shape_id = p.createVisualShape(
            shapeType=p.GEOM_SPHERE,
            radius=0.1,
            rgbaColor=[1, 0, 0, 0.7],
        )
        target_pos = [self.target_position[0], self.target_position[1], 0.1]
        p.createMultiBody(
            baseMass=0, baseVisualShapeIndex=visual_shape_id, basePosition=target_pos
        )

    def update_debug_lines(self) -> None:
        for line in self.debug_lines:
            p.removeUserDebugItem(line)
        self.debug_lines.clear()

        current_position, _ = p.getBasePositionAndOrientation(self.robotId)
        line_id = p.addUserDebugLine(
            current_position,
            [self.target_position[0], self.target_position[1], 0.1],
            [0, 1, 0],
            lineWidth=2.0,
        )
        self.debug_lines.append(line_id)

    def move_robot_to_target(self) -> bool:
        current_position, _ = p.getBasePositionAndOrientation(self.robotId)
        current_position = np.array(current_position)

        direction = self.target_position - current_position
        direction[2] = 0  # Ignore Z for horizontal movement
        distance = calculate_distance(current_position, self.target_position)

        print(f"Current Position: {current_position}")
        print(f"Target Position: {self.target_position}")
        print(f"Distance to Target: {distance}")

        if distance > 0.05:  # Adjust threshold for stopping
            direction_normalized = normalize_vector(direction)
            velocity = direction_normalized * self.robot_speed
            p.resetBaseVelocity(
                self.robotId, linearVelocity=velocity, angularVelocity=[0, 0, 0]
            )
            return False
        else:
            p.resetBaseVelocity(
                self.robotId, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, 0]
            )
            return True

    def run_simulation(self) -> None:
        if not self.setup_simulation():
            print("Failed to set up simulation")
            return

        print("Starting simulation - press Ctrl+C to stop")
        try:
            target_reached = False
            step = 0
            while not target_reached and step < 5000:
                target_reached = self.move_robot_to_target()
                self.update_debug_lines()
                p.stepSimulation()
                time.sleep(1.0 / 40.0)  # Slower simulation for visual clarity

                if step % 100 == 0:
                    pos, _ = p.getBasePositionAndOrientation(self.robotId)
                    print(f"Step {step}: Robot position = {pos}")
                    dist = np.linalg.norm(np.array(pos[:2]) - self.target_position[:2])
                    print(f"Distance to target: {dist:.3f}")

                step += 1

            if target_reached:
                print("Target reached successfully!")
            else:
                print("Simulation ended before reaching target")

        except KeyboardInterrupt:
            print("\nSimulation interrupted by user")
        finally:
            p.disconnect()
            print("Simulation ended")
