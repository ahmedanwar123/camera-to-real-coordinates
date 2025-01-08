"""
Robot Simulation with Target Detection and Navigation Made by:
 - Ahmed Anwar Mazhar
    - ID:120210007
    - Email: ahmed.gad@ejust.ed.eg , ahmed.anwar2003@gmail.com
 - Moustafa Mohamed Rezk
    - ID:120210004
    - Email: moustafa.rezk@ejust.ed.eg

This project is a robotic simulation system designed to demonstrate a robot's ability to detect a target using camera calibration and marker detection, and then navigate toward it in a simulated environment.

Key Features:
- Camera Calibration: Utilizes a chessboard-based calibration method to compute the camera's intrinsic and extrinsic parameters for accurate target localization.
- Marker Detection: Detects a predefined marker in a given image, converting pixel coordinates into real-world 3D coordinates.
- PyBullet Simulation: Integrates with the PyBullet physics engine to create a virtual environment where a robot operates and interacts with the simulation world.
- Robot Navigation: Implements a motion-planning algorithm to guide the robot to the target's position.

Structure:
1. main.py: The main entry point for the simulation, where the RobotSimulation class is initialized and executed.
2. camera_calibration.py: Contains the CameraCalibration class to handle camera calibration and marker detection.
3. utils.py: Provides utility functions for vector normalization, distance calculation, and coordinate transformations.

Simulation Workflow:
1. The system calibrates the camera using a chessboard image.
2. It detects the marker in a separate image and calculates the target's real-world position.
3. The robot is loaded into a PyBullet simulation environment with a world (flat plane) and visualized target.
4. The robot moves toward the target and stopp when it reachs the destination.
s
Dependencies:
- Python libraries: pybullet, numpy, opencv-python, and typing.
- Simulation files: URDF models for the robot and the plane.
- Calibration images: A chessboard image and a marker image.

Usage:
Run main.py to start the simulation and observe the robot's behavior as it navigates to the detected target.
"""

from camera_calibration import CameraCalibration
from robot_simulation import RobotSimulation

if __name__ == "__main__":
    simulation = RobotSimulation()
    simulation.run_simulation()
