# Converting Camera Co-ordinates to Real Co-ordinates to use in Robotics Applications

This project uses PyBullet to simulate a robot moving towards a target position in a 3D environment. The robot's target position is determined using camera calibration techniques, including chessboard corner and marker detection. The simulation ignores the Z-coordinate for movement but displays it for reference.

## Features

- **Camera Calibration**: This method calibrates the camera using chessboard images to compute the camera matrix and distortion coefficients.
- **Marker Detection**: Detects a marker in an image and converts its pixel coordinates to real-world coordinates.
- **Robot Simulation**: Simulates a robot moving towards a target position in a 3D environment using PyBullet.
- **Real-World Coordinates**: Converts pixel coordinates to real-world coordinates using the camera's intrinsic parameters.
- **Dynamic Target Positioning**: The target position is dynamically set based on the detected marker or a default position.

## Prerequisites

Before running the simulation, ensure you have the following installed:

- Python 3.8 or higher
- Required Python packages:
  ```bash
   numpy opencv-python pybullet
  ```

## Project Structure

```
robot-simulation/
├── camera_calibration.py       # Camera calibration and marker detection
├── robot_simulation.py         # Robot simulation logic
├── utils.py                    # Utility functions (e.g., vector normalization)
├── main.py                     # Entry point for the simulation
├── venv_install.sh             # Virtual environment setup script
├── calibration_images/         # Folder containing calibration images
│   ├── chessboard.jpg          # Chessboard image for calibration
│   ├── marker_image.jpg        # Marker image
├── README.md                   # Readme file
```

## Setup

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/your-username/robot-simulation.git
   cd robot-simulation
   ```

2. **Set Up Virtual Environment**:
   
   The project includes a script to set up a Python virtual environment with all required dependencies. To use it:

   ```bash
   # Make the script executable
   chmod +x venv_install.sh

   # Run the script
   ./venv_install.sh
   ```

   This script will:
   - Create a new virtual environment named 'coord_trans'
   - Activate the virtual environment
   - Install all required dependencies
   - Deactivate the virtual environment when done

   To activate the virtual environment manually:
   ```bash
   source coord_trans/bin/activate
   ```

   To deactivate when you're done:
   ```bash
   deactivate
   ```

3. **Prepare Calibration Images**:
   - Place your chessboard images in the `calibration_images/` folder.
   - Ensure the marker image (`marker_image.jpg`) is also in the folder.

4. **Run the Simulation**:
   ```bash
   python3 main.py
   ```

## How It Works

1. **Camera Calibration**:
   - The camera is calibrated using chessboard images to compute the camera matrix and distortion coefficients.
   - The calibration process detects chessboard corners and uses them to calculate the camera's intrinsic parameters.

2. **Marker Detection**:
   - A marker is detected in an image using **blob** detection.
   - The pixel coordinates of the marker are converted to real-world coordinates using the camera's intrinsic parameters.

3. **Robot Simulation**:
   - The robot moves towards the target position in the XY plane, ignoring the Z-coordinate.
   - The simulation uses PyBullet to simulate the robot's movement in a 3D environment.

4. **Real-World Coordinates**:
   - The target position is dynamically set based on the detected marker or a default position.
   - The robot's movement is controlled by calculating the distance to the target in the XY plane.

## Example Output

When running the simulation, you should see an output similar to the following:

```
Current Position: [-0.09912814  0.80586371] (Z: 0.09998981)
Target Position: [-0.09917776  0.80626828] (Z: 2.0)
Distance to Target: 0.0005
Target reached successfully!
```

## Customization

- **Add More Calibration Images**:
  Add more chessboard images to the `calibration_images/` folder and update the `calibration_images` list in `robot_simulation.py`.

- **Adjust Robot Speed**:
  Change the `robot_speed` variable in the `RobotSimulation` class to control the robot's movement speed.
