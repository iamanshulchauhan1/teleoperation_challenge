# Vision Teleoperation

---

## About The Project

This project focuses on vision-based teleoperation for dual-arm robotic systems using ROS 2 Humble. The goal is to enable intuitive control of robot end-effectors through human gestures and posture recognition. The system consists of multiple components, including robot arm visualization, motion planning with self-collision avoidance, real-time message publishing, and a teleoperation node. The dual-arm setup features 6-DOF manipulators with two-finger grippers, visualized in RViz rather than a simulator. The teleoperation node processes camera images to capture hand gestures and body posture, generating and publishing the target poses for the robotic arms. Since a depth camera is not required, movement is limited to a plane with 2D translation and 1D rotation.

---

## Built With

- **ROS 2 Humble**
- **MoveIt 2**
- **MediaPipe**
- **OpenCV (cv2)**

---

## Getting Started

### Prerequisites

- Ensure **Docker** is installed on your system.

### Installation



1. Navigate to the project directory:
   ```sh
   cd challenge_ws
   ```
2. Build the Docker image:
   ```sh
    docker build -t telechallenge_image .
   ```
3. Open the project in a development container using **DevContainer** in VS Code.
   - Ensure you have the **Dev Containers extension** installed.
   - Open the project folder and reopen it in a container.
   - All dependencies will be automatically installed inside the container.

---

## Usage

After opening the development container, you can run all necessary commands from a single script.

Running the Teleoperation System

Execute the following script to launch all required components:
```sh
   ./start_teleop.sh
```


What start_teleop.sh Does

This script performs the following actions:

- Builds the ROS 2 workspace.
- Sources the required setup files.
- Launches the Move Group for motion planning.
- Launches the left and right arm servo nodes.
- Switches the command type for both arms.
- Starts the teleoperation node for user control.



## Gripper Control
   The gripper is controlled by detecting the touching of the tip of the thumb and index finger. When the fingers touch, the gripper closes; otherwise, it remains open.

## Acknowledgments

This project utilizes the following libraries:

- [ROS 2](https://docs.ros.org/en/humble/index.html)
- [MoveIt 2](https://moveit.ros.org/)
- [MediaPipe](https://developers.google.com/mediapipe)
- [OpenCV](https://opencv.org/)

