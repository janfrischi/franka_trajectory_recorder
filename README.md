# Franka Trajectory Recorder

The `franka_trajectory_recorder` package provides tools for recording, playing back, and teleoperating trajectories for the Franka Emika Panda robot. It includes functionality for capturing joint states, controlling the robot's gripper, and saving trajectories in both CSV and HDF5 formats for later use.

## Features
- **Free Movement Mode**: Allows teleoperation of the robot for recording arm and gripper movements.
- **Trajectory Playback**: Plays back recorded trajectories with precise timing and gripper control.
- **Gripper Control**: Supports dynamic gripper actions such as opening, closing, and homing.

## Usage
1. **Free Movement Mode**: Use this mode to record trajectories by teleoperating the robot.
2. **Trajectory Playback**: Replay recorded trajectories to reproduce movements.
3. **Gripper Commands**: Dynamically control the gripper during playback or teleoperation.

## File Formats
- **CSV**: Human-readable format for trajectory data.
- **HDF5**: Efficient binary format for large datasets.

## Requirements
- ROS 2
- Franka Emika Panda robot
- Required ROS 2 packages for Franka control and messaging

## Getting Started
1. Launch the ROS 2 environment.
2. Run the `free_movement_mode.py` script to record trajectories.
3. Use the `trajectory_playback.py` script to replay recorded trajectories.

For more details, refer to the source code and inline documentation.