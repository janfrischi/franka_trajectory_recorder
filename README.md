# Franka Trajectory Recorder

The `franka_trajectory_recorder` package provides comprehensive tools for recording, playing back, and controlling trajectories for the Franka Emika Panda robot. It supports multiple recording modes, file formats, and provides both automated and manual control interfaces for robot manipulation and data collection.

## Package Overview

This package contains several nodes designed for different aspects of trajectory recording and playback:

### Core Nodes

#### 1. `trajectory_recorder` (Original Recorder)
**File**: `trajectory_recorder.py`

The original trajectory recording node that captures robot movements in free movement mode.

**Features**:
- Records joint positions and velocities at sensor callback rate
- Gripper state tracking and control
- Saves data in both CSV and HDF5 formats
- Interactive keyboard controls during recording

**Topics**:
- **Subscribes**: `/joint_states`, `/fr3_gripper/joint_states`
- **Actions**: `/fr3_gripper/homing`, `/fr3_gripper/move`, `/fr3_gripper/grasp`

#### 2. `trajectory_recorder_new` (Advanced Recorder)
**File**: `trajectory_recorder_new.py`

Enhanced trajectory recorder with configurable action modes for different Isaac Sim environments.

**Features**:
- **Dual Action Modes**:
  - `joint`: Records absolute joint positions (for Isaac-Stack-Cube-Franka-v0)
  - `pose`: Records absolute end-effector poses (for Isaac-Stack-Cube-Franka-IK-Abs-v0)
- Fixed 20Hz sampling rate for consistent data collection
- Advanced HDF5 structure compatible with imitation learning frameworks
- Initial state capture and trajectory metadata

**Topics**:
- **Subscribes**: `/joint_states`, `/fr3_gripper/joint_states`, `/franka_robot_state_broadcaster/robot_state`
- **Actions**: `/fr3_gripper/homing`, `/fr3_gripper/move`, `/fr3_gripper/grasp`

**Parameters**:
- `action_mode`: `'joint'` or `'pose'` (default: `'joint'`)

#### 3. `trajectory_playback` (Trajectory Player)
**File**: `trajectory_playback.py`

Plays back recorded trajectories with precise timing and gripper control.

**Features**:
- Supports both HDF5 and CSV trajectory files
- Configurable playback rate (speed control)
- Automatic gripper state transitions
- Real-time joint position publishing

**Topics**:
- **Publishes**: `/trajectory_playback/joint_positions`
- **Actions**: `/fr3_gripper/move`, `/fr3_gripper/grasp`

**Parameters**:
- `file_path`: Path to trajectory file (default: `'~/trajectory.h5'`)
- `playback_rate`: Playback speed multiplier (default: `1.0`)

#### 4. `joint_position_sender` (Manual Control)
**File**: `joint_position_sender.py`

Publishes joint positions for manual robot control or testing.

**Features**:
- Configurable publication rate
- Mode-specific topic selection
- Programmable joint position sequences

**Topics**:
- **Publishes**: `/trajectory_playback/joint_positions` or `/policy_outputs`

**Parameters**:
- `mode`: `'trajectory_playback'` or `'policy_runner'` (default: `'trajectory_playback'`)
- `publish_rate`: Publication frequency in Hz (default: `10.0`)
- `joint_positions`: Array of 7 joint positions (default: `[0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.5]`)

#### 5. `joint_position_gui` (GUI Control)
**File**: `joint_position_gui.py`

Interactive GUI for manual joint position control using sliders.

**Features**:
- Real-time joint position sliders
- Visual feedback of current positions
- Direct publishing to control topics
- PyQt5-based interface

## CLI Commands

### Basic Recording and Playback

#### Trajectory Recording (Original)
```bash
# Basic trajectory recording
ros2 run franka_trajectory_recorder trajectory_recorder
```

#### Advanced Trajectory Recording
```bash
# Record with joint position actions (default)
ros2 run franka_trajectory_recorder trajectory_recorder_new

# Record with joint position actions (explicit)
ros2 run franka_trajectory_recorder trajectory_recorder_new --ros-args -p action_mode:=joint

# Record with end-effector pose actions (for Isaac Sim IK environments)
ros2 run franka_trajectory_recorder trajectory_recorder_new --ros-args -p action_mode:=pose
```

#### Trajectory Playback
```bash
# Basic playback with default settings
ros2 run franka_trajectory_recorder trajectory_playback

# Playback with custom file path
ros2 run franka_trajectory_recorder trajectory_playback --ros-args -p file_path:=~/custom_trajectory.h5

# Playback with custom speed (2x faster)
ros2 run franka_trajectory_recorder trajectory_playback --ros-args -p playback_rate:=2.0

# Playback with both custom path and speed
ros2 run franka_trajectory_recorder trajectory_playback --ros-args -p file_path:=~/my_demo.csv -p playback_rate:=0.5
```

### Manual Control

#### Joint Position Control
```bash
# Basic joint position sender
ros2 run franka_trajectory_recorder joint_position_sender

# Custom mode and rate
ros2 run franka_trajectory_recorder joint_position_sender --ros-args -p mode:=policy_runner -p publish_rate:=20.0

# Custom joint positions
ros2 run franka_trajectory_recorder joint_position_sender --ros-args -p joint_positions:="[0.1, -0.8, 0.2, -2.0, 0.1, 1.2, 0.8]"
```

#### GUI Control
```bash
# Launch interactive GUI
ros2 run franka_trajectory_recorder joint_position_gui
```

### Complete Workflow Examples

#### Recording a Joint-based Trajectory
```bash
# 1. Start the cartesian impedance controller
ros2 launch cartesian_impedance_control cartesian_impedance_controller.launch.py

# 2. Enable free movement mode for teleoperation
ros2 param set /cartesian_impedance_controller free_movement_mode true

# 3. Start recording in joint mode
ros2 run franka_trajectory_recorder trajectory_recorder_new --ros-args -p action_mode:=joint

# 4. Use keyboard controls:
#    - 'r' to start/pause recording
#    - 'f' to finish and save
#    - 'b' to toggle gripper
```

#### Recording a Pose-based Trajectory
```bash
# 1. Start the cartesian impedance controller
ros2 launch cartesian_impedance_control cartesian_impedance_controller.launch.py

# 2. Enable free movement mode
ros2 param set /cartesian_impedance_controller free_movement_mode true

# 3. Start recording in pose mode (for Isaac Sim IK environments)
ros2 run franka_trajectory_recorder trajectory_recorder_new --ros-args -p action_mode:=pose

# 4. Use keyboard controls as above
```

#### Playing Back a Trajectory
```bash
# 1. Start the cartesian impedance controller
ros2 launch cartesian_impedance_control cartesian_impedance_controller.launch.py

# 2. Enable trajectory playback mode
ros2 param set /cartesian_impedance_controller trajectory_playback_mode true

# 3. Start playback
ros2 run franka_trajectory_recorder trajectory_playback --ros-args -p file_path:=~/franka_ros2_ws/src/franka_trajectory_recorder/trajectories/dataset.hdf5
```

## Interactive Controls

### Recording Controls (During Recording Session)
When running either trajectory recorder, use these keyboard commands:
- **`r`** - Start/Pause recording
- **`f`** - Finish recording and save trajectory
- **`b`** - Toggle gripper state (Open/Close)

## File Formats and Output

### Supported File Formats
- **HDF5 (`.h5`)**: Efficient binary format for large datasets, structured for ML frameworks
- **CSV (`.csv`)**: Human-readable format for inspection and debugging

### Output Locations
- **HDF5 Dataset**: `~/franka_ros2_ws/src/franka_trajectory_recorder/trajectories/dataset.hdf5`
- **CSV Files**: `~/franka_ros2_ws/src/franka_trajectory_recorder/trajectories/trajectory.csv`

### HDF5 Structure (Advanced Recorder)
The advanced recorder creates datasets compatible with Isaac Sim environments:

```
/data
├── demo_0, demo_1, ... (trajectory demonstrations)
│   ├── num_samples (number of data points)
│   ├── success (boolean success flag)
│   ├── actions (robot actions - format depends on mode)
│   ├── initial_state (robot and environment initial state)
│   ├── obs (observations during trajectory)
│   └── states (full state information)
```

### Action Formats
- **Joint Mode**: `[joint1, joint2, ..., joint7, gripper_command]` (8 elements)
- **Pose Mode**: `[x, y, z, qw, qx, qy, qz, gripper_command]` (8 elements)

## Integration with Isaac Sim

The package is designed to work with Isaac Sim environments:
- **Isaac-Stack-Cube-Franka-v0**: Use `action_mode:=joint`
- **Isaac-Stack-Cube-Franka-IK-Abs-v0**: Use `action_mode:=pose`

## Requirements

### System Requirements
- ROS 2 (Humble or later)
- Python 3.8+
- Franka Emika Panda robot with ROS 2 drivers

### ROS 2 Dependencies
- `rclpy`
- `sensor_msgs`
- `franka_msgs`
- `std_msgs`

### Python Dependencies
- `numpy`
- `h5py`
- `PyQt5` (for GUI)
- `termios`, `tty`, `select` (for keyboard input)

### Hardware Requirements
- Franka Emika Panda robot
- Franka Hand gripper
- Proper network connection to robot controller

## Troubleshooting

### Common Issues

1. **Action servers not available**: Ensure the Franka gripper driver is running
2. **Permission denied for keyboard input**: Run the recording nodes with appropriate terminal permissions
3. **File path errors**: Check that the trajectory output directories exist
4. **Joint state not received**: Verify the robot controller and joint state publisher are active

### Verification Commands
```bash
# Check active topics
ros2 topic list

# Monitor joint states
ros2 topic echo /joint_states

# Check gripper action servers
ros2 action list
```

## Advanced Usage

### Custom Trajectory Processing
The HDF5 files can be loaded and processed using standard Python libraries:

```python
import h5py
import numpy as np

# Load trajectory data
with h5py.File('dataset.hdf5', 'r') as f:
    demo_0_actions = f['data/demo_0/actions'][:]
    demo_0_observations = f['data/demo_0/obs/joint_pos'][:]
```

### Integration with Learning Frameworks
The structured HDF5 format is compatible with:
- Imitation learning frameworks
- Behavior cloning pipelines
- Reinforcement learning environments
- Isaac Sim simulation workflows

For detailed implementation examples, refer to the source code and inline documentation.