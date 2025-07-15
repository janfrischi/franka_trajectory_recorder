# Franka Trajectory Recorder

A comprehensive ROS2 package for recording expert demonstrations, analyzing trajectory following performance, and creating datasets with real robot dynamics for the Franka Emika Panda robot. **This package is specifically designed for creating imitation learning datasets for NVIDIA IsaacLab environments.**

## Package Overview

This package provides a complete workflow for collecting and analyzing robot trajectories for imitation learning in NVIDIA IsaacLab:

1. **Record Expert Demonstrations** - Capture human demonstrations on the real Franka Robot in Free-Float mode
2. **Playback & Record Dynamics** - Play back trajectories while recording actual robot motion
3. **Replace Actions with Real Dynamics** - Update datasets with real robot behavior
4. **Analyze Trajectory Performance** - Visualize tracking errors and performance metrics

**Primary Use Case:** Generate high-quality expert demonstration datasets for training imitation learning policies in NVIDIA IsaacLab simulation environments.

## Core Workflow

### 1. Record Expert Demonstrations

Record human demonstrations using the trajectory recorder for imitation learning:

```bash
# Start the cartesian impedance controller
ros2 launch cartesian_impedance_control cartesian_impedance_controller.launch.py

# Enable free movement mode for teleoperation
ros2 param set /cartesian_impedance_controller free_movement_mode true

# Record demonstrations (pose mode recommended for imitation learning environements)
ros2 run franka_trajectory_recorder trajectory_recorder --ros-args -p action_mode:=pose -p object_mode:=manual
```

**Interactive Controls:**
- `r` - Start/pause recording
- `f` - Finish and save trajectory
- `b` - Toggle gripper (open/close)
- `o` - Set custom object positions (manual mode only)

**Output:** 
- `~/franka_ros2_ws/src/franka_trajectory_recorder/trajectories/dataset.hdf5` - Expert demonstrations for IsaacLab
- `~/franka_ros2_ws/src/franka_trajectory_recorder/trajectories/trajectory.csv` - Human-readable format

### 2. Playback Trajectories & Record Real Dynamics

Play back expert demonstrations while recording actual robot motion to capture tracking errors and real dynamics:

```bash
# Enable trajectory playback mode
ros2 ros2 param set /cartesian_impedance_controller imitation_learning_mode true

# Play back all demos and record actual robot dynamics
ros2 run franka_trajectory_recorder multi_trajectory_playback_with_recording \
    --ros-args \
    -p file_path:=~/franka_ros2_ws/src/franka_trajectory_recorder/trajectories/dataset.hdf5 \
    -p control_mode:=pose \
    -p record_data:=true \
    -p auto_next_demo:=true \
    -p pause_between_demos:=3.0
```

**Output:**
- `~/multi_trajectory_comparison/session_YYYYMMDD_HHMMSS/` - Session directory containing:
  - Individual demo recordings: `demo_X_recording_HHMMSS.csv`
  - Comparison data: `demo_X_comparison_HHMMSS.csv`
  - Combined data: `all_demos_recording.csv`, `all_demos_comparison.csv`
  - Session summary: `session_summary.txt`

### 3. Replace Expert Actions with Real Dynamics

Replace the expert demonstrations with real robot dynamics to create a more realistic dataset for imitation learning:

```bash
# Replace expert actions with real robot dynamics
python3 ~/franka_ros2_ws/src/franka_trajectory_recorder/franka_trajectory_recorder/replace_actions_with_dynamics.py \
    --hdf5 ~/franka_ros2_ws/src/franka_trajectory_recorder/trajectories/dataset.hdf5 \
    --csv ~/multi_trajectory_comparison/session_YYYYMMDD_HHMMSS/all_demos_recording.csv \
    --output ~/franka_ros2_ws/src/franka_trajectory_recorder/trajectories/dataset_real_dynamics.hdf5 \
    --mode pose
```

**Output:**
- `dataset_real_dynamics.hdf5` - Updated dataset with real robot dynamics for improved IsaacLab training

### 4. Analyze Trajectory Performance

Visualize trajectory following performance and tracking errors:

```bash
# Analyze individual demo performance
python3 ~/franka_ros2_ws/src/franka_trajectory_recorder/franka_trajectory_recorder/trajectory_comparison.py \
    --csv ~/multi_trajectory_comparison/session_YYYYMMDD_HHMMSS/demo_0_comparison_HHMMSS.csv \
    --show --stats

# Analyze overall session performance
python3 ~/franka_ros2_ws/src/franka_trajectory_recorder/franka_trajectory_recorder/trajectory_comparison.py \
    --csv ~/multi_trajectory_comparison/session_YYYYMMDD_HHMMSS/all_demos_comparison.csv \
    --show --output ~/analysis_results
```

**Features:**
- 3D trajectory visualization (reference vs actual)
- Position and orientation error analysis
- Statistical performance metrics
- Interactive Plotly dashboards

## Key Parameters

### Recording Parameters
- `action_mode`: `'joint'` or `'pose'` (default: `'joint'`)
- `object_mode`: `'fixed'` or `'manual'` (default: `'fixed'`)

### Playback Parameters
- `control_mode`: `'joint'` or `'pose'` (controls robot)
- `playback_rate`: Speed multiplier (default: `1.0`)
- `auto_next_demo`: Automatically advance to next demo
- `pause_between_demos`: Seconds between demos

## File Formats & Structure

### HDF5 Dataset Structure
```
/data
├── demo_0, demo_1, ... (demonstrations)
│   ├── num_samples (trajectory length)
│   ├── success (boolean)
│   ├── actions (robot actions - joint or pose)
│   ├── initial_state (robot & environment state)
│   │   ├── articulation/robot (joint positions, poses)
│   │   └── rigid_object (cube positions & orientations)
│   ├── obs (observations during trajectory)
│   └── states (full state information)
```

### Action Formats
- **Joint Mode**: `[joint1, joint2, ..., joint7, gripper_command]` (8 elements)
- **Pose Mode**: `[x, y, z, qw, qx, qy, qz, gripper_command]` (8 elements)

### Output Files
- **Expert Recordings**: `dataset.hdf5`, `trajectory.csv`
- **Dynamics Data**: `*_recording.csv` (actual robot motion)
- **Comparison Data**: `*_comparison.csv` (reference vs actual tracking)
- **Real Dynamics Dataset**: `dataset_real_dynamics.hdf5`

## Integration with NVIDIA IsaacLab

The package creates datasets specifically formatted for NVIDIA IsaacLab imitation learning environments:

- **Isaac-Stack-Cube-Franka-v0**: Use `action_mode:=joint` for joint-space control
- **Isaac-Stack-Cube-Franka-IK-Abs-v0**: Use `action_mode:=pose` for end-effector control

**Imitation Learning Pipeline:**
1. Record expert demonstrations on real robot using this package
2. Train BC policies in IsaacLab using the generated datasets
3. Deploy trained policies back to the real robot via the franka_rl_bridge package

Object positions recorded in manual mode can be used to configure simulation environments with matching layouts for better sim-to-real transfer.

## Quick Start Example

```bash
# 1. Record expert demonstrations for IsaacLab
ros2 launch cartesian_impedance_control cartesian_impedance_controller.launch.py &
ros2 param set /cartesian_impedance_controller free_movement_mode true
ros2 run franka_trajectory_recorder trajectory_recorder --ros-args -p action_mode:=pose -p object_mode:=manual

# 2. Play back and record dynamics
ros2 param set /cartesian_impedance_controller imitation_learning_mode  true
ros2 run franka_trajectory_recorder multi_trajectory_playback_with_recording \
    --ros-args -p file_path:=~/franka_ros2_ws/src/franka_trajectory_recorder/trajectories/dataset.hdf5

# 3. Replace with real dynamics for better imitation learning
python3 franka_trajectory_recorder/replace_actions_with_dynamics.py \
    --hdf5 trajectories/dataset.hdf5 \
    --csv ~/multi_trajectory_comparison/session_*/all_demos_recording.csv \
    --output trajectories/dataset_real_dynamics.hdf5 \
    --mode pose

# 4. Analyze performance
python3 franka_trajectory_recorder/trajectory_comparison.py \
    --csv ~/multi_trajectory_comparison/session_*/all_demos_comparison.csv --show
```

## Requirements

- ROS 2 (Humble or later)
- Franka Emika Panda robot with ROS 2 drivers
- Python packages: `numpy`, `h5py`, `plotly`, `pandas`, `scipy`
- NVIDIA IsaacLab (for training imitation learning policies)

## Installation

```bash
cd ~/franka_ros2_ws/src
git clone <repository_url> franka_trajectory_recorder
cd ~/franka_ros2_ws
colcon build --packages-select franka_trajectory_recorder
source install/setup.bash
```

This workflow enables collection of expert demonstrations, analysis of robot tracking performance, and creation of realistic datasets that capture actual robot dynamics for improved imitation learning in NVIDIA IsaacLab environments.