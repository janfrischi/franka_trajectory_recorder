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

Enhanced trajectory recorder with configurable action modes and object positioning for different Isaac Sim environments.

**Features**:
- **Dual Action Modes**:
  - `joint`: Records absolute joint positions (for Isaac-Stack-Cube-Franka-v0)
  - `pose`: Records absolute end-effector poses (for Isaac-Stack-Cube-Franka-IK-Abs-v0)
- **Object Position Modes**:
  - `fixed`: Uses default hardcoded object positions (default)
  - `manual`: Allows interactive custom object positioning per recording session
- Fixed 20Hz sampling rate for consistent data collection
- Advanced HDF5 structure compatible with imitation learning frameworks
- Initial state capture and trajectory metadata
- Support for recording trajectories with varying object positions for better policy generalization

**Topics**:
- **Subscribes**: `/joint_states`, `/fr3_gripper/joint_states`, `/franka_robot_state_broadcaster/robot_state`
- **Actions**: `/fr3_gripper/homing`, `/fr3_gripper/move`, `/fr3_gripper/grasp`

**Parameters**:
- `action_mode`: `'joint'` or `'pose'` (default: `'joint'`)
- `object_mode`: `'fixed'` or `'manual'` (default: `'fixed'`)

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

#### 4. `trajectory_playback_with_recording` (Playback with Comparison)
**File**: `trajectory_playback_with_recording.py`

Advanced playback node that records actual robot motion during trajectory playback for comparison analysis.

**Features**:
- Records actual vs commanded trajectories
- Real-time error calculation (position and orientation)
- Generates comparison CSV files
- Statistical analysis of trajectory following performance

**Parameters**:
- `file_path`: Path to trajectory file
- `playback_rate`: Playback speed multiplier
- `control_mode`: `'joint'` or `'pose'`
- `record_data`: Enable/disable data recording
- `output_dir`: Directory for output files

#### 5. `joint_position_sender` (Manual Control)
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

#### 6. `trajectory_comparison` (Visualization Tool)
**File**: `trajectory_comparison.py`

Creates comprehensive visualizations comparing reference vs actual trajectories.

**Features**:
- Interactive Plotly visualizations
- 3D trajectory plots
- Error analysis charts
- Statistical summaries

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

# Record with fixed object positions (default)
ros2 run franka_trajectory_recorder trajectory_recorder_new --ros-args -p object_mode:=fixed

# Record with manual object positioning for each session
ros2 run franka_trajectory_recorder trajectory_recorder_new --ros-args -p object_mode:=manual

# Combined: pose mode with manual object positioning
ros2 run franka_trajectory_recorder trajectory_recorder_new --ros-args -p action_mode:=pose -p object_mode:=manual
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

#### Playback with Recording
```bash
# Playback and record comparison data
ros2 run franka_trajectory_recorder trajectory_playback_with_recording --ros-args -p file_path:=~/trajectory.csv

# Custom output directory
ros2 run franka_trajectory_recorder trajectory_playback_with_recording --ros-args -p file_path:=~/trajectory.csv -p output_dir:=~/comparison_results
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

### Visualization

#### Trajectory Comparison Analysis
```bash
# Analyze trajectory comparison data
python3 trajectory_comparison.py --csv ~/trajectory_comparison_data/trajectory_comparison_20241201_120000.csv

# Save plots and show interactive visualization
python3 trajectory_comparison.py --csv comparison_data.csv --output ~/plots --show

# Generate all visualizations
python3 trajectory_comparison.py --csv comparison_data.csv --output ~/analysis --show --stats
```

### Complete Workflow Examples

#### Recording a Joint-based Trajectory (Fixed Objects)
```bash
# 1. Start the cartesian impedance controller
ros2 launch cartesian_impedance_control cartesian_impedance_controller.launch.py

# 2. Enable free movement mode for teleoperation
ros2 param set /cartesian_impedance_controller free_movement_mode true

# 3. Start recording in joint mode with fixed object positions
ros2 run franka_trajectory_recorder trajectory_recorder_new --ros-args -p action_mode:=joint -p object_mode:=fixed

# 4. Use keyboard controls:
#    - 'r' to start/pause recording
#    - 'f' to finish and save
#    - 'b' to toggle gripper
```

#### Recording with Custom Object Positions (Manual Mode)
```bash
# 1. Start the cartesian impedance controller
ros2 launch cartesian_impedance_control cartesian_impedance_controller.launch.py

# 2. Enable free movement mode
ros2 param set /cartesian_impedance_controller free_movement_mode true

# 3. Start recording with manual object positioning
ros2 run franka_trajectory_recorder trajectory_recorder_new --ros-args -p action_mode:=joint -p object_mode:=manual

# 4. Use keyboard controls:
#    - 'o' to set custom object positions (before or during recording)
#    - 'r' to start/pause recording
#    - 'f' to finish and save
#    - 'b' to toggle gripper
```

#### Recording a Pose-based Trajectory with Custom Objects
```bash
# 1. Start the cartesian impedance controller
ros2 launch cartesian_impedance_control cartesian_impedance_controller.launch.py

# 2. Enable free movement mode
ros2 param set /cartesian_impedance_controller free_movement_mode true

# 3. Start recording in pose mode with manual object positioning
ros2 run franka_trajectory_recorder trajectory_recorder_new --ros-args -p action_mode:=pose -p object_mode:=manual

# 4. Set custom object positions and record as above
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

#### Complete Analysis Workflow
```bash
# Complete workflow example:
# 1. Record a demonstration trajectory
ros2 run franka_trajectory_recorder trajectory_recorder_new --ros-args -p action_mode:=pose

# 2. Play back trajectory with recording
ros2 run franka_trajectory_recorder trajectory_playback_with_recording \
    --ros-args -p file_path:="~/trajectory.csv" -p record_data:=true

# 3. Analyze the tracking performance
python3 trajectory_comparison.py --csv ~/trajectory_comparison_data/trajectory_comparison_*.csv --show --stats
```

## Interactive Controls

### Recording Controls (During Recording Session)
When running either trajectory recorder, use these keyboard commands:
- **`r`** - Start/Pause recording
- **`f`** - Finish recording and save trajectory
- **`b`** - Toggle gripper state (Open/Close)

### Manual Object Positioning Controls (Only in `object_mode:=manual`)
When using the advanced recorder with manual object mode:
- **`o`** - Set custom object positions (available only in manual mode)

### Object Position Setting Workflow
When you press `o` in manual mode, you'll enter an interactive setup:

1. **Per-object Configuration**: For each cube (Blue, Red, Green), you can:
   - Skip positioning: Type `n`, `no`, or `skip`
   - Keep current position: Press `Enter`
   - Set new position: Type `y` or `yes`, then enter X, Y, Z coordinates

2. **Input Format**:
   ```
   Blue Cube (cube_1):
   Current position: [0.400, 0.200, 0.050]
   Change position? (y/n/skip): y
   Enter new coordinates:
     X (default: 0.400): 0.35
     Y (default: 0.200): 0.15
     Z (default: 0.050): 0.05
   → New position: [0.350, 0.150, 0.050]
   ```

3. **Cancellation**: Press `Ctrl+C` at any time to cancel and keep current positions

4. **Default Object Positions**:
   - **Blue Cube (cube_1)**: [0.400, 0.200, 0.050]
   - **Red Cube (cube_2)**: [0.600, 0.300, 0.050]
   - **Green Cube (cube_3)**: [0.400, -0.200, 0.050]

## Object Position Modes

### Fixed Mode (`object_mode:=fixed`)
- Uses predefined hardcoded object positions
- Consistent object placement across all recordings
- Suitable for basic data collection and initial testing
- No interactive positioning required

### Manual Mode (`object_mode:=manual`)
- Allows custom object positioning for each recording session
- Interactive position setting with user-friendly prompts
- Supports partial updates (change only specific objects)
- Enhanced policy generalization through varied object configurations
- Position validation and error handling

### Use Cases for Manual Mode
- **Policy Generalization**: Record trajectories with objects at different positions to train more robust policies
- **Environment Variations**: Test robot behavior with varying object layouts
- **Data Augmentation**: Increase dataset diversity without changing the physical setup
- **Adaptive Learning**: Train policies that can handle dynamic object placement

## File Formats and Output

### Supported File Formats
- **HDF5 (`.h5`)**: Efficient binary format for large datasets, structured for ML frameworks
- **CSV (`.csv`)**: Human-readable format for inspection and debugging

### Output Locations
- **HDF5 Dataset**: `~/franka_ros2_ws/src/franka_trajectory_recorder/trajectories/dataset.hdf5`
- **CSV Files**: `~/franka_ros2_ws/src/franka_trajectory_recorder/trajectories/trajectory.csv`
- **Comparison Data**: `~/trajectory_comparison_data/trajectory_comparison_YYYYMMDD_HHMMSS.csv`

### HDF5 Structure (Advanced Recorder)
The advanced recorder creates datasets compatible with Isaac Sim environments:

```
/data
├── demo_0, demo_1, ... (trajectory demonstrations)
│   ├── num_samples (number of data points)
│   ├── success (boolean success flag)
│   ├── actions (robot actions - format depends on mode)
│   ├── initial_state (robot and environment initial state)
│   │   ├── articulation
│   │   │   ├── robot (joint positions, velocities, poses)
│   │   │   └── rigid_object
│   │   │       ├── cube_1 (blue cube position and orientation)
│   │   │       ├── cube_2 (red cube position and orientation)
│   │   │       └── cube_3 (green cube position and orientation)
│   ├── obs (observations during trajectory)
│   └── states (full state information)
```

### Trajectory Comparison Output
The trajectory comparison files contain:
- Reference vs actual end-effector positions
- Reference vs actual orientations (quaternions)
- Position and orientation errors
- Statistical metrics (RMSE, max error, etc.)

### Object Position Storage
Each cube's position and orientation are stored as:
- **root_pose**: `[x, y, z, qx, qy, qz, qw]` (position + quaternion)
- **root_velocity**: `[vx, vy, vz, wx, wy, wz]` (linear + angular velocity)

The object positions set during recording are automatically saved and logged for each demonstration.

### Action Formats
- **Joint Mode**: `[joint1, joint2, ..., joint7, gripper_command]` (8 elements)
- **Pose Mode**: `[x, y, z, qw, qx, qy, qz, gripper_command]` (8 elements)

## Visualization and Analysis

### Trajectory Comparison Tool

The `trajectory_comparison.py` script provides comprehensive analysis of trajectory following performance:

#### Features:
- **3D Trajectory Visualization**: Overlaid reference and actual paths
- **Position Tracking Analysis**: X, Y, Z positions over time with error plots
- **Orientation Analysis**: Quaternion tracking and angular error analysis
- **Error Statistics**: Mean, RMSE, maximum, and standard deviation
- **Velocity Analysis**: Speed comparison and tracking errors
- **Interactive Dashboard**: Combined view of all metrics

#### Usage Examples:
```bash
# Basic analysis with interactive plots
python3 trajectory_comparison.py --csv trajectory_comparison_20250703_171824.csv --show

# Save plots and show statistics only
python3 trajectory_comparison.py --csv trajectory_comparison_20250703_171824.csv --output ./plots --stats

# Full analysis with plots and statistics
python3 trajectory_comparison.py --csv trajectory_comparison_20250703_171824.csv --show --output ./analysis_results
```

#### Output Analysis:
The tool provides detailed performance metrics:
```
================================================================================
TRAJECTORY FOLLOWING STATISTICS
================================================================================
File: trajectory_comparison_20250703_171824.csv
Duration: 2.01 seconds
Data Points: 41
Frequency: 20.4 Hz

📍 POSITION ERROR STATISTICS (mm):
  Mean:     69.770
  RMSE:     74.692
  Max:      121.581
  Std:      29.147
  95th %:   117.280

🧭 ORIENTATION ERROR STATISTICS (degrees):
  Mean:     6.246
  RMSE:     6.593
  Max:      10.921
  Std:      2.578
  95th %:   10.565

🎯 PERFORMANCE ASSESSMENT:
  Position tracking: Needs improvement (RMSE > 50mm)
  Orientation tracking: Good (RMSE < 10°)
```

## Integration with Isaac Sim

The package is designed to work with Isaac Sim environments:
- **Isaac-Stack-Cube-Franka-v0**: Use `action_mode:=joint`
- **Isaac-Stack-Cube-Franka-IK-Abs-v0**: Use `action_mode:=pose`

Object positions recorded in manual mode can be used to:
- Configure simulation environments with matching object layouts
- Train policies on diverse object configurations
- Test sim-to-real transfer with varying object positions

## Controller Integration

### Cartesian Impedance Controller
The package integrates with the `cartesian_impedance_control` package for different operation modes:

```bash
# Enable recording mode (free movement)
ros2 param set /cartesian_impedance_controller free_movement_mode true

# Enable playback mode
ros2 param set /cartesian_impedance_controller trajectory_playback_mode true

# Enable imitation learning mode
ros2 param set /cartesian_impedance_controller imitation_learning_mode true
```

### Topic Mapping
- **Joint Control**: `/trajectory_playback/joint_positions`
- **Pose Control**: `/cartesian_position_controller/commands`
- **Policy Commands**: `/policy_outputs`

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
- `geometry_msgs`

### Python Dependencies
- `numpy`
- `h5py`
- `plotly` (for visualization)
- `pandas` (for data analysis)
- `scipy` (for spatial transformations)
- `termios`, `tty`, `select` (for keyboard input)

### Hardware Requirements
- Franka Emika Panda robot
- Franka Hand gripper
- Proper network connection to robot controller

## Installation

```bash
# Clone the repository into your ROS 2 workspace
cd ~/franka_ros2_ws/src
git clone <repository_url> franka_trajectory_recorder

# Build the package
cd ~/franka_ros2_ws
colcon build --packages-select franka_trajectory_recorder

# Source the workspace
source install/setup.bash
```

## Troubleshooting

### Common Issues

1. **Action servers not available**: Ensure the Franka gripper driver is running
2. **Permission denied for keyboard input**: Run the recording nodes with appropriate terminal permissions
3. **File path errors**: Check that the trajectory output directories exist
4. **Joint state not received**: Verify the robot controller and joint state publisher are active
5. **Object positioning interface not responding**: Ensure terminal is in focus and supports interactive input
6. **Visualization errors**: Install required Python packages (`pip install plotly pandas scipy`)

### Object Position Issues
- **Invalid coordinates**: The system validates input and prompts for re-entry on invalid values
- **Coordinate system**: Positions are in robot base frame (meters)
- **Reasonable ranges**: Typical workspace X: [0.2, 0.8], Y: [-0.5, 0.5], Z: [0.0, 0.5]

### Performance Analysis Issues
- **Missing comparison data**: Ensure `record_data:=true` when using playback with recording
- **Empty trajectory files**: Check that recording was properly started and finished
- **Plot display issues**: Verify browser compatibility for interactive Plotly visualizations

### Verification Commands
```bash
# Check active topics
ros2 topic list

# Monitor joint states
ros2 topic echo /joint_states

# Check gripper action servers
ros2 action list

# Verify recorded object positions in HDF5
python3 -c "import h5py; f=h5py.File('dataset.hdf5','r'); print('Cube positions in demo_0:'); [print(f'  {k}: {f[f\"data/demo_0/initial_state/articulation/rigid_object/{k}/root_pose\"][:]}') for k in ['cube_1','cube_2','cube_3']]"

# Test trajectory comparison analysis
python3 trajectory_comparison.py --csv trajectory_comparison_*.csv --stats
```

## Data Analysis Workflow

### Complete Analysis Pipeline
1. **Record Trajectory**: Use `trajectory_recorder_new` to capture demonstrations
2. **Playback with Recording**: Use `trajectory_playback_with_recording` to assess tracking
3. **Analyze Performance**: Use `trajectory_comparison.py` to visualize and quantify errors
4. **Controller Tuning**: Adjust impedance parameters based on analysis results
5. **Validation**: Re-record and analyze to verify improvements

### Performance Metrics
- **Position Accuracy**: RMSE typically < 50mm for good performance
- **Orientation Accuracy**: RMSE typically < 10° for good performance
- **Frequency Analysis**: Consistent 20Hz data collection and playback
- **Gripper Synchronization**: Proper timing of gripper state changes

## License

Apache License 2.0

## Maintainer

For questions and support, please contact the package maintainer.