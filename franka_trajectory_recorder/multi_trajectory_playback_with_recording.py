#!/usr/bin/env python3
"""
Multi-Trajectory Playback with Recording
Plays back multiple trajectories from HDF5 dataset and records dynamics comparison data.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from franka_msgs.action import Move, Grasp
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import h5py
import csv
import os
import numpy as np
import datetime

class MultiTrajectoryPlayback(Node):
    def __init__(self):
        super().__init__('multi_trajectory_playback')

        # Declare parameters
        self.declare_parameter('file_path', '~/dataset_25_06.hdf5')
        self.declare_parameter('playback_rate', 1.0)
        self.declare_parameter('control_mode', 'pose')  # 'joint' or 'pose'
        self.declare_parameter('record_data', True)
        self.declare_parameter('output_dir', '~/multi_trajectory_comparison')
        self.declare_parameter('demo_filter', '')  # Filter demos, e.g., "demo_0,demo_1" or leave empty for all
        self.declare_parameter('auto_next_demo', True)  # Automatically play next demo
        self.declare_parameter('pause_between_demos', 3.0)  # Pause between demos in seconds
        
        # Get parameters
        self.file_path = os.path.expanduser(self.get_parameter('file_path').value)
        self.playback_rate = self.get_parameter('playback_rate').value
        self.control_mode = self.get_parameter('control_mode').value
        self.record_data = self.get_parameter('record_data').value
        self.output_dir = os.path.expanduser(self.get_parameter('output_dir').value)
        self.demo_filter = self.get_parameter('demo_filter').value
        self.auto_next_demo = self.get_parameter('auto_next_demo').value
        self.pause_between_demos = self.get_parameter('pause_between_demos').value

        # Create output directory
        if self.record_data:
            os.makedirs(self.output_dir, exist_ok=True)
            self.session_timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

        # Load dataset and demo list
        self.dataset_file = None
        self.demo_list = []
        self.current_demo_index = 0
        self.current_trajectory = None
        self.current_demo_name = None
        
        # Playback state
        self.current_step_index = 0
        self.timer = None
        self.playback_start_time = None
        self.demo_start_time = None
        self.previous_gripper_state = None
        self.initial_state_waited = False
        
        # Robot state storage (same as original)
        self.current_joint_positions = None
        self.current_eef_pose = None
        self.current_gripper_positions = None
        self.current_gripper_width = None
        
        # Recording data for current demo
        self.recorded_data = []
        self.comparison_data = []

        # Add accumulators for all-demos data
        self.all_recorded_data = []
        self.all_comparison_data = []

        # Setup QoS and subscribers/publishers (same as original)
        self.setup_communications()
        
        # Load dataset and start playback
        if self.load_dataset():
            self.start_multi_trajectory_playback()
        else:
            self.get_logger().error("Failed to load dataset. Shutting down.")
            return

    def load_dataset(self) -> bool:
        """Load HDF5 dataset and extract demo list"""
        try:
            self.dataset_file = h5py.File(self.file_path, 'r')
            
            # Get all demo keys
            all_demos = [key for key in self.dataset_file['data'].keys() if key.startswith('demo_')]
            all_demos.sort()  # Sort numerically
            
            # Apply demo filter if specified
            if self.demo_filter:
                filter_demos = [d.strip() for d in self.demo_filter.split(',')]
                self.demo_list = [d for d in all_demos if d in filter_demos]
            else:
                self.demo_list = all_demos
            
            if not self.demo_list:
                self.get_logger().error("No demos found matching filter criteria")
                return False
                
            self.get_logger().info(f"Loaded dataset with {len(self.demo_list)} demos: {self.demo_list}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error loading dataset: {e}")
            return False

    def load_demo_trajectory(self, demo_name: str) -> bool:
        """Load a specific demo trajectory from the dataset"""
        try:
            demo_group = self.dataset_file['data'][demo_name]
            
            # Check if this is pose mode or joint mode based on data structure
            if 'obs' in demo_group and 'eef_pos' in demo_group['obs']:
                # This is pose mode data - extract from observations
                eef_pos = demo_group['obs']['eef_pos'][:]  # [N, 3]
                eef_quat = demo_group['obs']['eef_quat'][:]  # [N, 4] in [qw, qx, qy, qz] format
                
                # Convert quaternion format from [qw, qx, qy, qz] to [qx, qy, qz, qw] for compatibility
                eef_quat_converted = np.column_stack([
                    eef_quat[:, 1],  # qx
                    eef_quat[:, 2],  # qy  
                    eef_quat[:, 3],  # qz
                    eef_quat[:, 0]   # qw
                ])
                
                # Combine position and quaternion: [x, y, z, qx, qy, qz, qw]
                actions = np.column_stack([eef_pos, eef_quat_converted])
                
                # Extract gripper states if available
                if 'actions' in demo_group:
                    # Actions might contain gripper commands in last column
                    full_actions = demo_group['actions'][:]
                    if full_actions.shape[1] >= 8:  # Has gripper command
                        gripper_commands = full_actions[:, -1]  # Last column
                        gripper_states = ['open' if cmd > 0.5 else 'closed' for cmd in gripper_commands]
                    else:
                        gripper_states = ['open'] * len(actions)  # Default to open
                else:
                    gripper_states = ['open'] * len(actions)  # Default to open
                    
            elif 'actions' in demo_group:
                # Direct actions available
                full_actions = demo_group['actions'][:]
                
                if full_actions.shape[1] >= 8:  # Pose + gripper
                    actions = full_actions[:, :7]  # [x, y, z, qx, qy, qz, qw]
                    gripper_commands = full_actions[:, 7]
                    gripper_states = ['open' if cmd > 0.5 else 'closed' for cmd in gripper_commands]
                elif full_actions.shape[1] == 7:  # Just pose
                    actions = full_actions
                    gripper_states = ['open'] * len(actions)
                else:
                    self.get_logger().error(f"Unexpected action dimension: {full_actions.shape[1]}")
                    return False
            else:
                self.get_logger().error(f"No suitable trajectory data found in {demo_name}")
                return False
            
            # Create timestamps (assume 20Hz if not available)
            num_samples = len(actions)
            timestamps = np.arange(num_samples) * 0.05  # 20Hz = 0.05s intervals
            
            # Store trajectory data
            self.current_trajectory = {
                'timestamps': timestamps,
                'actions': actions,
                'gripper_state': gripper_states
            }
            
            self.current_demo_name = demo_name
            num_samples = demo_group.attrs.get('num_samples', len(actions))
            
            self.get_logger().info(f"Loaded demo '{demo_name}': {num_samples} samples, {timestamps[-1]:.2f}s duration")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error loading demo {demo_name}: {e}")
            return False

    def setup_communications(self):
        """Setup QoS, subscribers, and publishers (same as original)"""
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # Subscribers for robot state (same as original trajectory_playback_with_recording.py)
        if self.record_data:
            self.joint_state_sub = self.create_subscription(
                JointState, '/joint_states', self.joint_state_callback, qos_profile)
            self.eef_pose_sub = self.create_subscription(
                PoseStamped, '/franka_robot_state_broadcaster/current_pose', 
                self.eef_pose_callback, qos_profile)
            self.gripper_state_sub = self.create_subscription(
                JointState, '/fr3_gripper/joint_states', 
                self.gripper_state_callback, qos_profile)

        # Publishers for control commands
        if self.control_mode == 'joint':
            self.joint_positions_publisher = self.create_publisher(
                Float64MultiArray, '/trajectory_playback/jointPositions', 10)
        elif self.control_mode == 'pose':
            self.pose_publisher = self.create_publisher(
                Float64MultiArray, '/cartesian_position_controller/commands', 10)

        # Gripper action clients
        self.move_client = ActionClient(self, Move, '/fr3_gripper/move')
        self.grasp_client = ActionClient(self, Grasp, '/fr3_gripper/grasp')

    def start_multi_trajectory_playback(self):
        """Start playback of all demos"""
        if not self.demo_list:
            self.get_logger().error("No demos to play back")
            return
            
        self.get_logger().info(f"Starting multi-trajectory playback: {len(self.demo_list)} demos")
        self.start_next_demo()

    def finish_current_demo(self):
        """Finish current demo and clean up properly"""
        self.get_logger().info(f"Demo '{self.current_demo_name}' completed.")
        
        # Cancel current timer properly and reset timing state
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        
        # Reset timing variables
        self.playback_start_time = None
        self.demo_start_time = None
        
        # Save demo data
        if self.record_data:
            self.save_demo_data()
            self.print_demo_statistics()
        
        # Move to next demo
        self.current_demo_index += 1
        
        # Schedule next demo or finish
        self.schedule_next_demo()

    def start_next_demo(self):
        """Start playback of the next demo with proper state reset"""
        if self.current_demo_index >= len(self.demo_list):
            self.get_logger().info("All demos completed!")
            self.save_session_summary()
            return
            
        demo_name = self.demo_list[self.current_demo_index]
        self.get_logger().info(f"Starting demo {self.current_demo_index + 1}/{len(self.demo_list)}: {demo_name}")
        
        # Load demo trajectory
        if not self.load_demo_trajectory(demo_name):
            self.get_logger().error(f"Failed to load demo {demo_name}, skipping...")
            self.current_demo_index += 1
            self.start_next_demo()  # Try next demo
            return
        
        # CRITICAL: Reset ALL playback state variables
        self.current_step_index = 0
        self.previous_gripper_state = None
        self.playback_start_time = None
        self.demo_start_time = None
        
        # Clear previous demo data
        self.recorded_data = []
        self.comparison_data = []
        
        # Cancel any existing timer
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        
        # Initialize recording files for this demo
        if self.record_data:
            self.init_demo_recording_files()
        
        # Start playback timer with fresh timing
        self.demo_start_time = self.get_clock().now().nanoseconds / 1e9

        # Wait for robot state only before the first demo
        if self.record_data and not self.initial_state_waited:
            self.wait_for_robot_state()
            self.initial_state_waited = True

        self.start_demo_playback()

    def start_demo_playback(self):
        """Start playback timer for current demo with fresh timing calculation"""
        # Make sure any existing timer is canceled
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        
        # Calculate intervals from trajectory timestamps
        timestamps = self.current_trajectory['timestamps']
        intervals = np.diff(timestamps) / self.playback_rate
        self.intervals = np.append(intervals, intervals[-1])
        
        # Reset timing - CRITICAL for proper playback
        self.playback_start_time = self.get_clock().now().nanoseconds / 1e9
        
        # Start with first interval
        self.timer = self.create_timer(self.intervals[0], self.publish_next_point)
        
        self.get_logger().info(f"Started playback for demo '{self.current_demo_name}' with {len(self.current_trajectory['actions'])} points")
        self.get_logger().info(f"First interval: {self.intervals[0]:.4f}s, Playback rate: {self.playback_rate}")

    def publish_next_point(self):
        """Publish next trajectory point with proper timing management"""
        data_length = len(self.current_trajectory['actions'])
            
        if self.current_step_index >= data_length:
            self.finish_current_demo()
            return

        # Record current state before sending new command
        if self.record_data:
            self.record_current_state()

        # Get reference values
        current_action = self.current_trajectory['actions'][self.current_step_index]
        current_gripper_state = self.current_trajectory['gripper_state'][self.current_step_index]

        # Publish pose command
        self.publish_pose_command()

        # Record comparison data
        if self.record_data:
            self.record_comparison_data(current_action, current_gripper_state)

        # Handle gripper
        self.handle_gripper_state(current_gripper_state)

        # Move to next point
        self.current_step_index += 1
        
        # Update timer for next interval if we have more points
        if self.current_step_index < len(self.intervals):
            # Cancel current timer and create new one with next interval
            if self.timer is not None:
                self.timer.cancel()
                self.timer = None
            
            # Create new timer with the next interval
            next_interval = self.intervals[self.current_step_index]
            self.timer = self.create_timer(next_interval, self.publish_next_point)

    def schedule_next_demo(self):
        """Schedule the next demo with optional pause and proper cleanup"""
        if self.current_demo_index >= len(self.demo_list):
            self.get_logger().info("All demos completed!")
            if self.record_data:
                self.save_session_summary()
            return
            
        if self.auto_next_demo:
            if self.pause_between_demos > 0:
                self.get_logger().info(f"Pausing {self.pause_between_demos}s before next demo...")
                
                # Create a one-shot timer for the pause
                def start_next_after_pause():
                    self.start_next_demo()
                
                # Use create_timer for the pause
                pause_timer = self.create_timer(self.pause_between_demos, start_next_after_pause)
                
                # Make it one-shot by canceling after execution
                original_callback = pause_timer.callback
                def one_shot_callback():
                    pause_timer.cancel()
                    original_callback()
                pause_timer.callback = one_shot_callback
            
            else:
                # No pause, start immediately
                self.start_next_demo()
        else:
            self.get_logger().info(f"Auto-next disabled. Playback stopped after demo {self.current_demo_index}.")

    def load_demo_trajectory(self, demo_name: str) -> bool:
        """Load a specific demo trajectory from the dataset with validation"""
        try:
            demo_group = self.dataset_file['data'][demo_name]
            
            # Extract actions
            if 'actions' not in demo_group:
                self.get_logger().error(f"No 'actions' dataset found in demo {demo_name}")
                return False
            
            actions = demo_group['actions'][:]
            num_samples = len(actions)
            
            self.get_logger().info(f"Loading demo: {demo_name} with {num_samples} samples")
            
            # Generate timestamps (20Hz as per trajectory_recorder_new)
            dt = 0.05  # 20Hz
            timestamps = np.arange(num_samples) * dt
            
            # Extract gripper states from actions
            if actions.shape[1] >= 8:  # Has gripper command
                gripper_commands = actions[:, -1]  # Last column is gripper command
                gripper_states = np.array(['open' if g > 0.0 else 'closed' for g in gripper_commands])
                
                # Remove gripper command from actions for control
                if self.control_mode == 'pose':
                    # For pose mode, take [x, y, z, qw, qx, qy, qz] (first 7 elements)
                    control_actions = actions[:, :7]
                elif self.control_mode == 'joint':
                    # For joint mode, take first 7 elements (joint positions)
                    control_actions = actions[:, :7]
                else:
                    self.get_logger().error(f"Unknown control mode: {self.control_mode}")
                    return False
            else:
                # No gripper command, use all actions
                control_actions = actions
                gripper_states = np.array(['open'] * num_samples)
            
            # Validate action dimensions
            if control_actions.shape[1] < 7:
                self.get_logger().error(f"Insufficient action dimensions: got {control_actions.shape[1]}, need 7")
                return False
            
            # Store trajectory data
            self.current_trajectory = {
                'timestamps': timestamps,
                'actions': control_actions,
                'gripper_state': gripper_states
            }
            
            self.current_demo_name = demo_name
            
            # Log demo metadata if available
            if hasattr(demo_group, 'attrs'):
                for attr_name in demo_group.attrs:
                    attr_value = demo_group.attrs[attr_name]
                    self.get_logger().info(f"Demo attribute {attr_name}: {attr_value}")
            
            self.get_logger().info(f"Successfully loaded demo with {num_samples} samples")
            self.get_logger().info(f"Control mode: {self.control_mode}, Action shape: {control_actions.shape}")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error loading demo '{demo_name}': {e}")
            return False

    def init_demo_recording_files(self):
        """Initialize recording files for current demo"""
        demo_dir = os.path.join(self.output_dir, f"session_{self.session_timestamp}")
        os.makedirs(demo_dir, exist_ok=True)
        
        demo_timestamp = datetime.datetime.now().strftime("%H%M%S")
        demo_safe_name = self.current_demo_name.replace('/', '_').replace('\\', '_')
        
        self.demo_recording_filename = os.path.join(
            demo_dir, f"{demo_safe_name}_recording_{demo_timestamp}.csv"
        )
        self.demo_comparison_filename = os.path.join(
            demo_dir, f"{demo_safe_name}_comparison_{demo_timestamp}.csv"
        )

    def save_demo_data(self):
        """Save recorded data for current demo"""
        if not self.record_data:
            return

        # Save recording data (same format as original)
        if self.recorded_data:
            with open(self.demo_recording_filename, 'w', newline='') as csvfile:
                if self.recorded_data:
                    fieldnames = self.recorded_data[0].keys()
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                    writer.writeheader()
                    writer.writerows(self.recorded_data)
            
            self.get_logger().info(f"Saved {len(self.recorded_data)} recording samples for {self.current_demo_name}")

            # <-- THIS IS CRITICAL
            self.all_recorded_data.extend(self.recorded_data)  

        # Save comparison data
        if self.comparison_data:
            with open(self.demo_comparison_filename, 'w', newline='') as csvfile:
                if self.comparison_data:
                    fieldnames = self.comparison_data[0].keys()
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                    writer.writeheader()
                    writer.writerows(self.comparison_data)
            
            self.get_logger().info(f"Saved {len(self.comparison_data)} comparison samples for {self.current_demo_name}")

            # Print the demo statistics
            self.print_demo_statistics()

            # Append to all-demos accumulator
            self.all_comparison_data.extend(self.comparison_data)

    def save_session_summary(self):
        """Save summary of entire session and all-demos CSVs"""
        if not self.record_data:
            return

        summary_file = os.path.join(self.output_dir, f"session_{self.session_timestamp}", "session_summary.txt")
        
        with open(summary_file, 'w') as f:
            f.write(f"Multi-Trajectory Playback Session Summary\n")
            f.write(f"{'='*50}\n")
            f.write(f"Session timestamp: {self.session_timestamp}\n")
            f.write(f"Dataset file: {self.file_path}\n")
            f.write(f"Control mode: {self.control_mode}\n")
            f.write(f"Playback rate: {self.playback_rate}\n")
            f.write(f"Total demos played: {self.current_demo_index}\n")
            f.write(f"Demo list: {self.demo_list}\n")
            f.write(f"Auto next demo: {self.auto_next_demo}\n")
            f.write(f"Pause between demos: {self.pause_between_demos}s\n")
        
        self.get_logger().info(f"Session summary saved to: {summary_file}")

        # Save all-demos CSVs
        session_dir = os.path.join(self.output_dir, f"session_{self.session_timestamp}")
        os.makedirs(session_dir, exist_ok=True)

        # Save all-recorded-data
        if self.all_recorded_data:
            all_recording_file = os.path.join(session_dir, "all_demos_recording.csv")
            with open(all_recording_file, 'w', newline='') as csvfile:
                fieldnames = list(self.all_recorded_data[0].keys())
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.all_recorded_data)
            self.get_logger().info(f"All-demos recording saved to: {all_recording_file}")

        # Save all-comparison-data
        if self.all_comparison_data:
            all_comparison_file = os.path.join(session_dir, "all_demos_comparison.csv")
            with open(all_comparison_file, 'w', newline='') as csvfile:
                fieldnames = list(self.all_comparison_data[0].keys())
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.all_comparison_data)
            self.get_logger().info(f"All-demos comparison saved to: {all_comparison_file}")

    def print_demo_statistics(self):
        """Print statistics for current demo"""
        if not self.comparison_data:
            return

        pos_errors = [d['pos_error_norm'] for d in self.comparison_data]
        orient_errors = [d['orientation_error_angle'] for d in self.comparison_data]

        self.get_logger().info(f"=== DEMO '{self.current_demo_name}' STATISTICS ===")
        self.get_logger().info(f"Duration: {self.comparison_data[-1]['playback_time']:.2f}s")
        self.get_logger().info(f"Samples: {len(self.comparison_data)}")
        self.get_logger().info(f"Position Error (mm): Mean={np.mean(pos_errors)*1000:.2f}, Max={np.max(pos_errors)*1000:.2f}")
        self.get_logger().info(f"Orientation Error (deg): Mean={np.degrees(np.mean(orient_errors)):.2f}, Max={np.degrees(np.max(orient_errors)):.2f}")

    # Include all the callback and utility methods from the original trajectory_playback_with_recording.py
    # (joint_state_callback, eef_pose_callback, gripper_state_callback, record_current_state, 
    #  record_comparison_data, publish_pose_command, handle_gripper_state, etc.)
    
    def joint_state_callback(self, msg: JointState):
        """Callback for joint state updates (same as original)"""
        franka_joint_names = ['fr3_joint1', 'franka_joint_2', 'franka_joint_3', 'franka_joint_4',
                             'franka_joint_5', 'franka_joint_6', 'franka_joint_7']
        
        joint_positions = []
        joint_velocities = []
        for joint_name in franka_joint_names:
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                joint_positions.append(msg.position[idx])
                if idx < len(msg.velocity):
                    joint_velocities.append(msg.velocity[idx])
        
        if len(joint_positions) == 7:
            self.current_joint_positions = np.array(joint_positions)
            if len(joint_velocities) == 7:
                self.current_joint_velocities = np.array(joint_velocities)

    def eef_pose_callback(self, msg: PoseStamped):
        """Callback for end-effector pose updates (same as original)"""
        self.current_eef_pose = msg

    def gripper_state_callback(self, msg: JointState):
        """Callback for gripper state updates (same as original)"""
        if len(msg.position) >= 2:
            self.current_gripper_positions = np.array(msg.position[:2])
            self.current_gripper_width = sum(msg.position[:2])

    def record_current_state(self):
        """Record current robot state (same as original)"""
        if not self.record_data:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        playback_time = current_time - self.playback_start_time if self.playback_start_time else 0.0

        recording_data = {
            'timestamp': current_time,
            'playback_time': playback_time,
            'step_index': self.current_step_index,
            'demo_name': self.current_demo_name,
        }

        # Add EEF pose
        if self.current_eef_pose is not None:
            recording_data.update({
                'actual_eef_pos_x': self.current_eef_pose.pose.position.x,
                'actual_eef_pos_y': self.current_eef_pose.pose.position.y,
                'actual_eef_pos_z': self.current_eef_pose.pose.position.z,
                'actual_eef_quat_x': self.current_eef_pose.pose.orientation.x,
                'actual_eef_quat_y': self.current_eef_pose.pose.orientation.y,
                'actual_eef_quat_z': self.current_eef_pose.pose.orientation.z,
                'actual_eef_quat_w': self.current_eef_pose.pose.orientation.w,
            })
        else:
            recording_data.update({
                'actual_eef_pos_x': 0.0, 'actual_eef_pos_y': 0.0, 'actual_eef_pos_z': 0.0,
                'actual_eef_quat_x': 0.0, 'actual_eef_quat_y': 0.0, 'actual_eef_quat_z': 0.0, 'actual_eef_quat_w': 1.0,
            })

        # Add gripper state
        if self.current_gripper_width is not None:
            recording_data['actual_gripper_width'] = self.current_gripper_width
            recording_data['actual_gripper_pos_1'] = self.current_gripper_positions[0]
            recording_data['actual_gripper_pos_2'] = self.current_gripper_positions[1]
        else:
            recording_data['actual_gripper_width'] = 0.0
            recording_data['actual_gripper_pos_1'] = 0.0
            recording_data['actual_gripper_pos_2'] = 0.0

        self.recorded_data.append(recording_data)

    def record_comparison_data(self, reference_pose, reference_gripper_state):
        """Record comparison between reference and actual (same logic as original)"""
        if not self.record_data or self.current_eef_pose is None:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        playback_time = current_time - self.playback_start_time if self.playback_start_time else 0.0

        # Reference values
        ref_pos = reference_pose[:3]
        ref_quat = reference_pose[3:7]  # [qx, qy, qz, qw]

        # Actual values
        actual_pos = np.array([
            self.current_eef_pose.pose.position.x,
            self.current_eef_pose.pose.position.y,
            self.current_eef_pose.pose.position.z
        ])
        actual_quat = np.array([
            self.current_eef_pose.pose.orientation.x,
            self.current_eef_pose.pose.orientation.y,
            self.current_eef_pose.pose.orientation.z,
            self.current_eef_pose.pose.orientation.w
        ])

        # Calculate errors
        pos_error = actual_pos - ref_pos
        pos_error_norm = np.linalg.norm(pos_error)
        quat_error = actual_quat - ref_quat
        orientation_error_angle = self.calculate_orientation_error(ref_quat, actual_quat)

        comparison_data = {
            'timestamp': current_time,
            'playback_time': playback_time,
            'step_index': self.current_step_index,
            'demo_name': self.current_demo_name,
            # Reference
            'ref_eef_pos_x': ref_pos[0], 'ref_eef_pos_y': ref_pos[1], 'ref_eef_pos_z': ref_pos[2],
            'ref_eef_quat_x': ref_quat[0], 'ref_eef_quat_y': ref_quat[1], 
            'ref_eef_quat_z': ref_quat[2], 'ref_eef_quat_w': ref_quat[3],
            'ref_gripper_state': reference_gripper_state,
            # Actual
            'actual_eef_pos_x': actual_pos[0], 'actual_eef_pos_y': actual_pos[1], 'actual_eef_pos_z': actual_pos[2],
            'actual_eef_quat_x': actual_quat[0], 'actual_eef_quat_y': actual_quat[1], 
            'actual_eef_quat_z': actual_quat[2], 'actual_eef_quat_w': actual_quat[3],
            'actual_gripper_width': self.current_gripper_width if self.current_gripper_width else 0.0,
            # Errors
            'pos_error_x': pos_error[0], 'pos_error_y': pos_error[1], 'pos_error_z': pos_error[2],
            'pos_error_norm': pos_error_norm,
            'quat_error_x': quat_error[0], 'quat_error_y': quat_error[1], 
            'quat_error_z': quat_error[2], 'quat_error_w': quat_error[3],
            'orientation_error_angle': orientation_error_angle
        }

        self.comparison_data.append(comparison_data)

    def calculate_orientation_error(self, q_ref, q_actual):
        """Calculate orientation error as angle between quaternions (same as original)"""
        q_ref = q_ref / np.linalg.norm(q_ref)
        q_actual = q_actual / np.linalg.norm(q_actual)
        
        dot_product = np.abs(np.dot(q_actual, q_ref))
        dot_product = np.clip(dot_product, 0.0, 1.0)
        error_angle = 2.0 * np.arccos(dot_product)
        return error_angle

    def publish_pose_command(self):
        """Publish pose command for Cartesian control mode (same as original)"""
        action_data = self.current_trajectory['actions'][self.current_step_index]
        
        # Convert from [x, y, z, qx, qy, qz, qw] to [x, y, z, qw, qx, qy, qz]
        pose_msg = Float64MultiArray()
        pose_msg.data = [
            float(action_data[0]),  # x
            float(action_data[1]),  # y  
            float(action_data[2]),  # z
            float(action_data[4]),  # qx (was at index 4 in HDF5)
            float(action_data[5]),  # qy (was at index 5 in HDF5)
            float(action_data[6]),  # qz (was at index 6 in HDF5)
            float(action_data[3])   # qw (was at index 3 in HDF5)
        ]
        
        self.pose_publisher.publish(pose_msg)
        
        if self.current_step_index % 50 == 0:
            self.get_logger().info(
                f"Demo {self.current_demo_name}, Step {self.current_step_index}: "
                f"Pos=[{action_data[0]:.3f}, {action_data[1]:.3f}, {action_data[2]:.3f}]"
                f", Quat=[{action_data[4]:.3f}, {action_data[5]:.3f}, {action_data[6]:.3f}, {action_data[3]:.3f}]"
            )

    def handle_gripper_state(self, current_gripper_state):
        """Handle gripper state transitions (same as original)"""
        if self.previous_gripper_state != current_gripper_state:
            if current_gripper_state == 'open':
                self.send_gripper_goal_open()
            elif current_gripper_state in ['closed', 'close']:
                self.send_gripper_goal_close()
        self.previous_gripper_state = current_gripper_state

    def send_gripper_goal_open(self):
        """Send gripper open command (same as original)"""
        goal_msg = Move.Goal()
        goal_msg.width = 0.08  # Max width
        goal_msg.speed = 0.5
        self.move_client.send_goal_async(goal_msg)

    def send_gripper_goal_close(self):
        """Send gripper close command (same as original)"""
        goal_msg = Grasp.Goal()
        goal_msg.width = 0.0
        goal_msg.speed = 0.5
        goal_msg.force = 50.0
        goal_msg.epsilon.inner = 0.05
        goal_msg.epsilon.outer = 0.07
        self.grasp_client.send_goal_async(goal_msg)

    def cleanup(self):
        """Cleanup resources"""
        if self.dataset_file:
            self.dataset_file.close()

    def wait_for_robot_state(self, timeout=5.0):
        """Wait until all required robot state variables are initialized."""
        import time
        start_time = time.time()
        while (
            (self.current_joint_positions is None or
             self.current_eef_pose is None or
             self.current_gripper_positions is None or
             self.current_gripper_width is None)
            and (time.time() - start_time) < timeout
            and rclpy.ok()
        ):
            self.get_logger().info("Waiting for robot state messages...")
            time.sleep(0.1)
        if (self.current_joint_positions is None or
            self.current_eef_pose is None or
            self.current_gripper_positions is None or
            self.current_gripper_width is None):
            self.get_logger().warn("Timeout waiting for robot state. Some values may be None.")


def main(args=None):
    rclpy.init(args=args)
    node = MultiTrajectoryPlayback()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Multi-Trajectory Playback.")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()