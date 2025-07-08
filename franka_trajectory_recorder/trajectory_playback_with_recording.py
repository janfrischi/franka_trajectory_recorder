import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from franka_msgs.action import Move, Grasp
from franka_msgs.msg import FrankaRobotState
import h5py
import csv
import os
import numpy as np
import datetime
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class TrajectoryPlaybackWithRecording(Node):
    def __init__(self):
        super().__init__('trajectory_playback_with_recording')

        # Declare parameters
        self.declare_parameter('file_path', '~/trajectory.csv')
        self.declare_parameter('playback_rate', 1.0)
        self.declare_parameter('control_mode', 'pose')  # 'joint' or 'pose'
        self.declare_parameter('record_data', True)
        self.declare_parameter('output_dir', '~/trajectory_comparison_data')
    
        # Get parameters
        self.file_path = os.path.expanduser(self.get_parameter('file_path').value)
        self.playback_rate = self.get_parameter('playback_rate').value
        self.control_mode = self.get_parameter('control_mode').value
        self.record_data = self.get_parameter('record_data').value
        self.output_dir = os.path.expanduser(self.get_parameter('output_dir').value)

        # Create output directory
        if self.record_data:
            os.makedirs(self.output_dir, exist_ok=True)
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            self.recording_filename = os.path.join(
                self.output_dir, 
                f"playback_recording_{timestamp}.csv"
            )
            self.comparison_filename = os.path.join(
                self.output_dir,
                f"trajectory_comparison_{timestamp}.csv"
            )

        # QoS Profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # Current robot state storage
        self.current_joint_positions = None
        self.current_eef_pose = None
        self.current_gripper_positions = None
        self.current_gripper_width = None

        # Recording data
        self.recorded_data = []
        self.comparison_data = []

        # Subscribers for robot state
        if self.record_data:
            self.joint_state_sub = self.create_subscription(
                JointState,
                '/joint_states',  # or '/franka/joint_states' depending on your setup
                self.joint_state_callback,
                qos_profile
            )
            
            self.eef_pose_sub = self.create_subscription(
                PoseStamped,
                '/franka_robot_state_broadcaster/current_pose',
                self.eef_pose_callback,
                qos_profile
            )
            
            self.gripper_state_sub = self.create_subscription(
                JointState,
                '/fr3_gripper/joint_states',
                self.gripper_state_callback,
                qos_profile
            )

        # Publishers for control commands
        if self.control_mode == 'joint':
            self.joint_positions_publisher = self.create_publisher(
                Float64MultiArray,
                '/trajectory_playback/joint_positions',
                10
            )
        elif self.control_mode == 'pose':
            self.pose_publisher = self.create_publisher(
                Float64MultiArray,
                '/cartesian_position_controller/commands',
                10
            )

        # Gripper action clients
        self.move_client = ActionClient(self, Move, '/fr3_gripper/move')
        self.grasp_client = ActionClient(self, Grasp, '/fr3_gripper/grasp')

        # Load trajectory
        self.trajectory = self.load_trajectory(self.file_path)
        if self.trajectory is None:
            self.get_logger().error(f"Failed to load trajectory from {self.file_path}")
            return

        # Playback state
        self.current_index = 0
        self.previous_gripper_state = None
        self.timer = None
        self.playback_start_time = None

        # Initialize recording
        if self.record_data:
            self.init_recording_file()
            self.init_comparison_file()

        # Start playback
        self.get_logger().info(f"Starting trajectory playback from {self.file_path}")
        if self.record_data:
            self.get_logger().info(f"Recording data to {self.recording_filename}")
            self.get_logger().info(f"Comparison data to {self.comparison_filename}")
        
        self.start_playback()

    def joint_state_callback(self, msg: JointState):
        """Callback for joint state updates"""
        # Extract Franka joint positions (first 7 joints)
        franka_joint_names = ['fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
                             'fr3_joint5', 'fr3_joint6', 'fr3_joint7']
        
        joint_positions = []
        joint_velocities = []
        for joint_name in franka_joint_names:
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                joint_positions.append(msg.position[idx])
                if len(msg.velocity) > idx:
                    joint_velocities.append(msg.velocity[idx])
                else:
                    joint_velocities.append(0.0)
        
        if len(joint_positions) == 7:
            self.current_joint_positions = np.array(joint_positions)
            self.current_joint_velocities = np.array(joint_velocities)

    def eef_pose_callback(self, msg: PoseStamped):
        """Callback for end-effector pose updates"""
        self.current_eef_pose = msg

    def gripper_state_callback(self, msg: JointState):
        """Callback for gripper state updates"""
        if len(msg.position) >= 2:
            self.current_gripper_positions = np.array(msg.position[:2])
            self.current_gripper_width = sum(msg.position[:2])

    def init_recording_file(self):
        """Initialize the recording CSV file"""
        with open(self.recording_filename, 'w', newline='') as csvfile:
            fieldnames = [
                'timestamp', 'playback_time', 'step_index',
                'actual_joint_pos_1', 'actual_joint_pos_2', 'actual_joint_pos_3', 'actual_joint_pos_4',
                'actual_joint_pos_5', 'actual_joint_pos_6', 'actual_joint_pos_7',
                'actual_eef_pos_x', 'actual_eef_pos_y', 'actual_eef_pos_z',
                'actual_eef_quat_x', 'actual_eef_quat_y', 'actual_eef_quat_z', 'actual_eef_quat_w',
                'actual_gripper_width', 'actual_gripper_pos_1', 'actual_gripper_pos_2'
            ]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

    def init_comparison_file(self):
        """Initialize the comparison CSV file"""
        with open(self.comparison_filename, 'w', newline='') as csvfile:
            fieldnames = [
                'timestamp', 'playback_time', 'step_index',
                # Reference (commanded) values
                'ref_eef_pos_x', 'ref_eef_pos_y', 'ref_eef_pos_z',
                'ref_eef_quat_x', 'ref_eef_quat_y', 'ref_eef_quat_z', 'ref_eef_quat_w',
                'ref_gripper_state',
                # Actual values
                'actual_eef_pos_x', 'actual_eef_pos_y', 'actual_eef_pos_z',
                'actual_eef_quat_x', 'actual_eef_quat_y', 'actual_eef_quat_z', 'actual_eef_quat_w',
                'actual_gripper_width',
                # Errors
                'pos_error_x', 'pos_error_y', 'pos_error_z', 'pos_error_norm',
                'quat_error_x', 'quat_error_y', 'quat_error_z', 'quat_error_w',
                'orientation_error_angle'
            ]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

    def record_current_state(self):
        """Record current robot state"""
        if not self.record_data:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        playback_time = current_time - self.playback_start_time if self.playback_start_time else 0.0

        # Record raw robot state
        recording_data = {
            'timestamp': current_time,
            'playback_time': playback_time,
            'step_index': self.current_index,
        }

        # Add joint positions
        if self.current_joint_positions is not None:
            for i, pos in enumerate(self.current_joint_positions):
                recording_data[f'actual_joint_pos_{i+1}'] = pos
        else:
            for i in range(7):
                recording_data[f'actual_joint_pos_{i+1}'] = 0.0

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
        """Record comparison between reference and actual"""
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
        
        # Calculate orientation error as angle
        orientation_error_angle = self.calculate_orientation_error(ref_quat, actual_quat)

        comparison_data = {
            'timestamp': current_time,
            'playback_time': playback_time,
            'step_index': self.current_index,
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
        """Calculate orientation error as angle between quaternions"""
        # Normalize quaternions
        q_ref = q_ref / np.linalg.norm(q_ref)
        q_actual = q_actual / np.linalg.norm(q_actual)
        
        # Calculate relative quaternion
        # q_error = q_actual * q_ref^(-1)
        q_ref_inv = np.array([-q_ref[0], -q_ref[1], -q_ref[2], q_ref[3]])  # conjugate
        
        # Quaternion multiplication: q_error = q_actual * q_ref_inv
        dot_product = np.abs(np.dot(q_actual, q_ref))
        dot_product = np.clip(dot_product, 0.0, 1.0)  # Numerical stability
        
        # Error angle
        error_angle = 2.0 * np.arccos(dot_product)
        return error_angle

    def save_recorded_data(self):
        """Save all recorded data to files"""
        if not self.record_data:
            return

        # Save recording data
        if self.recorded_data:
            with open(self.recording_filename, 'w', newline='') as csvfile:
                fieldnames = self.recorded_data[0].keys()
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.recorded_data)
            self.get_logger().info(f"Saved {len(self.recorded_data)} recording samples to {self.recording_filename}")

        # Save comparison data
        if self.comparison_data:
            with open(self.comparison_filename, 'w', newline='') as csvfile:
                fieldnames = self.comparison_data[0].keys()
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.comparison_data)
            self.get_logger().info(f"Saved {len(self.comparison_data)} comparison samples to {self.comparison_filename}")

            # Print summary statistics
            self.print_trajectory_statistics()

    def print_trajectory_statistics(self):
        """Print trajectory following statistics"""
        if not self.comparison_data:
            return

        pos_errors = [d['pos_error_norm'] for d in self.comparison_data]
        orient_errors = [d['orientation_error_angle'] for d in self.comparison_data]

        self.get_logger().info("=== TRAJECTORY FOLLOWING STATISTICS ===")
        self.get_logger().info(f"Position Error (m):")
        self.get_logger().info(f"  Mean: {np.mean(pos_errors):.4f}")
        self.get_logger().info(f"  Max:  {np.max(pos_errors):.4f}")
        self.get_logger().info(f"  Std:  {np.std(pos_errors):.4f}")
        self.get_logger().info(f"Orientation Error (rad):")
        self.get_logger().info(f"  Mean: {np.mean(orient_errors):.4f}")
        self.get_logger().info(f"  Max:  {np.max(orient_errors):.4f}")
        self.get_logger().info(f"  Std:  {np.std(orient_errors):.4f}")

    # [Rest of the original methods remain the same...]
    def load_trajectory(self, file_path):
        if file_path.endswith('.csv'):
            return self.load_csv(file_path)
        else:
            self.get_logger().error("Unsupported file format. Use .csv for this version.")
            return None

    def load_csv(self, file_path):
        try:
            timestamps = []
            actions = []
            gripper_states = []
            
            with open(file_path, 'r') as csv_file:
                reader = csv.DictReader(csv_file)
                for row in reader:
                    timestamps.append(float(row['timestamp']))
                    gripper_states.append(row['gripper_goal_state'])
                    
                    # Load pose actions
                    if 'eef_position' in row and 'eef_orientation' in row:
                        eef_pos = [float(x) for x in row['eef_position'].split(',')]
                        eef_orient = [float(x) for x in row['eef_orientation'].split(',')]
                        pose_action = eef_pos + eef_orient
                        actions.append(pose_action)
            
            return {
                'timestamps': np.array(timestamps), 
                'gripper_state': np.array(gripper_states),
                'actions': np.array(actions)
            }
        except Exception as e:
            self.get_logger().error(f"Error loading CSV file: {e}")
            return None

    def start_playback(self):
        timestamps = self.trajectory['timestamps']
        intervals = np.diff(timestamps) / self.playback_rate
        self.intervals = np.append(intervals, intervals[-1])
        
        self.playback_start_time = self.get_clock().now().nanoseconds / 1e9
        self.timer = self.create_timer(self.intervals[0], self.publish_next_point)

    def publish_next_point(self):
        data_length = len(self.trajectory['actions'])
            
        if self.current_index >= data_length:
            self.get_logger().info("Trajectory playback completed.")
            self.timer.cancel()
            if self.record_data:
                self.save_recorded_data()
            return

        # Record current state before sending new command
        if self.record_data:
            self.record_current_state()

        # Get reference values
        current_action = self.trajectory['actions'][self.current_index]
        current_gripper_state = self.trajectory['gripper_state'][self.current_index]

        # Publish pose command
        self.publish_pose_command()

        # Record comparison data
        if self.record_data:
            self.record_comparison_data(current_action, current_gripper_state)

        # Handle gripper
        self.handle_gripper_state(current_gripper_state)

        # Move to next point
        self.current_index += 1
        if self.current_index < len(self.intervals):
            self.timer.cancel()
            self.timer = self.create_timer(self.intervals[self.current_index], self.publish_next_point)

    def publish_pose_command(self):
        """Publish pose command for Cartesian control mode."""
        action_data = self.trajectory['actions'][self.current_index]
        
        pose_msg = Float64MultiArray()
        pose_msg.data = [float(x) for x in action_data[:7]]  # [x, y, z, qx, qy, qz, qw]
        
        self.pose_publisher.publish(pose_msg)
        
        if self.current_index % 50 == 0:  # Log every 50th point to avoid spam
            self.get_logger().info(f"Step {self.current_index}: Published pose: "
                                  f"pos=[{action_data[0]:.3f}, {action_data[1]:.3f}, {action_data[2]:.3f}]")

    def handle_gripper_state(self, current_gripper_state):
        """Handle gripper state transitions."""
        if self.previous_gripper_state != current_gripper_state:
            if current_gripper_state == 'open':
                self.send_gripper_goal_open()
            elif current_gripper_state == 'closed':
                self.send_gripper_goal_close()
        self.previous_gripper_state = current_gripper_state

    def send_gripper_goal_open(self):
        goal_msg = Move.Goal()
        goal_msg.width = 0.08  # Max width
        goal_msg.speed = 0.5
        self.get_logger().info("Sending gripper OPEN command...")
        self.move_client.send_goal_async(goal_msg)

    def send_gripper_goal_close(self):
        goal_msg = Grasp.Goal()
        goal_msg.width = 0.0
        goal_msg.speed = 0.5
        goal_msg.force = 50.0
        goal_msg.epsilon.inner = 0.05
        goal_msg.epsilon.outer = 0.07
        self.get_logger().info("Sending gripper CLOSE command...")
        self.grasp_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlaybackWithRecording()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Trajectory Playback with Recording.")
        if node.record_data:
            node.save_recorded_data()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()