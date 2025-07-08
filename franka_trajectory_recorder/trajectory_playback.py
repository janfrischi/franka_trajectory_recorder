import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from franka_msgs.action import Move, Grasp
import h5py
import csv
import os
import numpy as np

class TrajectoryPlayback(Node):
    def __init__(self):
        super().__init__('trajectory_playback')

        # Declare and get parameters
        self.declare_parameter('file_path', '~/trajectory.hdf5')
        self.declare_parameter('playback_rate', 1.0)
        self.declare_parameter('control_mode', 'joint')  # 'joint' or 'pose'
    
        # Get parameters
        self.file_path = os.path.expanduser(self.get_parameter('file_path').value)
        self.playback_rate = self.get_parameter('playback_rate').value
        self.control_mode = self.get_parameter('control_mode').value

        # Validate control mode
        if self.control_mode not in ['joint', 'pose']:
            self.get_logger().error(f"Invalid control_mode '{self.control_mode}'. Must be 'joint' or 'pose'.")
            return

        # Gripper parameters
        self.gripper_speed = 0.5  # Default speed (m/s)
        self.gripper_force = 50.0  # Default grasp force (N)
        self.gripper_max_width = 0.08  # Maximum gripper width (m)
        self.gripper_epsilon_inner = 0.5 # Inner tolerance for grasping
        self.gripper_epsilon_outer = 0.5  # Outer tolerance for grasping

        # Publishers based on control mode
        if self.control_mode == 'joint':
            self.joint_positions_publisher = self.create_publisher(
                Float64MultiArray,
                '/trajectory_playback/joint_positions',
                10
            )
            self.get_logger().info("Initialized joint control mode")
        elif self.control_mode == 'pose':
            self.pose_publisher = self.create_publisher(
                Float64MultiArray,
                '/cartesian_position_controller/commands', # Imitation Learning Topic
                10
            )
            self.get_logger().info("Initialized pose control mode")

        # Gripper action clients
        self.move_client = ActionClient(self, Move, '/fr3_gripper/move')
        self.grasp_client = ActionClient(self, Grasp, '/fr3_gripper/grasp')

        # Wait for servers
        self.wait_for_action_server(self.move_client, 'Move')
        self.wait_for_action_server(self.grasp_client, 'Grasp')

        # Load trajectory
        self.trajectory = self.load_trajectory(self.file_path)
        if self.trajectory is None:
            self.get_logger().error(f"Failed to load trajectory from {self.file_path}")
            return

        # Validate trajectory data for control mode
        if not self.validate_trajectory_data():
            return

        # Prepare playback
        self.current_index = 0
        self.previous_gripper_state = None
        self.timer = None
        # Start playback
        self.start_playback()

    def validate_trajectory_data(self):
        """Validate that trajectory contains required data for the selected control mode."""
        if self.control_mode == 'joint':
            if 'joint_positions' not in self.trajectory:
                self.get_logger().error("Joint control mode requires 'joint_positions' data in trajectory file")
                return False
        elif self.control_mode == 'pose':
            if 'actions' not in self.trajectory:
                self.get_logger().error("Pose control mode requires 'actions' data in trajectory file")
                return False
            # Check if actions have correct dimensions (7: x,y,z,qx,qy,qz,qw)
            if self.trajectory['actions'].shape[1] != 7:
                self.get_logger().error(f"Action data should have 7 elements (x,y,z,qx,qy,qz,qw), got {self.trajectory['actions'].shape[1]}")
                return False
        return True

    def load_trajectory(self, file_path):
        if file_path.endswith('.hdf5'):
            return self.load_hdf5(file_path)
        elif file_path.endswith('.csv'):
            return self.load_csv(file_path)
        else:
            self.get_logger().error("Unsupported file format. Use .h5 or .csv.")
            return None

    # Load trajectory data from HDF5 file
    def load_hdf5(self, file_path):
        try:
            trajectory_data = {}
            with h5py.File(file_path, 'r') as hdf5_file:
                trajectory_data['timestamps'] = hdf5_file['timestamps'][:]
                trajectory_data['gripper_action'] = hdf5_file['gripper_action'][:].astype(str)

                # Load joint positions if available
                if 'joint_positions' in hdf5_file:
                    trajectory_data['joint_positions'] = hdf5_file['joint_positions'][:, :7]
                
                # Load actions (poses) if available
                if 'actions' in hdf5_file:
                    trajectory_data['actions'] = hdf5_file['actions'][:]
                
                return trajectory_data
        except Exception as e:
            self.get_logger().error(f"Error loading HDF5 file: {e}")
            return None

    # Load trajectory data from CSV file
    def load_csv(self, file_path):
        try:
            timestamps = []
            joint_positions = []
            actions = []
            gripper_states = []
            
            with open(file_path, 'r') as csv_file:
                reader = csv.DictReader(csv_file)
                for row in reader:
                    timestamps.append(float(row['timestamp']))
                    gripper_states.append(row['gripper_goal_state'])  # Use gripper_goal_state instead of gripper_action
                    
                    # Load joint positions if column exists
                    if 'joint_positions' in row:
                        positions = [float(x) for x in row['joint_positions'].split(',')[:7]]
                        joint_positions.append(positions)
                    
                    # Load pose actions by concatenating eef_position and eef_orientation
                    if 'eef_position' in row and 'eef_orientation' in row:
                        eef_pos = [float(x) for x in row['eef_position'].split(',')]  # x, y, z
                        eef_orient = [float(x) for x in row['eef_orientation'].split(',')]  # qx, qy, qz, qw
                        # Concatenate position and orientation: [x, y, z, qx, qy, qz, qw]
                        pose_action = eef_pos + eef_orient
                        actions.append(pose_action)
            
            trajectory_data = {
                'timestamps': np.array(timestamps), 
                'gripper_state': np.array(gripper_states)
            }
            
            if joint_positions:
                trajectory_data['joint_positions'] = np.array(joint_positions)
            if actions:
                trajectory_data['actions'] = np.array(actions)
                
            return trajectory_data
        except Exception as e:
            self.get_logger().error(f"Error loading CSV file: {e}")
            return None
        
    def wait_for_action_server(self, client, name):
        self.get_logger().info(f'Waiting for {name} action server...')
        while not client.wait_for_server(timeout_sec=2.0) and rclpy.ok():
            self.get_logger().info(f'{name} action server not available, waiting again...')
        if rclpy.ok():
            self.get_logger().info(f'{name} action server found.')
        else:
            self.get_logger().error(f'ROS shutdown while waiting for {name} server.')
            raise SystemExit('ROS shutdown')

    def start_playback(self):
        timestamps = self.trajectory['timestamps']
        # Calculate intervals based on timestamps
        intervals = np.diff(timestamps) / self.playback_rate
        self.intervals = np.append(intervals, intervals[-1])
        # Every self.intervals[i] the self.publish_next_point will be called
        self.timer = self.create_timer(self.intervals[0], self.publish_next_point)

    def publish_next_point(self):
        if self.control_mode == 'joint':
            data_length = len(self.trajectory['joint_positions'])
        else:  # pose mode
            data_length = len(self.trajectory['actions'])
            
        if self.current_index >= data_length:
            self.get_logger().info("Trajectory playback completed.")
            self.timer.cancel()
            return

        # Extract gripper state
        current_gripper_state = self.trajectory['gripper_state'][self.current_index]

        # Publish control commands based on mode
        if self.control_mode == 'joint':
            self.publish_joint_command()
        elif self.control_mode == 'pose':
            self.publish_pose_command()

        # Handle gripper state transition
        self.handle_gripper_state(current_gripper_state)

        # Move to the next point
        self.current_index += 1
        # Schedule the next point and cancel the previous timer
        if self.current_index < len(self.intervals):
            self.timer.cancel()
            self.timer = self.create_timer(self.intervals[self.current_index], self.publish_next_point)

    def publish_joint_command(self):
        """Publish joint positions for joint control mode."""
        joint_positions = self.trajectory['joint_positions'][self.current_index]
        msg = Float64MultiArray()
        msg.data = joint_positions.tolist()
        self.joint_positions_publisher.publish(msg)
        self.get_logger().info(f"Published joint positions: {joint_positions}")

    def publish_pose_command(self):
        """Publish pose command for Cartesian control mode."""
        action_data = self.trajectory['actions'][self.current_index]
        
        # Create Float64MultiArray message with pose data [x, y, z, qx, qy, qz, qw]
        pose_msg = Float64MultiArray()
        pose_msg.data = [
            float(action_data[0]),  # x
            float(action_data[1]),  # y
            float(action_data[2]),  # z
            float(action_data[3]),  # qx
            float(action_data[4]),  # qy
            float(action_data[5]),  # qz
            float(action_data[6])   # qw
        ]
        
        self.pose_publisher.publish(pose_msg)
        self.get_logger().info(f"Published pose: pos=[{action_data[0]:.3f}, {action_data[1]:.3f}, {action_data[2]:.3f}], "
                              f"quat=[{action_data[3]:.3f}, {action_data[4]:.3f}, {action_data[5]:.3f}, {action_data[6]:.3f}]")

    def handle_gripper_state(self, current_gripper_state):
        """Handle gripper state transitions."""
        if self.previous_gripper_state is None:
            # Initialize the previous state
            self.previous_gripper_state = current_gripper_state
            return

        # Check if the gripper state has changed
        if self.previous_gripper_state != current_gripper_state:
            if current_gripper_state == 'open':
                self.send_gripper_goal_open()
            elif current_gripper_state == 'closed':
                self.send_gripper_goal_close()

        # Update the previous state
        self.previous_gripper_state = current_gripper_state

    def send_gripper_goal_open(self):
        goal_msg = Move.Goal()
        goal_msg.width = self.gripper_max_width
        goal_msg.speed = self.gripper_speed
        self.get_logger().info("Sending gripper OPEN command...")
        self.move_client.send_goal_async(goal_msg)

    def send_gripper_goal_close(self):
        goal_msg = Grasp.Goal()
        goal_msg.width = 0.0
        goal_msg.speed = self.gripper_speed
        goal_msg.force = self.gripper_force
        goal_msg.epsilon.inner = self.gripper_epsilon_inner
        goal_msg.epsilon.outer = self.gripper_epsilon_outer
        self.get_logger().info("Sending gripper CLOSE command...")
        self.grasp_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlayback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Trajectory Playback.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
