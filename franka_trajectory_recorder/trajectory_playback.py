import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float64MultiArray
from franka_msgs.action import Move, Grasp
import h5py
import csv
import os
import numpy as np

class TrajectoryPlayback(Node):
    def __init__(self):
        super().__init__('trajectory_playback')

        # Declare parameters for file path and playback rate
        self.declare_parameter('file_path', '~/trajectory.h5')  # Default file path
        self.declare_parameter('playback_rate', 1.0)  # Playback rate multiplier

        # Get parameters
        self.file_path = os.path.expanduser(self.get_parameter('file_path').value)
        self.playback_rate = self.get_parameter('playback_rate').value

        # Publisher for joint positions
        self.joint_positions_publisher = self.create_publisher(
            Float64MultiArray, 
            '/trajectory_playback/joint_positions', 
            10)

        # Gripper control initialization
        self.gripper_goal_state = 'unknown'  # 'open', 'closed', 'unknown'
        self.gripper_max_width = 0.08  # Max width for Franka Hand
        self.gripper_speed = 0.05  # Default speed (m/s)
        self.gripper_force = 30.0  # Default grasp force (N)
        self.gripper_epsilon_inner = 0.05  # Tolerance for successful grasp
        self.gripper_epsilon_outer = 0.05

        # Action clients for gripper
        self.move_client = ActionClient(self, Move, '/fr3_gripper/move')
        self.grasp_client = ActionClient(self, Grasp, '/fr3_gripper/grasp')

        # Wait for gripper action servers
        self.wait_for_action_server(self.move_client, 'Move')
        self.wait_for_action_server(self.grasp_client, 'Grasp')

        # Timer for playback
        self.timer = None

        # Load trajectory
        self.trajectory = self.load_trajectory(self.file_path)
        if self.trajectory is None:
            self.get_logger().error(f"Failed to load trajectory from {self.file_path}")
            return

        # Start playback
        self.current_index = 0
        self.start_playback()

    def load_trajectory(self, file_path):
        """Load trajectory from HDF5 or CSV file."""
        if file_path.endswith('.h5'):
            return self.load_hdf5(file_path)
        elif file_path.endswith('.csv'):
            return self.load_csv(file_path)
        else:
            self.get_logger().error("Unsupported file format. Use .h5 or .csv.")
            return None

    def load_hdf5(self, file_path):
        """Load trajectory from an HDF5 file."""
        try:
            with h5py.File(file_path, 'r') as hdf5_file:
                timestamps = hdf5_file['timestamps'][:]
                joint_positions = hdf5_file['joint_positions'][:, :9]  # Include gripper commands
                return {'timestamps': timestamps, 'joint_positions': joint_positions}
        except Exception as e:
            self.get_logger().error(f"Error loading HDF5 file: {e}")
            return None

    def load_csv(self, file_path):
        """Load trajectory from a CSV file."""
        try:
            timestamps = []
            joint_positions = []
            with open(file_path, 'r') as csv_file:
                reader = csv.DictReader(csv_file)
                for row in reader:
                    timestamps.append(float(row['timestamp']))
                    positions = [float(x) for x in row['joint_positions'].split(',')[:9]]  # Include gripper commands
                    joint_positions.append(positions)
            return {'timestamps': np.array(timestamps), 'joint_positions': np.array(joint_positions)}
        except Exception as e:
            self.get_logger().error(f"Error loading CSV file: {e}")
            return None

    def start_playback(self):
        """Start the trajectory playback."""
        if self.trajectory is None:
            self.get_logger().error("No trajectory loaded. Cannot start playback.")
            return

        # Calculate playback interval based on timestamps and playback rate
        timestamps = self.trajectory['timestamps']
        intervals = np.diff(timestamps) / self.playback_rate
        self.intervals = np.append(intervals, intervals[-1])  # Repeat last interval for safety

        # Start timer
        self.timer = self.create_timer(self.intervals[0], self.publish_next_point)

    def publish_next_point(self):
        """Publish the next point in the trajectory."""
        if self.current_index >= len(self.trajectory['joint_positions']):
            self.get_logger().info("Trajectory playback completed.")
            self.timer.cancel()
            return

        # Publish joint positions
        joint_positions = self.trajectory['joint_positions'][self.current_index][:7]  # First 7 values are joint positions
        gripper_command = self.trajectory['joint_positions'][self.current_index][7:9]  # Last 2 values are gripper commands

        # Publish joint positions
        msg = Float64MultiArray()
        msg.data = joint_positions.tolist()
        self.joint_positions_publisher.publish(msg)

        # Log the published joint positions
        self.get_logger().info(f"Published joint positions: {joint_positions}")

        # Handle gripper command
        self.handle_gripper_command(gripper_command)

        # Schedule the next point
        self.current_index += 1
        if self.current_index < len(self.intervals):
            self.timer.cancel()
            self.timer = self.create_timer(self.intervals[self.current_index], self.publish_next_point)

    def handle_gripper_command(self, gripper_command):
        """Handle gripper commands based on the trajectory."""
        # Extract the desired gripper width from the command
        gripper_width = gripper_command[0]  # Assume the first value represents the desired width
        self.get_logger().info(f"Handling gripper command with width: {gripper_width:.4f}")

        # Open the gripper if the width is above the threshold and it's not already open
        if gripper_width >= 0.04 and self.gripper_goal_state != 'open':
            self.get_logger().info("Gripper command indicates opening the gripper.")
            self.open_gripper()

        # Close the gripper if the width is zero and it's not already closed
        elif gripper_width == 0.0 and self.gripper_goal_state != 'closed':
            self.get_logger().info("Gripper command indicates closing the gripper.")
            self.close_gripper()

        # Log if no action is required
        else:
            self.get_logger().info("Gripper command does not require any action.")

    def open_gripper(self):
        """Open the gripper."""
        self.get_logger().info("Sending open gripper goal...")
        goal_msg = Move.Goal()
        goal_msg.width = self.gripper_max_width
        goal_msg.speed = self.gripper_speed
        self.get_logger().info(f"Open gripper goal: width={goal_msg.width}, speed={goal_msg.speed}")
        self.move_client.send_goal_async(goal_msg)
        self.gripper_goal_state = 'open'
        self.get_logger().info("Gripper opened.")

    def close_gripper(self):
        """Close the gripper."""
        self.get_logger().info("Sending close gripper goal...")
        goal_msg = Grasp.Goal()
        goal_msg.width = 0.0
        goal_msg.speed = self.gripper_speed
        goal_msg.force = self.gripper_force
        goal_msg.epsilon.inner = self.gripper_epsilon_inner
        goal_msg.epsilon.outer = self.gripper_epsilon_outer
        self.get_logger().info(
            f"Close gripper goal: width={goal_msg.width}, speed={goal_msg.speed}, "
            f"force={goal_msg.force}, epsilon_inner={goal_msg.epsilon.inner}, epsilon_outer={goal_msg.epsilon.outer}"
        )
        self.grasp_client.send_goal_async(goal_msg)
        self.gripper_goal_state = 'closed'
        self.get_logger().info("Gripper closed.")

    def wait_for_action_server(self, client, name):
        """Wait for an action server to become available."""
        self.get_logger().info(f'Waiting for {name} action server...')
        while not client.wait_for_server(timeout_sec=2.0) and rclpy.ok():
            self.get_logger().info(f'{name} action server not available, waiting again...')
        if rclpy.ok():
            self.get_logger().info(f'{name} action server found.')
        else:
            self.get_logger().error(f'ROS shutdown while waiting for {name} server.')
            raise SystemExit('ROS shutdown')


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