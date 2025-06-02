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

        # Declare and get parameters
        self.declare_parameter('file_path', '~/trajectory.h5')
        self.declare_parameter('playback_rate', 1.0)
    
        # Get file path and playback rate from parameters
        self.file_path = os.path.expanduser(self.get_parameter('file_path').value)
        self.playback_rate = self.get_parameter('playback_rate').value

        # Gripper parameters
        self.gripper_speed = 0.5  # Default speed (m/s)
        self.gripper_force = 50.0  # Default grasp force (N)
        self.gripper_max_width = 0.08  # Maximum gripper width (m)
        self.gripper_epsilon_inner = 0.5 # Inner tolerance for grasping
        self.gripper_epsilon_outer = 0.5  # Outer tolerance for grasping

        # Publisher for joint positions -> Cartesian Impedance Controller subscription
        self.joint_positions_publisher = self.create_publisher(
            Float64MultiArray,
            '/trajectory_playback/joint_positions',
            10
        )

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

        # Prepare playback
        self.current_index = 0
        self.previous_gripper_state = None
        self.timer = None
        # Start playback
        self.start_playback()

    def load_trajectory(self, file_path):
        if file_path.endswith('.h5'):
            return self.load_hdf5(file_path)
        elif file_path.endswith('.csv'):
            return self.load_csv(file_path)
        else:
            self.get_logger().error("Unsupported file format. Use .h5 or .csv.")
            return None

    # Load trajectory data from HDF5 file
    def load_hdf5(self, file_path):
        try:
            with h5py.File(file_path, 'r') as hdf5_file:
                timestamps = hdf5_file['timestamps'][:]
                joint_positions = hdf5_file['joint_positions'][:, :7]
                gripper_states = hdf5_file['gripper_state'][:].astype(str)
                return {'timestamps': timestamps, 'joint_positions': joint_positions, 'gripper_state': gripper_states}
        except Exception as e:
            self.get_logger().error(f"Error loading HDF5 file: {e}")
            return None

    # Load trajectory data from CSV file
    def load_csv(self, file_path):
        try:
            timestamps = []
            joint_positions = []
            gripper_states = []
            with open(file_path, 'r') as csv_file:
                reader = csv.DictReader(csv_file)
                for row in reader:
                    timestamps.append(float(row['timestamp']))
                    positions = [float(x) for x in row['joint_positions'].split(',')[:7]]
                    joint_positions.append(positions)
                    gripper_states.append(row['gripper_state'])
            return {'timestamps': np.array(timestamps), 'joint_positions': np.array(joint_positions), 'gripper_state': np.array(gripper_states)}
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
        if self.current_index >= len(self.trajectory['joint_positions']):
            self.get_logger().info("Trajectory playback completed.")
            self.timer.cancel()
            return

        # Extract joint positions and gripper state
        joint_positions = self.trajectory['joint_positions'][self.current_index]
        current_gripper_state = self.trajectory['gripper_state'][self.current_index]

        # Publish joint positions to the /trajectory_playback/joint_positions topic -> Cartesian Impedance Controller subscription
        msg = Float64MultiArray()
        msg.data = joint_positions.tolist()
        self.joint_positions_publisher.publish(msg)
        self.get_logger().info(f"Published joint positions: {joint_positions} to /trajectory_playback/joint_positions")

        # Handle gripper state transition
        self.handle_gripper_state(current_gripper_state)

        # Move to the next point
        self.current_index += 1
        # Schedule the next point and cancel the previous timer
        if self.current_index < len(self.intervals):
            self.timer.cancel()
            self.timer = self.create_timer(self.intervals[self.current_index], self.publish_next_point)

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
