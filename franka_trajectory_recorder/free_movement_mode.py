import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from franka_msgs.action import Homing, Move, Grasp
from sensor_msgs.msg import JointState
import csv
import os
import time
import threading
import sys
import select
import tty
import termios
import h5py
import numpy as np

"""Free Movement Mode / Teleoperation for Franka Emika Panda Robot for recording robot arm and gripper movements."""

class FreeMovementMode(Node):
    def __init__(self):
        super().__init__('free_movement_mode')
        # Flags for recording and pausing
        self.recording = False
        self.paused = False
        # Initialize trajectory storage
        self.trajectory = []
        self.save_path_csv = os.path.expanduser('~/trajectory.csv')
        self.save_path_hdf5 = os.path.expanduser('~/trajectory.h5')
        # Create a lock for thread safety
        self.lock = threading.Lock()

        # Create a subscription to the joint states topic
        self.joint_state_sub = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_callback, 
            10)

        # Create a subscription to the gripper state topic
        self.gripper_subscription = self.create_subscription(
            JointState,
            '/fr3_gripper/joint_states',
            self.gripper_state_callback,
            10
        )

        # Initialize gripper state
        self.gripper_state = None  # Will hold the gripper state dynamically
        self.gripper_goal_state = 'open'  # Start with the gripper in the "open" state

        # Gripper control initialization
        self.gripper_max_width = 0.08  # Max width for Franka Hand
        self.gripper_speed = 0.5  # Default speed (m/s)
        self.gripper_force = 30.0  # Default grasp force (N)
        self.gripper_epsilon_inner = 0.05  # Tolerance for successful grasp
        self.gripper_epsilon_outer = 0.05

        # Action clients for gripper
        self.homing_client = ActionClient(self, Homing, '/fr3_gripper/homing')
        self.move_client = ActionClient(self, Move, '/fr3_gripper/move')
        self.grasp_client = ActionClient(self, Grasp, '/fr3_gripper/grasp')

        # Wait for gripper action servers
        self.wait_for_action_server(self.homing_client, 'Homing')
        self.wait_for_action_server(self.move_client, 'Move')
        self.wait_for_action_server(self.grasp_client, 'Grasp')

        # Perform initial homing
        self.home_gripper()

        # Display instructions
        self.get_logger().info(
            "\n================ Free Movement Mode =================\n"
            "Press the following keys for corresponding actions:\n"
            "  [r] - Start/Pause recording\n"
            "  [f] - Finish recording and save trajectory\n"
            "  [c] - Close the gripper\n"
            "  [o] - Open the gripper\n"
            "  [b] - Toggle gripper state (via foot pedal)\n"
            "====================================================="
        )

        # Start a thread to listen for keyboard inputs
        threading.Thread(target=self.keyboard_listener, daemon=True).start()

    def joint_state_callback(self, msg):
        # This callback is called when in recording mode
        if self.recording and not self.paused:
            with self.lock:
                timestamp = time.time()

                # Extract joint positions
                joint_positions = list(msg.position)[:7]  # Only take the joint positions
                # Add the gripper state
                joint_positions.extend(self.gripper_state)

                self.trajectory.append({
                    'timestamp': timestamp,
                    'joint_positions': joint_positions,
                    'joint_velocities': msg.velocity,
                    'gripper_state': self.gripper_goal_state  # Add gripper state to trajectory
                })

    def gripper_state_callback(self, msg):
        # Update the gripper state dynamically
        self.gripper_state = list(msg.position)

    # Runs in a separate thread to listen for keyboard inputs
    def keyboard_listener(self):
        # Save terminal settings
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        try:
            while True:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key == 'r':  # Start or pause recording
                        self.toggle_recording()
                    elif key == 'f':  # Finish recording
                        self.finish_recording()
                    elif key == 'b':  # Foot pedal pressed
                        # Toggle gripper state
                        if self.gripper_goal_state == 'open':
                            self.close_gripper()
                            self.gripper_goal_state = 'closed'
                        elif self.gripper_goal_state == 'closed':
                            self.open_gripper()
                            self.gripper_goal_state = 'open'
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def toggle_recording(self):
        if not self.recording:
            self.recording = True
            self.paused = False
            self.get_logger().info("Recording started.")
        elif self.recording and not self.paused:
            self.paused = True
            self.get_logger().info("Recording paused.")
        elif self.recording and self.paused:
            self.paused = False
            self.get_logger().info("Recording resumed.")

    def finish_recording(self):
        if self.recording:
            self.recording = False
            self.paused = False
            self.get_logger().info("Recording finished. Saving trajectory...")
            self.save_trajectory()
        else:
            self.get_logger().info("No recording in progress to finish.")

    # Save the trajectory to CSV and HDF5 files
    def save_trajectory(self):
        with self.lock:
            # Save to CSV
            with open(self.save_path_csv, 'w', newline='') as csvfile:
                fieldnames = ['timestamp', 'joint_positions', 'joint_velocities', 'gripper_state']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                for entry in self.trajectory:
                    writer.writerow({
                        'timestamp': entry['timestamp'],
                        'joint_positions': ','.join(map(str, entry['joint_positions'])),
                        'joint_velocities': ','.join(map(str, entry['joint_velocities'])),
                        'gripper_state': entry['gripper_state']
                    })
            self.get_logger().info(f"Trajectory saved to {self.save_path_csv}")

            # Save to HDF5
            with h5py.File(self.save_path_hdf5, 'w') as hdf5file:
                timestamps = [entry['timestamp'] for entry in self.trajectory]
                joint_positions = [entry['joint_positions'] for entry in self.trajectory]
                joint_velocities = [entry['joint_velocities'] for entry in self.trajectory]
                gripper_states = [entry['gripper_state'] for entry in self.trajectory]

                hdf5file.create_dataset('timestamps', data=timestamps)
                hdf5file.create_dataset('joint_positions', data=joint_positions)
                hdf5file.create_dataset('joint_velocities', data=joint_velocities)
                hdf5file.create_dataset('gripper_state', data=np.string_(gripper_states))  # Save gripper state as strings

            self.get_logger().info(f"Trajectory saved to {self.save_path_hdf5}")

            # Clear the trajectory
            self.trajectory = []

    def wait_for_action_server(self, client, name):
        self.get_logger().info(f'Waiting for {name} action server...')
        while not client.wait_for_server(timeout_sec=2.0) and rclpy.ok():
            self.get_logger().info(f'{name} action server not available, waiting again...')
        if rclpy.ok():
            self.get_logger().info(f'{name} action server found.')
        else:
            self.get_logger().error(f'ROS shutdown while waiting for {name} server.')
            raise SystemExit('ROS shutdown')

    # Define gripper control methods

    def home_gripper(self):
        self.get_logger().info("Sending homing goal...")
        goal_msg = Homing.Goal()
        self.homing_client.send_goal_async(goal_msg)
        self.gripper_goal_state = 'open'
        self.get_logger().info("Homing goal sent.")

    def close_gripper(self):
        self.get_logger().info("Sending close gripper goal...")
        goal_msg = Grasp.Goal()
        goal_msg.width = 0.0
        goal_msg.speed = self.gripper_speed
        goal_msg.force = self.gripper_force
        goal_msg.epsilon.inner = self.gripper_epsilon_inner
        goal_msg.epsilon.outer = self.gripper_epsilon_outer
        self.grasp_client.send_goal_async(goal_msg)
        self.gripper_goal_state = 'closed'
        self.get_logger().info("Gripper closed.")

    def open_gripper(self):
        self.get_logger().info("Sending open gripper goal...")
        goal_msg = Move.Goal()
        goal_msg.width = self.gripper_max_width
        goal_msg.speed = self.gripper_speed
        self.move_client.send_goal_async(goal_msg)
        self.gripper_goal_state = 'open'
        self.get_logger().info("Gripper opened.")

def main(args=None):
    rclpy.init(args=args)
    node = FreeMovementMode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Free Movement Mode.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()