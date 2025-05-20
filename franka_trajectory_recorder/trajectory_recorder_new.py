import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from franka_msgs.action import Homing, Move, Grasp
from sensor_msgs.msg import JointState
from franka_msgs.msg import FrankaRobotState
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

class TrajectoryRecorderNew(Node):
    def __init__(self):
        super().__init__('trajectory_recorder_new')
        self.recording = False
        self.paused = False
        self.trajectory = []
        self.save_path_hdf5 = os.path.expanduser('~/franka_ros2_ws/src/franka_trajectory_recorder/trajectories/dataset.hdf5')
        self.lock = threading.Lock()
        
        # Initialize the robot root pose and velocity
        self.initial_state = {
            'joint_position': None,
            'joint_velocity': None,
            'root_pose': None,
            'root_velocity': None
        }

        # Initialize placeholders for latest joint positions and velocities
        self.latest_joint_positions = None
        self.latest_joint_velocities = None

        # Hardcoded rigid object data - Implement vision system to get this data
        # As soon as objects are grasped we can simply update the root_pose and root_velocity of the rigid objects to the ee pose and velocity
        self.rigid_objects = {
            'cube_1': {
                'root_pose': np.array([[0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]], dtype=np.float32),  # [x, y, z, qx, qy, qz, qw]
                'root_velocity': np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], dtype=np.float32)  # [vx, vy, vz, wx, wy, wz]
            },
            'cube_2': {
                'root_pose': np.array([[0.4, 0.5, 0.6, 0.0, 0.0, 0.0, 1.0]], dtype=np.float32),
                'root_velocity': np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], dtype=np.float32)
            },
            'cube_3': {
                'root_pose': np.array([[0.7, 0.8, 0.9, 0.0, 0.0, 0.0, 1.0]], dtype=np.float32),
                'root_velocity': np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], dtype=np.float32)
            }
        }

        # ----------------------------- Initialize subscribers -----------------------------
        # Joint state subscription
        self.joint_state_sub = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_callback, 
            10)
        
        # Gripper state subscription
        self.gripper_subscription = self.create_subscription(
            JointState, 
            '/fr3_gripper/joint_states', 
            self.gripper_state_callback, 
            10)
        
        # Franka state callback 
        self.franka_state = self.create_subscription(
            FrankaRobotState,
            '/franka_robot_state_broadcaster/robot_state',
            self.franka_state_callback,
            10
        )

        # Gripper state
        self.gripper_state = 'open'
        self.gripper_goal_state = 'open'  # Start with the gripper in the "open" state

        # Gripper control
        self.gripper_max_width = 0.08
        self.gripper_speed = 0.5
        self.gripper_force = 50.0
        self.gripper_epsilon_inner = 0.05
        self.gripper_epsilon_outer = 0.05

        # Action clients for gripper control
        self.homing_client = ActionClient(self, Homing, '/fr3_gripper/homing')
        self.move_client = ActionClient(self, Move, '/fr3_gripper/move')
        self.grasp_client = ActionClient(self, Grasp, '/fr3_gripper/grasp')

        # Wait for action servers
        self.wait_for_action_server(self.homing_client, 'Homing')
        self.wait_for_action_server(self.move_client, 'Move')
        self.wait_for_action_server(self.grasp_client, 'Grasp')

        # Perform initial homing
        self.home_gripper()

        # Display instructions
        self.get_logger().info(
            "\n================ Trajectory Recorder =================\n"
            "Press the following keys for corresponding actions:\n"
            "  [r] - Start/Pause recording\n"
            "  [f] - Finish recording and save trajectory\n"
            "  [b] - Open/Close gripper state\n"
            "====================================================="
        )

        # Start keyboard listener
        threading.Thread(target=self.keyboard_listener, daemon=True).start()

    # ----------------------------- Callbacks -----------------------------
    # Joint state callback -> Joint positions and velocities
    def joint_state_callback(self, msg):
        # Store the latest joint positions and velocities
        self.latest_joint_positions = list(msg.position)[:7]  # First 7 joint positions
        self.latest_joint_velocities = list(msg.velocity)[:7]  # First 7 joint velocities

        if self.recording and not self.paused:
            with self.lock:
                timestamp = time.time()
                
                # Append the gripper position twice, gripper_state is from the gripper state callback
                gripper_position = self.gripper_state[0] if isinstance(self.gripper_state, list) else 0.0
                joint_positions = self.latest_joint_positions + [gripper_position, gripper_position]
                
                # Append the data to the trajectory
                self.trajectory.append({
                    'timestamp': timestamp,
                    'joint_positions': joint_positions,
                    'joint_velocities': self.latest_joint_velocities,
                    'gripper_state': self.gripper_goal_state,
                    'robot_root_pose': self.robot_root_pose,
                    'robot_root_velocity': self.robot_root_velocity
                })
            self.get_logger().debug(f"Joint state recorded: {joint_positions}, {self.latest_joint_velocities}")

    # Gripper state callback -> Gripper position
    def gripper_state_callback(self, msg):
        self.gripper_state = list(msg.position)  # Store the gripper state dynamically
        self.get_logger().debug(f"Gripper state updated: {self.gripper_state}")


    # Franka state callback -> Root pose and velocity
    def franka_state_callback(self, msg: FrankaRobotState):
        position = msg.o_t_ee.pose.position
        orientation = msg.o_t_ee.pose.orientation
        # Root pose is a 7-element array [x, y, z, qx, qy, qz, qw]
        self.robot_root_pose = np.array([
            position.x, position.y, position.z,
            orientation.x, orientation.y, orientation.z, orientation.w
        ], dtype=np.float32)

        # Extract root_velocity (linear + angular velocity as a 6-element array)
        # Assuming o_d_p_ee is a 6-element array [vx, vy, vz, wx, wy, wz]
        #self.robot_root_velocity = np.array(msg._f_t_ee, dtype=np.float32)
        # Set root velocity to zero for now
        self.robot_root_velocity = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)

        self.get_logger().debug(f"Updated root_pose: {self.robot_root_pose}, root_velocity: {self.robot_root_velocity}")

    def keyboard_listener(self):
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while True:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key == 'r':
                        self.toggle_recording()
                    elif key == 'f':
                        self.finish_recording()
                    elif key == 'b':
                        if self.gripper_goal_state == 'open':
                            self.close_gripper()
                            self.gripper_goal_state = 'closed'
                        elif self.gripper_goal_state == 'closed':
                            self.open_gripper()
                            self.gripper_goal_state = 'open'
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def toggle_recording(self):
        if not self.recording:
            self.recording = True
            self.paused = False
            self.get_logger().info("Recording started.")

            # Capture initial state of the robot when recording starts
            with self.lock:
                if self.latest_joint_positions is not None and self.latest_joint_velocities is not None:
                    self.initial_state['joint_position'] = self.latest_joint_positions + [self.gripper_state[0], self.gripper_state[0]]
                    self.initial_state['joint_velocity'] = np.zeros(9, dtype=np.float32)
                    self.initial_state['root_pose'] = self.robot_root_pose
                    self.initial_state['root_velocity'] = np.zeros(6, dtype=np.float32)
                else:
                    self.get_logger().warning("No joint states available to initialize the recording.")
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

    # ----------------------------- Save trajectory to HDF5 -----------------------------
    # The data is stored in the following structure:
    # /data
    #   ├── demo_0
    #   │   ├── num_samples
    #   │   ├── success
    #   │   ├── actions
    #   │   ├── initial_state
    #   │   │   ├── articulation
    #   │   │   │   ├── robot
    #   │   │   │       ├── joint_position
    #   │   │   │       ├── joint_velocity
    #   │   │   │       ├── root_pose
    #   │   │   │       ├── root_velocity
    #   │   │       ├── rigid_object
    #   │   │   │       ├── cube_1
    #   │   │   │       ├── cube_2
    #   │       │       ├── cube_3
    #   │   ├── obs
    #   │   │   ├── actions
    #   │   │   ├── cube_orientations
    #   │   │   ├── cube_positions
    #   │   │   ├── eef_pos
    #   │   │   ├── eef_quat
    #   │   │   ├── gripper_pos
    #   │   │   ├── joint_pos
    #   │   │   ├── joint_vel
    #   │   │   ├── object
    def save_trajectory(self):
        with self.lock:
            with h5py.File(self.save_path_hdf5, 'a') as hdf5file:
                # Ensure the /data group exists
                if 'data' not in hdf5file:
                    data_group = hdf5file.create_group('data')
                    data_group.attrs['env_args'] = '{"env_name": "Isaac-Stack-Cube-Franka-IK-Rel-v0", "type": 2}'
                    data_group.attrs['total'] = 0
                else:
                    data_group = hdf5file['data']

                # Determine the next demo group name
                demo_index = len(data_group.keys())
                demo_group_name = f'demo_{demo_index}'
                demo_group = data_group.create_group(demo_group_name)

                # Add attributes to the demo group
                demo_group.attrs['num_samples'] = len(self.trajectory)
                demo_group.attrs['success'] = np.bool_(True)

                # Dynamically populate the 'actions' dataset
                actions = [entry['joint_positions'][:7] for entry in self.trajectory]
                demo_group.create_dataset('actions', data=np.array(actions, dtype=np.float32))

                # -------------------------- Add the initial_state group --------------------------
                initial_state = demo_group.create_group('initial_state')
                articulation = initial_state.create_group('articulation')
                robot = articulation.create_group('robot')

                # Save joint_position with shape (1, 9)
                robot.create_dataset(
                    'joint_position',
                    data=np.expand_dims(np.array(self.initial_state['joint_position'], dtype=np.float32), axis=0)
                )

                # Save joint_velocity with shape (1, 9)
                robot.create_dataset(
                    'joint_velocity',
                    data=np.expand_dims(np.array(self.initial_state['joint_velocity'], dtype=np.float32), axis=0)
                )

                # Save root_pose with shape (1, 7)
                robot.create_dataset(
                    'root_pose',
                    data=np.expand_dims(np.array(self.initial_state['root_pose'], dtype=np.float32), axis=0)
                )

                # Save root_velocity with shape (1, 6)
                robot.create_dataset(
                    'root_velocity',
                    data=np.expand_dims(np.array(self.initial_state['root_velocity'], dtype=np.float32), axis=0)
                )

                # -------------------------- Add the rigid_object group --------------------------
                rigid_object = initial_state.create_group('rigid_object')
                for name, data in self.rigid_objects.items():
                    cube = rigid_object.create_group(name)
                    cube.create_dataset('root_pose', data=data['root_pose'])
                    cube.create_dataset('root_velocity', data=data['root_velocity'])

                # --------------------------- Add the obs group ----------------------------------             
                obs = demo_group.create_group('obs')
                num_samples = len(self.trajectory)

                # Dynamically populate the 'actions' dataset
                obs.create_dataset('actions', data=np.array(actions, dtype=np.float32))

                # Dynamically populate the 'cube_orientations' and 'cube_positions' datasets
                cube_orientations = []
                cube_positions = []
                for _ in range(num_samples):
                    # Extract orientations (quaternion: qx, qy, qz, qw) for each cube
                    orientation_cube_1 = self.rigid_objects['cube_1']['root_pose'][0, 3:]
                    orientation_cube_2 = self.rigid_objects['cube_2']['root_pose'][0, 3:]
                    orientation_cube_3 = self.rigid_objects['cube_3']['root_pose'][0, 3:]
                    concatenated_orientations = np.concatenate((orientation_cube_1, orientation_cube_2, orientation_cube_3))
                    cube_orientations.append(concatenated_orientations)

                    # Extract positions (x, y, z) for each cube
                    position_cube_1 = self.rigid_objects['cube_1']['root_pose'][0, :3]
                    position_cube_2 = self.rigid_objects['cube_2']['root_pose'][0, :3]
                    position_cube_3 = self.rigid_objects['cube_3']['root_pose'][0, :3]
                    concatenated_positions = np.concatenate((position_cube_1, position_cube_2, position_cube_3))
                    cube_positions.append(concatenated_positions)

                obs.create_dataset('cube_orientations', data=np.array(cube_orientations, dtype=np.float32))
                obs.create_dataset('cube_positions', data=np.array(cube_positions, dtype=np.float32))

                # Dynamically populate the 'eef_pos', 'eef_quat' datasets
                # Extract x,y,z
                eef_pos = [entry['robot_root_pose'][:3] for entry in self.trajectory]
                # Extract qx,qy,qz,qw
                eef_quat = [entry['robot_root_pose'][3:] for entry in self.trajectory]
                # Create datasets
                obs.create_dataset('eef_pos', data=np.array(eef_pos), dtype=np.float32)
                obs.create_dataset('eef_quat', data=np.array(eef_quat), dtype=np.float32)

                # Dynamically populate the 'gripper_pos' dataset
                gripper_pos = [[entry['joint_positions'][-2], entry['joint_positions'][-1]] for entry in self.trajectory]
                obs.create_dataset('gripper_pos', data=np.array(gripper_pos, dtype=np.float32))
                
                # Dynamically populate the 'joint_pos', 'joint_vel' datasets
                joint_pos = [entry['joint_positions'] for entry in self.trajectory]
                joint_vel = [entry['joint_velocities'] for entry in self.trajectory]
                obs.create_dataset('joint_pos', data=np.array(joint_pos, dtype=np.float32))
                obs.create_dataset('joint_vel', data=np.array(joint_vel, dtype=np.float32))

                # ------------------------ Add the 'object' dataset --------------------------
                # The object dataset in the obs group contains info about the stae of objects in the env at each timestep
                # Dataset is storing information for three cubes each with 13 features
                # Obtain the data from the vision system
                # The features are: [x, y, z, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz]
                obs.create_dataset('object', data=np.zeros((num_samples, 39), dtype=np.float32))

                # # ------------------------ Add the 'states' dataset --------------------------
                # # Fill with zeros to maintain compativility with the original code
                # states = demo_group.create_group('states')
                # articulation_states = states.create_group('articulation')
                # robot_states = articulation_states.create_group('robot')
                # robot_states.create_dataset('joint_position', data=np.zeros((num_samples, 9), dtype=np.float32))
                # robot_states.create_dataset('joint_velocity', data=np.zeros((num_samples, 9), dtype=np.float32))
                # robot_states.create_dataset('root_pose', data=np.zeros((num_samples, 7), dtype=np.float32))
                # robot_states.create_dataset('root_velocity', data=np.zeros((num_samples, 6), dtype=np.float32))

                # rigid_object_states = states.create_group('rigid_object')
                # for name, data in self.rigid_objects.items():
                #     cube_states = rigid_object_states.create_group(name)
                #     cube_states.create_dataset('root_pose', data=np.tile(data['root_pose'], (num_samples, 1)))
                #     cube_states.create_dataset('root_velocity', data=np.tile(data['root_velocity'], (num_samples, 1)))

                # Update the total number of samples in the data group
                if 'total' in data_group.attrs:
                    data_group.attrs['total'] += len(self.trajectory)
                else:
                    data_group.attrs['total'] = len(self.trajectory)

            self.get_logger().info(f"Trajectory saved to {self.save_path_hdf5} under group {demo_group_name}")
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

    def home_gripper(self):
        self.get_logger().info("Sending homing goal...")
        goal_msg = Homing.Goal()
        self.homing_client.send_goal_async(goal_msg)
        self.gripper_state = 'open'
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
        self.gripper_state = 'closed'
        self.get_logger().info("Gripper closed.")

    def open_gripper(self):
        self.get_logger().info("Sending open gripper goal...")
        goal_msg = Move.Goal()
        goal_msg.width = self.gripper_max_width
        goal_msg.speed = self.gripper_speed
        self.move_client.send_goal_async(goal_msg)
        self.gripper_state = 'open'
        self.get_logger().info("Gripper opened.")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorderNew()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Trajectory Recorder.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()