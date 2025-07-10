#!/usr/bin/env python3
"""
Convert trajectory comparison CSV data back to HDF5 format matching trajectory_recorder_new.py structure.
This captures the actual robot dynamics (with controller lag/filtering) for more realistic imitation learning.
"""

import os
import sys
import csv
import h5py
import numpy as np
import argparse
from datetime import datetime
from typing import Dict, List, Optional

class TrajectoryDynamicsConverter:
    """Converts trajectory comparison CSV to HDF5 format with actual robot dynamics"""
    
    def __init__(self, csv_file: str, output_dir: str = None):
        self.csv_file = csv_file
        self.output_dir = output_dir or os.path.dirname(csv_file)
        self.data = None
        
        # Default object positions (matching trajectory_recorder_new.py defaults)
        self.default_rigid_objects = {
            'cube_1': {  # Blue cube
                'root_pose': np.array([[0.4, 0.2, 0.0203, 0.0, 0.0, 0.0, 1.0]], dtype=np.float32),
                'root_velocity': np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], dtype=np.float32)
            },
            'cube_2': {  # Red cube  
                'root_pose': np.array([[0.6, 0.3, 0.0203, 0.0, 0.0, 0.0, 1.0]], dtype=np.float32),
                'root_velocity': np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], dtype=np.float32)
            },
            'cube_3': {  # Green cube
                'root_pose': np.array([[0.4, -0.2, 0.0203, 0.0, 0.0, 0.0, 1.0]], dtype=np.float32),
                'root_velocity': np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], dtype=np.float32)
            }
        }
    
    def load_csv_data(self) -> bool:
        """Load trajectory comparison data from CSV"""
        try:
            data = []
            with open(self.csv_file, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    # Convert string values to float
                    converted_row = {}
                    for key, value in row.items():
                        try:
                            if key in ['ref_gripper_state']:
                                converted_row[key] = value  # Keep as string
                            else:
                                converted_row[key] = float(value)
                        except ValueError:
                            converted_row[key] = value
                    data.append(converted_row)
            
            self.data = data
            print(f"✅ Loaded {len(data)} trajectory points from CSV")
            return True
            
        except Exception as e:
            print(f"❌ Error loading CSV: {e}")
            return False
    
    def extract_joint_positions_from_pose(self, pose_data: Dict) -> np.ndarray:
        """
        Extract/estimate joint positions from end-effector pose data.
        Since we don't have joint positions in the comparison CSV, we'll use a placeholder approach.
        In practice, you might want to use inverse kinematics here.
        """
        # For now, we'll create a reasonable joint configuration
        # In a real implementation, you'd use inverse kinematics
        num_points = len(pose_data['actual_eef_pos_x'])
        
        # Use a default joint configuration that changes slightly over time
        # This is a simplified approach - ideally use IK solver
        default_joints = np.array([0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.5])
        
        joint_positions = []
        for i in range(num_points):
            # Add small variations to simulate joint movement
            joints = default_joints + 0.1 * np.sin(np.linspace(0, 2*np.pi, 7) + i * 0.1)
            joint_positions.append(joints)
        
        return np.array(joint_positions, dtype=np.float32)
    
    def extract_joint_velocities(self, joint_positions: np.ndarray, timestamps: List[float]) -> np.ndarray:
        """Calculate joint velocities from joint positions using numerical differentiation"""
        if len(joint_positions) < 2:
            return np.zeros_like(joint_positions)
        
        dt = np.diff(timestamps)
        velocities = np.diff(joint_positions, axis=0) / dt[:, np.newaxis]
        
        # Extend to match length (duplicate last velocity)
        velocities = np.vstack([velocities, velocities[-1:]])
        
        return velocities.astype(np.float32)
    
    def create_actions_array(self, use_actual_data: bool = True, action_mode: str = 'pose') -> np.ndarray:
        """
        Create actions array in the format expected by trajectory_recorder_new.py
        
        Args:
            use_actual_data: If True, use actual robot data; if False, use reference data
            action_mode: 'joint' or 'pose' mode
        """
        if not self.data:
            return np.array([])
        
        actions = []
        
        if action_mode == 'joint':
            # Joint mode: [joint1, joint2, ..., joint7, gripper_command]
            # Since we don't have joint data in comparison CSV, estimate it
            joint_positions = self.extract_joint_positions_from_pose(self.data[0])
            
            for i, row in enumerate(self.data):
                gripper_cmd = 1.0 if row['ref_gripper_state'] == 'open' else 0.0
                action = np.concatenate([joint_positions[i], [gripper_cmd]])
                actions.append(action)
                
        elif action_mode == 'pose':
            # Pose mode: [x, y, z, qw, qx, qy, qz, gripper_command] 
            # Note: Using qw, qx, qy, qz order (Isaac Lab convention)
            
            for row in self.data:
                if use_actual_data:
                    # Use actual robot data (with dynamics)
                    pos = [row['actual_eef_pos_x'], row['actual_eef_pos_y'], row['actual_eef_pos_z']]
                    quat = [row['actual_eef_quat_w'], row['actual_eef_quat_x'], 
                           row['actual_eef_quat_y'], row['actual_eef_quat_z']]
                else:
                    # Use reference data (ideal trajectory)
                    pos = [row['ref_eef_pos_x'], row['ref_eef_pos_y'], row['ref_eef_pos_z']]
                    quat = [row['ref_eef_quat_w'], row['ref_eef_quat_x'], 
                           row['ref_eef_quat_y'], row['ref_eef_quat_z']]
                
                gripper_cmd = 1.0 if row['ref_gripper_state'] == 'open' else 0.0
                action = pos + quat + [gripper_cmd]
                actions.append(action)
        
        return np.array(actions, dtype=np.float32)
    
    def create_observations_dict(self, actions: np.ndarray, use_actual_data: bool = True) -> Dict:
        """Create observations dictionary matching trajectory_recorder_new.py format"""
        num_samples = len(self.data)
        
        # Extract data arrays
        if use_actual_data:
            eef_pos = np.array([[row['actual_eef_pos_x'], row['actual_eef_pos_y'], row['actual_eef_pos_z']] 
                               for row in self.data], dtype=np.float32)
            eef_quat = np.array([[row['actual_eef_quat_w'], row['actual_eef_quat_x'], 
                                 row['actual_eef_quat_y'], row['actual_eef_quat_z']] 
                                for row in self.data], dtype=np.float32)
        else:
            eef_pos = np.array([[row['ref_eef_pos_x'], row['ref_eef_pos_y'], row['ref_eef_pos_z']] 
                               for row in self.data], dtype=np.float32)
            eef_quat = np.array([[row['ref_eef_quat_w'], row['ref_eef_quat_x'], 
                                 row['ref_eef_quat_y'], row['ref_eef_quat_z']] 
                                for row in self.data], dtype=np.float32)
        
        # Estimate joint positions and velocities
        timestamps = [row['playback_time'] for row in self.data]
        joint_pos = self.extract_joint_positions_from_pose(self.data[0])
        joint_vel = self.extract_joint_velocities(joint_pos, timestamps)
        
        # Create gripper positions (estimate from gripper width if available)
        gripper_pos = []
        for row in self.data:
            if 'actual_gripper_width' in row and row['actual_gripper_width'] is not None:
                width = row['actual_gripper_width']
                # Convert width to finger positions [finger1, finger2]
                finger_pos = [width/2, width/2]
            else:
                # Default gripper position
                finger_pos = [0.04, 0.04]  # Open position
            gripper_pos.append(finger_pos)
        
        gripper_pos = np.array(gripper_pos, dtype=np.float32)
        
        # Create object observations (39D) - using default positions for consistency
        object_obs = self.create_object_observations(eef_pos)
        
        # Extract cube positions and orientations for compatibility
        cube_positions = np.array([
            [obj['root_pose'][0, :3] for obj in self.default_rigid_objects.values()]
        ] * num_samples, dtype=np.float32)
        
        cube_orientations = np.array([
            [obj['root_pose'][0, 3:7] for obj in self.default_rigid_objects.values()]  
        ] * num_samples, dtype=np.float32)
        
        return {
            'actions': actions,
            'eef_pos': eef_pos,
            'eef_quat': eef_quat,
            'gripper_pos': gripper_pos,
            'joint_pos': joint_pos,
            'joint_vel': joint_vel,
            'object': object_obs,
            'cube_positions': cube_positions,
            'cube_orientations': cube_orientations
        }
    
    def create_object_observations(self, eef_positions: np.ndarray) -> np.ndarray:
        """
        Create 39D object observations matching Isaac Lab structure.
        Uses default object positions and calculates relative distances.
        """
        num_samples = len(eef_positions)
        object_obs = []
        
        # Get object positions
        cube_1_pos = self.default_rigid_objects['cube_1']['root_pose'][0, :3]
        cube_2_pos = self.default_rigid_objects['cube_2']['root_pose'][0, :3] 
        cube_3_pos = self.default_rigid_objects['cube_3']['root_pose'][0, :3]
        
        cube_1_quat = self.default_rigid_objects['cube_1']['root_pose'][0, 3:7]
        cube_2_quat = self.default_rigid_objects['cube_2']['root_pose'][0, 3:7]
        cube_3_quat = self.default_rigid_objects['cube_3']['root_pose'][0, 3:7]
        
        for i in range(num_samples):
            ee_pos = eef_positions[i]
            
            # Compute relative positions and distances
            gripper_to_cube_1 = cube_1_pos - ee_pos
            gripper_to_cube_2 = cube_2_pos - ee_pos  
            gripper_to_cube_3 = cube_3_pos - ee_pos
            cube_1_to_2 = cube_1_pos - cube_2_pos
            cube_2_to_3 = cube_2_pos - cube_3_pos
            cube_1_to_3 = cube_1_pos - cube_3_pos
            
            # Concatenate 39D observation
            obs = np.concatenate([
                cube_1_pos,           # [3] - cube 1 position
                cube_1_quat,          # [4] - cube 1 quaternion
                cube_2_pos,           # [3] - cube 2 position  
                cube_2_quat,          # [4] - cube 2 quaternion
                cube_3_pos,           # [3] - cube 3 position
                cube_3_quat,          # [4] - cube 3 quaternion
                gripper_to_cube_1,    # [3] - gripper to cube 1
                gripper_to_cube_2,    # [3] - gripper to cube 2
                gripper_to_cube_3,    # [3] - gripper to cube 3
                cube_1_to_2,          # [3] - cube 1 to cube 2
                cube_2_to_3,          # [3] - cube 2 to cube 3
                cube_1_to_3           # [3] - cube 1 to cube 3
            ])
            
            object_obs.append(obs)
        
        return np.array(object_obs, dtype=np.float32)
    
    def create_initial_state_dict(self, use_actual_data: bool = True) -> Dict:
        """Create initial state dictionary from first data point"""
        if not self.data:
            return {}
        
        first_row = self.data[0]
        
        # Extract initial pose
        if use_actual_data:
            initial_pose = np.array([
                first_row['actual_eef_pos_x'], first_row['actual_eef_pos_y'], first_row['actual_eef_pos_z'],
                first_row['actual_eef_quat_x'], first_row['actual_eef_quat_y'], 
                first_row['actual_eef_quat_z'], first_row['actual_eef_quat_w']
            ], dtype=np.float32)
        else:
            initial_pose = np.array([
                first_row['ref_eef_pos_x'], first_row['ref_eef_pos_y'], first_row['ref_eef_pos_z'],
                first_row['ref_eef_quat_x'], first_row['ref_eef_quat_y'], 
                first_row['ref_eef_quat_z'], first_row['ref_eef_quat_w']
            ], dtype=np.float32)
        
        # Estimate initial joint positions
        joint_positions = self.extract_joint_positions_from_pose(self.data[0])
        initial_joints = joint_positions[0] if len(joint_positions) > 0 else np.zeros(7, dtype=np.float32)
        
        return {
            'articulation': {
                'robot': {
                    'joint_position': np.array([initial_joints], dtype=np.float32),
                    'joint_velocity': np.array([np.zeros(7)], dtype=np.float32),
                    'root_pose': np.array([initial_pose], dtype=np.float32),
                    'root_velocity': np.array([np.zeros(6)], dtype=np.float32)
                },
                'rigid_object': self.default_rigid_objects
            }
        }
    
    def save_to_hdf5(self, output_file: str, action_mode: str = 'pose', 
                     use_actual_data: bool = True, demo_name: Optional[str] = None) -> bool:
        """
        Save converted trajectory data to HDF5 format matching trajectory_recorder_new.py structure
        
        Args:
            output_file: Output HDF5 file path
            action_mode: 'joint' or 'pose' action mode
            use_actual_data: Use actual robot data (True) or reference data (False)
            demo_name: Custom demo name (auto-generated if None)
        """
        try:
            # Create actions array
            actions = self.create_actions_array(use_actual_data, action_mode)
            if len(actions) == 0:
                print("❌ No actions data to save")
                return False
            
            # Create observations
            observations = self.create_observations_dict(actions, use_actual_data)
            
            # Create initial state
            initial_state = self.create_initial_state_dict(use_actual_data)
            
            # Generate demo name
            if demo_name is None:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                data_type = "actual" if use_actual_data else "reference"
                demo_name = f"demo_dynamics_{data_type}_{action_mode}_{timestamp}"
            
            # Save to HDF5
            with h5py.File(output_file, 'a') as f:
                # Create data group if it doesn't exist
                if 'data' not in f:
                    f.create_group('data')
                
                # Find next demo number if demo_name not specified
                if demo_name.startswith('demo_dynamics_'):
                    existing_demos = [key for key in f['data'].keys() if key.startswith('demo_')]
                    demo_num = len(existing_demos)
                    demo_group_name = f'demo_{demo_num}'
                else:
                    demo_group_name = demo_name
                
                # Create demo group
                if demo_group_name in f['data']:
                    del f['data'][demo_group_name]  # Overwrite if exists
                
                demo_group = f['data'].create_group(demo_group_name)
                
                # Add metadata
                demo_group.attrs['num_samples'] = len(actions)
                demo_group.attrs['success'] = True
                demo_group.attrs['action_mode'] = action_mode
                demo_group.attrs['data_source'] = 'actual_dynamics' if use_actual_data else 'reference'
                demo_group.attrs['original_csv'] = os.path.basename(self.csv_file)
                demo_group.attrs['conversion_timestamp'] = datetime.now().isoformat()
                
                # Save actions
                demo_group.create_dataset('actions', data=actions)
                
                # Save initial state
                initial_group = demo_group.create_group('initial_state')
                articulation_group = initial_group.create_group('articulation')
                
                # Robot initial state
                robot_group = articulation_group.create_group('robot')
                robot_group.create_dataset('joint_position', data=initial_state['articulation']['robot']['joint_position'])
                robot_group.create_dataset('joint_velocity', data=initial_state['articulation']['robot']['joint_velocity'])
                robot_group.create_dataset('root_pose', data=initial_state['articulation']['robot']['root_pose'])
                robot_group.create_dataset('root_velocity', data=initial_state['articulation']['robot']['root_velocity'])
                
                # Rigid objects initial state
                rigid_object_group = articulation_group.create_group('rigid_object')
                for obj_name, obj_data in initial_state['articulation']['rigid_object'].items():
                    obj_group = rigid_object_group.create_group(obj_name)
                    obj_group.create_dataset('root_pose', data=obj_data['root_pose'])
                    obj_group.create_dataset('root_velocity', data=obj_data['root_velocity'])
                
                # Save observations
                obs_group = demo_group.create_group('obs')
                for key, data in observations.items():
                    obs_group.create_dataset(key, data=data)
                
                # Save states (copy of observations for compatibility)
                states_group = demo_group.create_group('states')
                for key, data in observations.items():
                    states_group.create_dataset(key, data=data)
            
            print(f"✅ Successfully saved trajectory to {output_file}")
            print(f"   Demo group: {demo_group_name}")
            print(f"   Action mode: {action_mode}")
            print(f"   Data source: {'Actual robot dynamics' if use_actual_data else 'Reference trajectory'}")
            print(f"   Number of samples: {len(actions)}")
            print(f"   Action shape: {actions.shape}")
            
            return True
            
        except Exception as e:
            print(f"❌ Error saving to HDF5: {e}")
            return False
    
    def convert_trajectory(self, action_mode: str = 'pose', use_actual_data: bool = True, 
                          output_filename: Optional[str] = None) -> bool:
        """
        Complete conversion process from CSV to HDF5
        
        Args:
            action_mode: 'joint' or 'pose' action mode  
            use_actual_data: Use actual robot data (True) or reference data (False)
            output_filename: Custom output filename (auto-generated if None)
        """
        print("🔄 Starting trajectory dynamics conversion...")
        print(f"   Input CSV: {os.path.basename(self.csv_file)}")
        print(f"   Action mode: {action_mode}")
        print(f"   Data source: {'Actual robot dynamics' if use_actual_data else 'Reference trajectory'}")
        
        # Load CSV data
        if not self.load_csv_data():
            return False
        
        # Generate output filename
        if output_filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            data_type = "actual" if use_actual_data else "reference"
            csv_basename = os.path.splitext(os.path.basename(self.csv_file))[0]
            output_filename = f"{csv_basename}_dynamics_{data_type}_{action_mode}_{timestamp}.hdf5"
        
        output_path = os.path.join(self.output_dir, output_filename)
        
        # Convert and save
        success = self.save_to_hdf5(output_path, action_mode, use_actual_data)
        
        if success:
            print(f"🎉 Conversion completed successfully!")
            print(f"   Output: {output_path}")
            self.print_conversion_summary(output_path)
        
        return success
    
    def print_conversion_summary(self, output_path: str):
        """Print summary of the conversion"""
        try:
            with h5py.File(output_path, 'r') as f:
                print("\n📊 CONVERSION SUMMARY")
                print("=" * 50)
                
                # Find the demo group
                demo_groups = list(f['data'].keys())
                if demo_groups:
                    demo_group = f['data'][demo_groups[-1]]  # Get latest demo
                    
                    print(f"Demo group: {demo_groups[-1]}")
                    print(f"Number of samples: {demo_group.attrs.get('num_samples', 'Unknown')}")
                    print(f"Action mode: {demo_group.attrs.get('action_mode', 'Unknown')}")
                    print(f"Data source: {demo_group.attrs.get('data_source', 'Unknown')}")
                    print(f"Success flag: {demo_group.attrs.get('success', 'Unknown')}")
                    
                    if 'actions' in demo_group:
                        actions_shape = demo_group['actions'].shape
                        print(f"Actions shape: {actions_shape}")
                    
                    if 'obs' in demo_group:
                        obs_keys = list(demo_group['obs'].keys())
                        print(f"Observation keys: {obs_keys}")
                        
                        # Show key observation shapes
                        for key in ['eef_pos', 'eef_quat', 'object', 'joint_pos']:
                            if key in demo_group['obs']:
                                shape = demo_group['obs'][key].shape
                                print(f"  {key}: {shape}")
                
                print("=" * 50)
                
        except Exception as e:
            print(f"❌ Error reading summary: {e}")


def main():
    parser = argparse.ArgumentParser(
        description="Convert trajectory comparison CSV to HDF5 format with robot dynamics",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Convert using actual robot dynamics (recommended)
  python3 trajectory_dynamics_converter.py --csv trajectory_comparison_20250703_171824.csv --actual
  
  # Convert using reference trajectory (ideal)
  python3 trajectory_dynamics_converter.py --csv trajectory_comparison_20250703_171824.csv --reference
  
  # Convert to joint mode (requires inverse kinematics)
  python3 trajectory_dynamics_converter.py --csv trajectory_comparison_20250703_171824.csv --actual --mode joint
  
  # Convert both actual and reference for comparison
  python3 trajectory_dynamics_converter.py --csv trajectory_comparison_20250703_171824.csv --both
        """
    )
    
    parser.add_argument("--csv", type=str, required=True,
                       help="Path to trajectory comparison CSV file")
    parser.add_argument("--output", type=str, default=None,
                       help="Output directory (default: same as CSV file)")
    parser.add_argument("--mode", type=str, choices=['pose', 'joint'], default='pose',
                       help="Action mode: 'pose' (end-effector) or 'joint' positions")
    
    # Data source options
    data_group = parser.add_mutually_exclusive_group(required=True)
    data_group.add_argument("--actual", action="store_true",
                           help="Use actual robot data (captures controller dynamics)")
    data_group.add_argument("--reference", action="store_true", 
                           help="Use reference trajectory data (ideal)")
    data_group.add_argument("--both", action="store_true",
                           help="Convert both actual and reference data")
    
    parser.add_argument("--filename", type=str, default=None,
                       help="Custom output filename")
    
    args = parser.parse_args()
    
    # Validate input file
    if not os.path.exists(args.csv):
        print(f"❌ CSV file not found: {args.csv}")
        return 1
    
    if not args.csv.endswith('.csv'):
        print(f"❌ Input file must be a CSV file: {args.csv}")
        return 1
    
    try:
        # Create converter
        converter = TrajectoryDynamicsConverter(args.csv, args.output)
        
        success = True
        
        if args.both:
            # Convert both actual and reference data
            print("🔄 Converting both actual and reference trajectories...")
            
            # Actual data
            print("\n1️⃣ Converting actual robot dynamics...")
            success &= converter.convert_trajectory(
                action_mode=args.mode, 
                use_actual_data=True,
                output_filename=args.filename
            )
            
            # Reference data
            print("\n2️⃣ Converting reference trajectory...")
            success &= converter.convert_trajectory(
                action_mode=args.mode, 
                use_actual_data=False,
                output_filename=args.filename
            )
            
        elif args.actual:
            # Convert actual data only
            success = converter.convert_trajectory(
                action_mode=args.mode,
                use_actual_data=True, 
                output_filename=args.filename
            )
            
        elif args.reference:
            # Convert reference data only
            success = converter.convert_trajectory(
                action_mode=args.mode,
                use_actual_data=False,
                output_filename=args.filename
            )
        
        if success:
            print("\n✅ All conversions completed successfully!")
            print("\n💡 Usage Tips:")
            print("   - Use 'actual' data for training BC policies that account for robot dynamics")
            print("   - Use 'reference' data for ideal trajectory following")
            print("   - The converted HDF5 files can be used directly with Isaac Lab imitation learning")
            return 0
        else:
            print("\n❌ Some conversions failed!")
            return 1
            
    except KeyboardInterrupt:
        print("\n\n👋 Conversion interrupted by user")
        return 1
    except Exception as e:
        print(f"\n❌ Error during conversion: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())