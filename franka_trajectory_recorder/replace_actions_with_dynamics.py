#!/usr/bin/env python3
"""
Replace actions in an HDF5 dataset "Expert Trajectories" with real robot actions from a CSV file "Expert Trajectory Playback capturing real dynamics".
Usage: python3 replace_actions_with_dynamics.py --hdf5 dataset_25_06.hdf5 --csv all_demos_recording.csv --output dataset_real_dynamics.hdf5
"""

import argparse
import h5py
import pandas as pd
import numpy as np
import os

def load_actions_by_demo_from_csv(csv_file, action_mode='pose'):
    # Read CSV file and initialize a dictionary to hold actions grouped by demo_name
    df = pd.read_csv(csv_file)
    demos = {}
    # Group together all rows with the same demo_name
    for demo_name, group in df.groupby('demo_name'):
        if action_mode == 'pose':
            # ROS2: [actual_eef_quat_x, actual_eef_quat_y, actual_eef_quat_z, actual_eef_quat_w]
            # IsaacLab expects: [qw, qx, qy, qz]
            pos = group[['actual_eef_pos_x', 'actual_eef_pos_y', 'actual_eef_pos_z']].to_numpy(dtype=np.float32)
            quat_x = group['actual_eef_quat_x'].to_numpy(dtype=np.float32)
            quat_y = group['actual_eef_quat_y'].to_numpy(dtype=np.float32)
            quat_z = group['actual_eef_quat_z'].to_numpy(dtype=np.float32)
            quat_w = group['actual_eef_quat_w'].to_numpy(dtype=np.float32)
            # Rearrange quaternion to [qw, qx, qy, qz]
            quat = np.stack([quat_w, quat_x, quat_y, quat_z], axis=1)
            # IsaacLab convention: gripper > 0.02 -> 1 (open), else -1 (closed)
            gripper_raw = group['actual_gripper_width'].to_numpy(dtype=np.float32)
            gripper = np.where(gripper_raw > 0.02, 1.0, -1.0).reshape(-1, 1)
            actions = np.concatenate([pos, quat, gripper], axis=1)
        elif action_mode == 'joint':
            # IsaacLab convention: gripper > 0.02 -> 1 (open), else -1 (closed)
            gripper_raw = group['actual_gripper_width'].to_numpy(dtype=np.float32)
            gripper = np.where(gripper_raw > 0.02, 1.0, -1.0).reshape(-1, 1)
            actions = group[[
                'actual_joint_pos_1', 'actual_joint_pos_2', 'actual_joint_pos_3',
                'actual_joint_pos_4', 'actual_joint_pos_5', 'actual_joint_pos_6', 'actual_joint_pos_7'
            ]].to_numpy(dtype=np.float32)
            actions = np.concatenate([actions, gripper], axis=1)
        else:
            raise ValueError("Unknown action_mode: {}".format(action_mode))
        # Store actions in the dictionary under the demo_name
        demos[demo_name] = actions

    return demos

def replace_all_demos_in_hdf5(hdf5_in, hdf5_out, csv_file, action_mode='pose'):
    """
    Replace actions in an HDF5 dataset with real robot actions from a CSV file.
    """
    # Load all actions grouped by demo_name from CSV
    actions_by_demo = load_actions_by_demo_from_csv(csv_file, action_mode=action_mode)
    with h5py.File(hdf5_in, 'r') as fin, h5py.File(hdf5_out, 'w') as fout:
        fin.copy('data', fout)
        demo_names = list(fout['data'].keys())
        replaced, skipped = 0, 0
        for demo_name in demo_names:
            if demo_name not in actions_by_demo:
                print(f"⚠️  No actions found for {demo_name} in CSV, skipping.")
                skipped += 1
                continue
            actions = actions_by_demo[demo_name]
            demo_group = fout['data'][demo_name]
            if 'actions' in demo_group:
                del demo_group['actions']
            demo_group.create_dataset('actions', data=actions, dtype=np.float32)
            if 'obs' in demo_group and 'actions' in demo_group['obs']:
                del demo_group['obs']['actions']
                demo_group['obs'].create_dataset('actions', data=actions, dtype=np.float32)
            demo_group.attrs['num_samples'] = actions.shape[0]
            print(f"✅ Replaced actions for {demo_name} ({actions.shape[0]} steps)")
            replaced += 1
        print(f"\nSummary: {replaced} demos replaced, {skipped} demos skipped (no CSV data)")

def main():
    parser = argparse.ArgumentParser(description="Replace HDF5 actions with real robot actions from CSV (multi-demo)")
    parser.add_argument('--hdf5', type=str, required=True, help="Input HDF5 file (expert/reference)")
    parser.add_argument('--csv', type=str, required=True, help="CSV file with real robot actions (multiple demos)")
    parser.add_argument('--output', type=str, required=True, help="Output HDF5 file")
    parser.add_argument('--mode', type=str, default='pose', choices=['pose', 'joint'], help="Action mode")
    args = parser.parse_args()

    if not os.path.exists(args.hdf5):
        print(f"❌ HDF5 file not found: {args.hdf5}")
        return
    if not os.path.exists(args.csv):
        print(f"❌ CSV file not found: {args.csv}")
        return

    replace_all_demos_in_hdf5(args.hdf5, args.output, args.csv, action_mode=args.mode)

if __name__ == "__main__":
    main()