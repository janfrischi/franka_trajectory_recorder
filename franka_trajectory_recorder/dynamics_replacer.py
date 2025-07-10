#!/usr/bin/env python3
"""
Replace HDF5 Actions with Real Robot Dynamics
This script replaces the actions in an HDF5 dataset with real robot dynamics captured
from CSV recordings, creating a more realistic dataset for imitation learning.
"""

import os
import sys
import argparse
import h5py
import pandas as pd
import numpy as np
import glob
from typing import Dict, List, Optional, Tuple
from datetime import datetime
import shutil
import json

class DynamicsReplacer:
    """Replaces HDF5 actions with real robot dynamics from CSV recordings"""
    
    def __init__(self, hdf5_file: str, csv_dir: str, output_file: str = None):
        self.hdf5_file = hdf5_file
        self.csv_dir = csv_dir
        self.output_file = output_file or self._generate_output_filename()
        
        # Mapping from demo names to CSV files
        self.demo_csv_mapping = {}
        
        # Statistics for reporting
        self.replacement_stats = {}
        
    def _generate_output_filename(self) -> str:
        """Generate output filename with timestamp"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_name = os.path.splitext(os.path.basename(self.hdf5_file))[0]
        return f"{base_name}_real_dynamics_{timestamp}.hdf5"
    
    def discover_csv_files(self) -> bool:
        """Discover and map CSV files to demo names"""
        print("🔍 Discovering CSV recording files...")
        
        # Pattern to match recording CSV files
        csv_pattern = os.path.join(self.csv_dir, "**/demo_*_recording_*.csv")
        csv_files = glob.glob(csv_pattern, recursive=True)
        
        if not csv_files:
            # Try alternative patterns
            patterns = [
                os.path.join(self.csv_dir, "demo_*_recording_*.csv"),
                os.path.join(self.csv_dir, "**/demo_*_comparison_*.csv"),
                os.path.join(self.csv_dir, "**/*recording*.csv")
            ]
            
            for pattern in patterns:
                csv_files.extend(glob.glob(pattern, recursive=True))
        
        if not csv_files:
            print(f"❌ No CSV recording files found in {self.csv_dir}")
            print("Expected patterns:")
            print("  - demo_*_recording_*.csv")
            print("  - demo_*_comparison_*.csv")
            return False
        
        print(f"✅ Found {len(csv_files)} CSV files")
        
        # Extract demo names from CSV filenames
        for csv_file in csv_files:
            demo_name = self._extract_demo_name_from_csv(csv_file)
            if demo_name:
                self.demo_csv_mapping[demo_name] = csv_file
                print(f"  📄 {demo_name} -> {os.path.basename(csv_file)}")
        
        print(f"📊 Mapped {len(self.demo_csv_mapping)} demos to CSV files")
        return len(self.demo_csv_mapping) > 0
    
    def _extract_demo_name_from_csv(self, csv_file: str) -> Optional[str]:
        """Extract demo name from CSV filename"""
        filename = os.path.basename(csv_file)
        
        # Pattern: demo_X_recording_HHMMSS.csv or demo_X_comparison_HHMMSS.csv
        if 'demo_' in filename:
            parts = filename.split('_')
            for i, part in enumerate(parts):
                if part == 'demo' and i + 1 < len(parts):
                    try:
                        demo_num = int(parts[i + 1])
                        return f"demo_{demo_num}"
                    except ValueError:
                        continue
        
        return None
    
    def validate_hdf5_structure(self) -> bool:
        """Validate HDF5 file structure"""
        print("🔍 Validating HDF5 file structure...")
        
        try:
            with h5py.File(self.hdf5_file, 'r') as f:
                if 'data' not in f:
                    print("❌ No 'data' group found in HDF5 file")
                    return False
                
                data_group = f['data']
                demos = [key for key in data_group.keys() if key.startswith('demo_')]
                
                if not demos:
                    print("❌ No demo groups found in HDF5 file")
                    return False
                
                print(f"✅ Found {len(demos)} demos in HDF5: {demos}")
                
                # Check first demo structure
                demo_name = demos[0]
                demo_group = data_group[demo_name]
                
                required_datasets = ['actions', 'obs']
                for dataset in required_datasets:
                    if dataset not in demo_group:
                        print(f"❌ Required dataset '{dataset}' not found in {demo_name}")
                        return False
                
                # Check action dimensions
                actions = demo_group['actions']
                print(f"📊 Action shape in {demo_name}: {actions.shape}")
                print(f"📊 Action format: {actions.shape[1]}D (expected: 7 or 8)")
                
                return True
                
        except Exception as e:
            print(f"❌ Error validating HDF5 file: {e}")
            return False
    
    def load_csv_dynamics(self, csv_file: str) -> Optional[Tuple[np.ndarray, Dict]]:
        """Load robot dynamics from CSV file"""
        try:
            df = pd.read_csv(csv_file)
            
            # Check required columns
            required_pose_cols = [
                'actual_eef_pos_x', 'actual_eef_pos_y', 'actual_eef_pos_z',
                'actual_eef_quat_x', 'actual_eef_quat_y', 'actual_eef_quat_z', 'actual_eef_quat_w'
            ]
            
            missing_cols = [col for col in required_pose_cols if col not in df.columns]
            if missing_cols:
                print(f"❌ Missing required columns in {csv_file}: {missing_cols}")
                return None
            
            # Extract pose data: [x, y, z, qx, qy, qz, qw]
            poses = df[required_pose_cols].values
            
            # Extract gripper data if available
            gripper_data = None
            if 'actual_gripper_width' in df.columns:
                # Convert gripper width to binary command (0 = closed, 1 = open)
                gripper_widths = df['actual_gripper_width'].values
                # Threshold: if width > 0.04m (40mm), consider it open
                gripper_data = (gripper_widths > 0.04).astype(float)
            else:
                # Default to open gripper
                gripper_data = np.ones(len(poses))
            
            # Combine pose and gripper: [x, y, z, qx, qy, qz, qw, gripper]
            actions = np.column_stack([poses, gripper_data])
            
            # Create metadata
            metadata = {
                'source_csv': os.path.basename(csv_file),
                'num_samples': len(actions),
                'duration': df['playback_time'].iloc[-1] if 'playback_time' in df.columns else 0.0,
                'frequency': len(actions) / df['playback_time'].iloc[-1] if 'playback_time' in df.columns and df['playback_time'].iloc[-1] > 0 else 0.0
            }
            
            return actions, metadata
            
        except Exception as e:
            print(f"❌ Error loading CSV {csv_file}: {e}")
            return None
    
    def replace_demo_actions(self, demo_name: str, new_actions: np.ndarray, 
                           hdf5_input: h5py.Group, hdf5_output: h5py.Group) -> bool:
        """Replace actions for a single demo"""
        try:
            demo_input = hdf5_input[demo_name]
            
            # Create demo group in output
            demo_output = hdf5_output.create_group(demo_name)
            
            # Copy all datasets except actions
            for key in demo_input.keys():
                if key == 'actions':
                    continue  # We'll replace this
                elif isinstance(demo_input[key], h5py.Dataset):
                    # Copy dataset
                    demo_output.create_dataset(key, data=demo_input[key][:])
                elif isinstance(demo_input[key], h5py.Group):
                    # Recursively copy group
                    self._copy_group(demo_input[key], demo_output.create_group(key), new_actions.shape[0])
            
            # Copy attributes
            for attr_name, attr_value in demo_input.attrs.items():
                demo_output.attrs[attr_name] = attr_value
            
            # Replace actions
            demo_output.create_dataset('actions', data=new_actions.astype(np.float32))
            
            # Update num_samples attribute
            demo_output.attrs['num_samples'] = new_actions.shape[0]
            
            # Add metadata about replacement
            demo_output.attrs['real_dynamics_source'] = f"Replaced with real robot dynamics"
            demo_output.attrs['original_samples'] = demo_input.attrs.get('num_samples', 0)
            demo_output.attrs['new_samples'] = new_actions.shape[0]
            
            return True
            
        except Exception as e:
            print(f"❌ Error replacing actions for {demo_name}: {e}")
            return False
    
    def _copy_group(self, input_group: h5py.Group, output_group: h5py.Group, new_num_samples: int):
        """Recursively copy HDF5 group, adjusting observation arrays if needed"""
        for key in input_group.keys():
            if isinstance(input_group[key], h5py.Dataset):
                data = input_group[key][:]
                
                # If this is an observation array that needs to be adjusted
                if hasattr(data, 'shape') and len(data.shape) > 0 and data.shape[0] != new_num_samples:
                    if key in ['eef_pos', 'eef_quat', 'gripper_pos', 'joint_pos', 'joint_vel']:
                        # Interpolate or truncate observation data to match new length
                        data = self._adjust_observation_length(data, new_num_samples)
                
                output_group.create_dataset(key, data=data)
            elif isinstance(input_group[key], h5py.Group):
                self._copy_group(input_group[key], output_group.create_group(key), 
                               new_num_samples)
        
        # Copy attributes
        for attr_name, attr_value in input_group.attrs.items():
            output_group.attrs[attr_name] = attr_value
    
    def _adjust_observation_length(self, data: np.ndarray, new_length: int) -> np.ndarray:
        """Adjust observation array length by interpolation or truncation"""
        if len(data) == new_length:
            return data
        elif len(data) > new_length:
            # Truncate: take evenly spaced samples
            indices = np.linspace(0, len(data) - 1, new_length, dtype=int)
            return data[indices]
        else:
            # Interpolate: expand to new length
            old_indices = np.arange(len(data))
            new_indices = np.linspace(0, len(data) - 1, new_length)
            
            if len(data.shape) == 1:
                return np.interp(new_indices, old_indices, data)
            else:
                # Multi-dimensional data
                result = np.zeros((new_length,) + data.shape[1:], dtype=data.dtype)
                for i in range(data.shape[1]):
                    if len(data.shape) == 2:
                        result[:, i] = np.interp(new_indices, old_indices, data[:, i])
                    # Add more dimensions if needed
                return result
    
    def process_all_demos(self) -> bool:
        """Process all demos and replace actions with real dynamics"""
        print("🔄 Starting action replacement process...")
        
        # Create backup of original file
        backup_file = f"{self.hdf5_file}.backup_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        shutil.copy2(self.hdf5_file, backup_file)
        print(f"💾 Created backup: {backup_file}")
        
        try:
            with h5py.File(self.hdf5_file, 'r') as f_input:
                with h5py.File(self.output_file, 'w') as f_output:
                    
                    # Copy top-level attributes
                    for attr_name, attr_value in f_input.attrs.items():
                        f_output.attrs[attr_name] = attr_value
                    
                    # Create data group
                    data_input = f_input['data']
                    data_output = f_output.create_group('data')
                    
                    # Copy data group attributes
                    for attr_name, attr_value in data_input.attrs.items():
                        data_output.attrs[attr_name] = attr_value
                    
                    # Process each demo
                    demos = [key for key in data_input.keys() if key.startswith('demo_')]
                    processed_count = 0
                    
                    for demo_name in sorted(demos):
                        print(f"\n📝 Processing {demo_name}...")
                        
                        if demo_name in self.demo_csv_mapping:
                            # Load real dynamics from CSV
                            csv_file = self.demo_csv_mapping[demo_name]
                            result = self.load_csv_dynamics(csv_file)
                            
                            if result is not None:
                                new_actions, metadata = result
                                
                                # Replace actions
                                if self.replace_demo_actions(demo_name, new_actions, data_input, data_output):
                                    print(f"✅ Replaced actions for {demo_name}")
                                    print(f"   📊 Original samples: {data_input[demo_name].attrs.get('num_samples', 'unknown')}")
                                    print(f"   📊 New samples: {new_actions.shape[0]}")
                                    print(f"   📊 Action shape: {new_actions.shape}")
                                    
                                    self.replacement_stats[demo_name] = {
                                        'status': 'replaced',
                                        'csv_source': csv_file,
                                        'new_samples': new_actions.shape[0],
                                        'metadata': metadata
                                    }
                                    processed_count += 1
                                else:
                                    print(f"❌ Failed to replace actions for {demo_name}")
                                    self.replacement_stats[demo_name] = {'status': 'failed', 'reason': 'replacement_error'}
                            else:
                                print(f"❌ Failed to load CSV data for {demo_name}")
                                self.replacement_stats[demo_name] = {'status': 'failed', 'reason': 'csv_load_error'}
                        else:
                            print(f"⚠️ No CSV recording found for {demo_name}, keeping original actions")
                            # Copy original demo without changes
                            self._copy_demo_unchanged(data_input[demo_name], data_output.create_group(demo_name))
                            self.replacement_stats[demo_name] = {'status': 'unchanged', 'reason': 'no_csv'}
                    
                    print(f"\n🎉 Processed {processed_count}/{len(demos)} demos with real dynamics")
                    
                    # Add processing metadata
                    data_output.attrs['real_dynamics_processing'] = f"Processed on {datetime.now().isoformat()}"
                    data_output.attrs['processed_demos'] = processed_count
                    data_output.attrs['total_demos'] = len(demos)
            
            return True
            
        except Exception as e:
            print(f"❌ Error during processing: {e}")
            return False
    
    def _copy_demo_unchanged(self, input_demo: h5py.Group, output_demo: h5py.Group):
        """Copy demo without any changes"""
        for key in input_demo.keys():
            if isinstance(input_demo[key], h5py.Dataset):
                output_demo.create_dataset(key, data=input_demo[key][:])
            elif isinstance(input_demo[key], h5py.Group):
                self._copy_group(input_demo[key], output_demo.create_group(key), 
                               input_demo.attrs.get('num_samples', 0))
        
        # Copy attributes
        for attr_name, attr_value in input_demo.attrs.items():
            output_demo.attrs[attr_name] = attr_value
    
    def generate_report(self) -> str:
        """Generate a detailed processing report"""
        report = []
        report.append("="*80)
        report.append("REAL DYNAMICS REPLACEMENT REPORT")
        report.append("="*80)
        report.append(f"Input HDF5: {self.hdf5_file}")
        report.append(f"Output HDF5: {self.output_file}")
        report.append(f"CSV Directory: {self.csv_dir}")
        report.append(f"Processing Time: {datetime.now().isoformat()}")
        report.append("")
        
        # Statistics
        total_demos = len(self.replacement_stats)
        replaced_demos = sum(1 for stats in self.replacement_stats.values() if stats['status'] == 'replaced')
        failed_demos = sum(1 for stats in self.replacement_stats.values() if stats['status'] == 'failed')
        unchanged_demos = sum(1 for stats in self.replacement_stats.values() if stats['status'] == 'unchanged')
        
        report.append("📊 SUMMARY STATISTICS:")
        report.append(f"  Total demos: {total_demos}")
        report.append(f"  Successfully replaced: {replaced_demos}")
        report.append(f"  Failed: {failed_demos}")
        report.append(f"  Unchanged (no CSV): {unchanged_demos}")
        report.append(f"  Success rate: {replaced_demos/total_demos*100:.1f}%")
        report.append("")
        
        # Detailed results
        report.append("📝 DETAILED RESULTS:")
        for demo_name, stats in sorted(self.replacement_stats.items()):
            status_icon = "✅" if stats['status'] == 'replaced' else "❌" if stats['status'] == 'failed' else "⚠️"
            report.append(f"  {status_icon} {demo_name}: {stats['status']}")
            
            if stats['status'] == 'replaced':
                metadata = stats.get('metadata', {})
                report.append(f"      📄 Source: {os.path.basename(stats['csv_source'])}")
                report.append(f"      📊 Samples: {stats['new_samples']}")
                report.append(f"      ⏱️ Duration: {metadata.get('duration', 'unknown'):.2f}s")
                report.append(f"      📈 Frequency: {metadata.get('frequency', 'unknown'):.1f}Hz")
            elif stats['status'] == 'failed':
                report.append(f"      ❌ Reason: {stats.get('reason', 'unknown')}")
        
        report.append("")
        report.append("🎯 RECOMMENDED NEXT STEPS:")
        if replaced_demos > 0:
            report.append("  ✅ Dataset successfully updated with real robot dynamics")
            report.append("  🚀 Ready for imitation learning training")
            report.append(f"  📁 New dataset: {self.output_file}")
        
        if failed_demos > 0:
            report.append(f"  ⚠️ {failed_demos} demos failed - check CSV file format and content")
        
        if unchanged_demos > 0:
            report.append(f"  📝 {unchanged_demos} demos unchanged - record more CSV dynamics if needed")
        
        report.append("="*80)
        
        return "\n".join(report)
    
    def save_report(self, report_content: str):
        """Save report to file"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_file = f"dynamics_replacement_report_{timestamp}.txt"
        
        with open(report_file, 'w') as f:
            f.write(report_content)
        
        print(f"📄 Report saved to: {report_file}")


def main():
    parser = argparse.ArgumentParser(
        description="Replace HDF5 actions with real robot dynamics from CSV recordings",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic usage
  python3 replace_actions_with_dynamics.py --hdf5 dataset_25_06.hdf5 --csv-dir ./recordings/

  # Specify output file
  python3 replace_actions_with_dynamics.py --hdf5 dataset_25_06.hdf5 --csv-dir ./recordings/ --output dataset_real_dynamics.hdf5

  # Dry run to check mapping
  python3 replace_actions_with_dynamics.py --hdf5 dataset_25_06.hdf5 --csv-dir ./recordings/ --dry-run
        """
    )
    
    parser.add_argument("--hdf5", type=str, required=True,
                       help="Path to input HDF5 dataset file")
    parser.add_argument("--csv-dir", type=str, required=True,
                       help="Directory containing CSV recording files")
    parser.add_argument("--output", type=str, default=None,
                       help="Output HDF5 file (auto-generated if not specified)")
    parser.add_argument("--dry-run", action="store_true",
                       help="Perform dry run to check file mapping without processing")
    parser.add_argument("--backup", action="store_true", default=True,
                       help="Create backup of original file (default: True)")
    
    args = parser.parse_args()
    
    # Validate inputs
    if not os.path.exists(args.hdf5):
        print(f"❌ HDF5 file not found: {args.hdf5}")
        return 1
    
    if not os.path.exists(args.csv_dir):
        print(f"❌ CSV directory not found: {args.csv_dir}")
        return 1
    
    try:
        print("🤖 Real Robot Dynamics Replacement Tool")
        print("="*50)
        
        # Create replacer
        replacer = DynamicsReplacer(args.hdf5, args.csv_dir, args.output)
        
        # Discover CSV files
        if not replacer.discover_csv_files():
            return 1
        
        # Validate HDF5 structure
        if not replacer.validate_hdf5_structure():
            return 1
        
        if args.dry_run:
            print("\n🔍 DRY RUN - File mapping check:")
            print("CSV files found:")
            for demo_name, csv_file in replacer.demo_csv_mapping.items():
                print(f"  ✅ {demo_name} -> {os.path.basename(csv_file)}")
            
            print("\nDry run completed. Use without --dry-run to perform actual replacement.")
            return 0
        
        # Process all demos
        if replacer.process_all_demos():
            # Generate and display report
            report = replacer.generate_report()
            print("\n" + report)
            
            # Save report
            replacer.save_report(report)
            
            print(f"\n🎉 Successfully created dataset with real robot dynamics!")
            print(f"📁 Output file: {replacer.output_file}")
            return 0
        else:
            print("\n❌ Processing failed!")
            return 1
            
    except KeyboardInterrupt:
        print("\n\n👋 Processing interrupted by user")
        return 1
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())