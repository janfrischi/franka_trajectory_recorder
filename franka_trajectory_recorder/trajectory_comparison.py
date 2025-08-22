#!/usr/bin/env python3
"""
Trajectory Comparison Visualization
Compares reference vs actual end-effector trajectories from CSV data using Plotly.
"""

import pandas as pd
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import plotly.express as px
import argparse
import os
import sys
from typing import Optional, Dict, Tuple
from scipy.spatial.transform import Rotation as R


class TrajectoryComparison:
    def __init__(self, csv_file: str):
        self.csv_file = csv_file
        self.data = None
        
    def load_data(self) -> bool:
        """Load trajectory comparison data from CSV file"""
        try:
            if not os.path.exists(self.csv_file):
                print(f"❌ CSV file not found: {self.csv_file}")
                return False
                
            self.data = pd.read_csv(self.csv_file)
            print(f"✅ Loaded {len(self.data)} data points from {os.path.basename(self.csv_file)}")
            
            # Validate required columns
            required_cols = [
                'timestamp', 'playback_time', 'step_index',
                'ref_eef_pos_x', 'ref_eef_pos_y', 'ref_eef_pos_z',
                'ref_eef_quat_x', 'ref_eef_quat_y', 'ref_eef_quat_z', 'ref_eef_quat_w',
                'actual_eef_pos_x', 'actual_eef_pos_y', 'actual_eef_pos_z',
                'actual_eef_quat_x', 'actual_eef_quat_y', 'actual_eef_quat_z', 'actual_eef_quat_w',
                'actual_gripper_width'
            ]
            
            missing_cols = [col for col in required_cols if col not in self.data.columns]
            if missing_cols:
                print(f"❌ Missing required columns: {missing_cols}")
                return False
                
            print(f"📊 Data time range: {self.data['playback_time'].min():.2f} to {self.data['playback_time'].max():.2f} seconds")
            return True
            
        except Exception as e:
            print(f"❌ Error loading CSV file: {e}")
            return False
    
    def calculate_additional_metrics(self):
        """Calculate additional error metrics and trajectory properties"""
        if self.data is None:
            return
            
        # Position errors (if not already calculated)
        if 'pos_error_norm' not in self.data.columns:
            self.data['pos_error_x'] = self.data['actual_eef_pos_x'] - self.data['ref_eef_pos_x']
            self.data['pos_error_y'] = self.data['actual_eef_pos_y'] - self.data['ref_eef_pos_y']
            self.data['pos_error_z'] = self.data['actual_eef_pos_z'] - self.data['ref_eef_pos_z']
            self.data['pos_error_norm'] = np.sqrt(
                self.data['pos_error_x']**2 + 
                self.data['pos_error_y']**2 + 
                self.data['pos_error_z']**2
            )
        
        # Quaternion errors (if not already calculated)
        if 'orientation_error_angle' not in self.data.columns:
            self.data['quat_error_x'] = self.data['actual_eef_quat_x'] - self.data['ref_eef_quat_x']
            self.data['quat_error_y'] = self.data['actual_eef_quat_y'] - self.data['ref_eef_quat_y']
            self.data['quat_error_z'] = self.data['actual_eef_quat_z'] - self.data['ref_eef_quat_z']
            self.data['quat_error_w'] = self.data['actual_eef_quat_w'] - self.data['ref_eef_quat_w']
            
            # Calculate orientation error angle
            orientation_errors = []
            for idx, row in self.data.iterrows():
                ref_quat = [row['ref_eef_quat_x'], row['ref_eef_quat_y'], 
                           row['ref_eef_quat_z'], row['ref_eef_quat_w']]
                actual_quat = [row['actual_eef_quat_x'], row['actual_eef_quat_y'], 
                              row['actual_eef_quat_z'], row['actual_eef_quat_w']]
                
                # Calculate angular error between quaternions
                error_angle = self.quaternion_angular_error(ref_quat, actual_quat)
                orientation_errors.append(error_angle)
            
            self.data['orientation_error_angle'] = orientation_errors
        
        # Velocities (numerical differentiation)
        dt = np.diff(self.data['playback_time'])
        dt = np.append(dt, dt[-1])  # Extend to match array length
        
        # Reference velocities
        ref_vel_x = np.gradient(self.data['ref_eef_pos_x'], self.data['playback_time'])
        ref_vel_y = np.gradient(self.data['ref_eef_pos_y'], self.data['playback_time'])
        ref_vel_z = np.gradient(self.data['ref_eef_pos_z'], self.data['playback_time'])
        self.data['ref_vel_norm'] = np.sqrt(ref_vel_x**2 + ref_vel_y**2 + ref_vel_z**2)
        
        # Actual velocities
        actual_vel_x = np.gradient(self.data['actual_eef_pos_x'], self.data['playback_time'])
        actual_vel_y = np.gradient(self.data['actual_eef_pos_y'], self.data['playback_time'])
        actual_vel_z = np.gradient(self.data['actual_eef_pos_z'], self.data['playback_time'])
        self.data['actual_vel_norm'] = np.sqrt(actual_vel_x**2 + actual_vel_y**2 + actual_vel_z**2)
        
    def quaternion_angular_error(self, q_ref: list, q_actual: list) -> float:
        """Calculate angular error between two quaternions in radians"""
        try:
            # Normalize quaternions
            q_ref = np.array(q_ref)
            q_actual = np.array(q_actual)
            q_ref = q_ref / np.linalg.norm(q_ref)
            q_actual = q_actual / np.linalg.norm(q_actual)
            
            # Calculate dot product (cosine of half the rotation angle)
            dot_product = np.abs(np.dot(q_ref, q_actual))
            dot_product = np.clip(dot_product, 0.0, 1.0)
            
            # Angular error
            error_angle = 2.0 * np.arccos(dot_product)
            return error_angle
            
        except Exception:
            return 0.0
    
    def create_3d_trajectory_comparison(self) -> go.Figure:
        """Create 3D trajectory comparison plot"""
        fig = go.Figure()
        
        # Reference trajectory
        fig.add_trace(go.Scatter3d(
            x=self.data['ref_eef_pos_x'],
            y=self.data['ref_eef_pos_y'],
            z=self.data['ref_eef_pos_z'],
            mode='lines+markers',
            line=dict(color='#2E86C1', width=4),
            marker=dict(size=3, color='#2E86C1'),
            name='Reference Trajectory',
            showlegend=True,
            legendgroup='reference',
            hovertemplate='<b>Reference</b><br>' +
                         'X: %{x:.4f} m<br>' +
                         'Y: %{y:.4f} m<br>' +
                         'Z: %{z:.4f} m<br>' +
                         '<extra></extra>'
        ))
        
        # Actual trajectory
        fig.add_trace(go.Scatter3d(
            x=self.data['actual_eef_pos_x'],
            y=self.data['actual_eef_pos_y'],
            z=self.data['actual_eef_pos_z'],
            mode='lines+markers',
            line=dict(color='#E74C3C', width=4),
            marker=dict(size=3, color='#E74C3C'),
            name='Actual Trajectory',
            showlegend=True,
            legendgroup='actual',
            hovertemplate='<b>Actual</b><br>' +
                         'X: %{x:.4f} m<br>' +
                         'Y: %{y:.4f} m<br>' +
                         'Z: %{z:.4f} m<br>' +
                         '<extra></extra>'
        ))
        
        # Start points
        fig.add_trace(go.Scatter3d(
            x=[self.data['ref_eef_pos_x'].iloc[0]],
            y=[self.data['ref_eef_pos_y'].iloc[0]],
            z=[self.data['ref_eef_pos_z'].iloc[0]],
            mode='markers',
            marker=dict(size=8, color='#27AE60', symbol='diamond'),
            name='Start (Reference)',
            showlegend=True,
            legendgroup='markers'
        ))
        
        fig.add_trace(go.Scatter3d(
            x=[self.data['actual_eef_pos_x'].iloc[0]],
            y=[self.data['actual_eef_pos_y'].iloc[0]],
            z=[self.data['actual_eef_pos_z'].iloc[0]],
            mode='markers',
            marker=dict(size=8, color='#F39C12', symbol='diamond'),
            name='Start (Actual)',
            showlegend=True,
            legendgroup='markers'
        ))
        
        # End points
        fig.add_trace(go.Scatter3d(
            x=[self.data['ref_eef_pos_x'].iloc[-1]],
            y=[self.data['ref_eef_pos_y'].iloc[-1]],
            z=[self.data['ref_eef_pos_z'].iloc[-1]],
            mode='markers',
            marker=dict(size=8, color='#1B4F72', symbol='square'),
            name='End (Reference)',
            showlegend=True,
            legendgroup='markers'
        ))
        
        fig.add_trace(go.Scatter3d(
            x=[self.data['actual_eef_pos_x'].iloc[-1]],
            y=[self.data['actual_eef_pos_y'].iloc[-1]],
            z=[self.data['actual_eef_pos_z'].iloc[-1]],
            mode='markers',
            marker=dict(size=8, color='#922B21', symbol='square'),
            name='End (Actual)',
            showlegend=True,
            legendgroup='markers'
        ))
        
        fig.update_layout(
            title={
                'text': f'<b>3D End-Effector Trajectory Comparison</b><br>', #+
                       #f'<span style="font-size:14px; color:#666;">{os.path.basename(self.csv_file)}</span>',
                'x': 0.5,
                'xanchor': 'center',
                'font': {'size': 20}
            },
            scene=dict(
                xaxis_title='<b>X (m)</b>',
                yaxis_title='<b>Y (m)</b>',
                zaxis_title='<b>Z (m)</b>',
                aspectmode='data',
                camera=dict(eye=dict(x=1.5, y=1.5, z=1.5))
            ),
            height=1000,
            showlegend=True,
            legend=dict(
                orientation="h",
                yanchor="bottom",
                y=-0.3,
                xanchor="center",
                x=0.5,
                bgcolor='rgba(255,255,255,0.8)',
                bordercolor='rgba(0,0,0,0.2)',
                borderwidth=1,
                font=dict(size=14),
                groupclick="toggleitem"
            ),
            font=dict(family="Arial, sans-serif"),
            plot_bgcolor='rgba(0,0,0,0)',
            paper_bgcolor='rgba(0,0,0,0)'
        )
        
        return fig
    
    def create_position_comparison_plot(self) -> go.Figure:
        """Create position vs time comparison plot"""
        fig = make_subplots(
            rows=3, cols=1,
            subplot_titles=('<b>X Position</b>', '<b>Y Position</b>', '<b>Z Position</b>'),
            vertical_spacing=0.10
        )
        
        colors = ['#E74C3C', '#27AE60', '#3498DB']
        axes = ['x', 'y', 'z']
        
        for i, (axis, color) in enumerate(zip(axes, colors)):
            # Reference position
            fig.add_trace(go.Scatter(
                x=self.data['playback_time'],
                y=self.data[f'ref_eef_pos_{axis}'],
                mode='lines',
                name=f'Reference {axis.upper()}',
                line=dict(color=color, width=2),
                showlegend=True,
                legendgroup='reference',
                hovertemplate=f'<b>Reference {axis.upper()}</b><br>' +
                             'Time: %{x:.2f} s<br>' +
                             f'{axis.upper()}: %{{y:.4f}} m<br>' +
                             '<extra></extra>'
            ), row=i+1, col=1)
            
            # Actual position
            fig.add_trace(go.Scatter(
                x=self.data['playback_time'],
                y=self.data[f'actual_eef_pos_{axis}'],
                mode='lines',
                name=f'Actual {axis.upper()}',
                line=dict(color=color, width=2, dash='dash'),
                showlegend=True,
                legendgroup='actual',
                hovertemplate=f'<b>Actual {axis.upper()}</b><br>' +
                             'Time: %{x:.2f} s<br>' +
                             f'{axis.upper()}: %{{y:.4f}} m<br>' +
                             '<extra></extra>'
            ), row=i+1, col=1)

        # Update layout
        fig.update_layout(
            title={
                'text': f'<b>End-Effector Position Comparison vs Time</b><br>', #+
                       #f'<span style="font-size:14px; color:#666;">{os.path.basename(self.csv_file)}</span>',
                'x': 0.5,
                'xanchor': 'center',
                'font': {'size': 20}
            },
            height=700,
            hovermode='x unified',
            showlegend=True,
            legend=dict(
                orientation="h",
                yanchor="bottom",
                y=-0.3,
                xanchor="center",
                x=0.5,
                bgcolor='rgba(255,255,255,0.8)',
                bordercolor='rgba(0,0,0,0.2)',
                borderwidth=1,
                font=dict(size=14),
                groupclick="toggleitem"
            ),
            font=dict(family="Arial, sans-serif"),
            plot_bgcolor='rgba(0,0,0,0)',
            paper_bgcolor='rgba(0,0,0,0)'
        )
        
        # Update subplot titles to be centered
        for i in fig['layout']['annotations']:
            i['font'] = dict(size=14, color='#2C3E50')
            i['xanchor'] = 'center'

        # Update axis labels
        for i in range(1, 4):
            fig.update_yaxes(title_text=f"<b>{axes[i-1].upper()} Position (m)</b>", row=i, col=1, 
                            showgrid=True, gridcolor='rgba(128,128,128,0.2)')
            fig.update_xaxes(showgrid=True, gridcolor='rgba(128,128,128,0.2)', row=i, col=1)

        # Update the bottom subplot's X-axis with time label
        fig.update_xaxes(title_text="<b>Time (s)</b>", row=3, col=1, 
                        showgrid=True, gridcolor='rgba(128,128,128,0.2)')

        return fig
    
    def create_quaternion_comparison_plot(self) -> go.Figure:
        """Create quaternion vs time comparison plot"""
        fig = make_subplots(
            rows=5, cols=1,
            subplot_titles=('<b>Quaternion X</b>', '<b>Quaternion Y</b>', '<b>Quaternion Z</b>', 
                           '<b>Quaternion W</b>', '<b>Orientation Error</b>'),
            vertical_spacing=0.06
        )
        
        colors = ['#E74C3C', '#27AE60', '#3498DB', '#F39C12']
        components = ['x', 'y', 'z', 'w']
        
        for i, (comp, color) in enumerate(zip(components, colors)):
            # Reference quaternion
            fig.add_trace(go.Scatter(
                x=self.data['playback_time'],
                y=self.data[f'ref_eef_quat_{comp}'],
                mode='lines',
                name=f'Reference q{comp}',
                line=dict(color=color, width=2),
                showlegend=True,
                legendgroup='reference',
                hovertemplate=f'<b>Reference q{comp}</b><br>' +
                             'Time: %{x:.2f} s<br>' +
                             f'q{comp}: %{{y:.4f}}<br>' +
                             '<extra></extra>'
            ), row=i+1, col=1)
            
            # Actual quaternion
            fig.add_trace(go.Scatter(
                x=self.data['playback_time'],
                y=self.data[f'actual_eef_quat_{comp}'],
                mode='lines',
                name=f'Actual q{comp}',
                line=dict(color=color, width=2, dash='dash'),
                showlegend=True,
                legendgroup='actual',
                hovertemplate=f'<b>Actual q{comp}</b><br>' +
                             'Time: %{x:.2f} s<br>' +
                             f'q{comp}: %{{y:.4f}}<br>' +
                             '<extra></extra>'
            ), row=i+1, col=1)
    
        # Orientation error
        fig.add_trace(go.Scatter(
            x=self.data['playback_time'],
            y=np.degrees(self.data['orientation_error_angle']),
            mode='lines',
            name='Orientation Error',
            line=dict(color='#8E44AD', width=3),
            fill='tonexty',
            fillcolor='rgba(142, 68, 173, 0.1)',
            showlegend=True,
            legendgroup='errors',
            hovertemplate='<b>Orientation Error</b><br>' +
                         'Time: %{x:.2f} s<br>' +
                         'Error: %{y:.2f} deg<br>' +
                         '<extra></extra>'
        ), row=5, col=1)
        
        fig.update_layout(
            title={
                'text': f'<b>End-Effector Quaternion Comparison vs Time</b><br>' +
                       f'<span style="font-size:14px; color:#666;">{os.path.basename(self.csv_file)}</span>',
                'x': 0.5,
                'xanchor': 'center',
                'font': {'size': 20}
            },
            height=1000,
            hovermode='x unified',
            showlegend=True,
            legend=dict(
                orientation="h",
                yanchor="bottom",
                y=-0.3,
                xanchor="center",
                x=0.5,
                bgcolor='rgba(255,255,255,0.8)',
                bordercolor='rgba(0,0,0,0.2)',
                borderwidth=1,
                font=dict(size=14),
                groupclick="toggleitem"
            ),
            font=dict(family="Arial, sans-serif"),
            plot_bgcolor='rgba(0,0,0,0)',
            paper_bgcolor='rgba(0,0,0,0)'
        )
        
        # Update subplot titles to be centered
        for i in fig['layout']['annotations']:
            i['font'] = dict(size=14, color='#2C3E50')
            i['xanchor'] = 'center'
    
        # Update axis labels
        for i in range(1, 5):
            fig.update_yaxes(title_text="<b>Quaternion</b>", row=i, col=1, 
                            showgrid=True, gridcolor='rgba(128,128,128,0.2)')
            fig.update_xaxes(showgrid=True, gridcolor='rgba(128,128,128,0.2)', row=i, col=1)
    
        fig.update_yaxes(title_text="<b>Error (deg)</b>", row=5, col=1, 
                        showgrid=True, gridcolor='rgba(128,128,128,0.2)')
        fig.update_xaxes(title_text="<b>Time (s)</b>", row=5, col=1, 
                        showgrid=True, gridcolor='rgba(128,128,128,0.2)')
    
        return fig
    
    def create_error_analysis_plot(self) -> go.Figure:
        """Create comprehensive error analysis plot"""
        fig = make_subplots(
            rows=2, cols=2,
            subplot_titles=('<b>Position Error Components</b>', '<b>Position Error vs Time</b>', 
                           '<b>Orientation Error vs Time</b>', '<b>Error Statistics</b>'),
            specs=[[{'type': 'scatter'}, {'type': 'scatter'}],
                   [{'type': 'scatter'}, {'type': 'bar'}]]
        )
        
        # Position error components
        error_components = ['pos_error_x', 'pos_error_y', 'pos_error_z']
        error_colors = ['#E74C3C', '#27AE60', '#3498DB']
        
        for comp, color in zip(error_components, error_colors):
            axis_label = comp.split('_')[-1].upper()
            fig.add_trace(go.Scatter(
                x=self.data['playback_time'],
                y=self.data[comp] * 1000,  # Convert to mm
                mode='lines',
                name=f'Error {axis_label}',
                line=dict(color=color, width=2),
                showlegend=True,
                legendgroup='error_components',
                hovertemplate=f'<b>Error {axis_label}</b><br>' +
                             'Time: %{x:.2f} s<br>' +
                             'Error: %{y:.2f} mm<br>' +
                             '<extra></extra>'
            ), row=1, col=1)
    
        # Position error norm
        fig.add_trace(go.Scatter(
            x=self.data['playback_time'],
            y=self.data['pos_error_norm'] * 1000,  # Convert to mm
            mode='lines',
            name='Position Error Norm',
            line=dict(color='#8E44AD', width=3),
            showlegend=True,
            legendgroup='errors',
            hovertemplate='<b>Position Error</b><br>' +
                         'Time: %{x:.2f} s<br>' +
                         'Error: %{y:.2f} mm<br>' +
                         '<extra></extra>'
        ), row=1, col=2)
        
        # Orientation error
        fig.add_trace(go.Scatter(
            x=self.data['playback_time'],
            y=np.degrees(self.data['orientation_error_angle']),
            mode='lines',
            name='Orientation Error',
            line=dict(color='#D35400', width=3),
            showlegend=True,
            legendgroup='errors',
            hovertemplate='<b>Orientation Error</b><br>' +
                         'Time: %{x:.2f} s<br>' +
                         'Error: %{y:.2f} deg<br>' +
                         '<extra></extra>'
        ), row=2, col=1)
        
        # Error statistics
        pos_error_stats = [
            self.data['pos_error_norm'].mean() * 1000,
            self.data['pos_error_norm'].max() * 1000,
            self.data['pos_error_norm'].std() * 1000
        ]
        
        orient_error_stats = [
            np.degrees(self.data['orientation_error_angle'].mean()),
            np.degrees(self.data['orientation_error_angle'].max()),
            np.degrees(self.data['orientation_error_angle'].std())
        ]
        
        fig.add_trace(go.Bar(
            x=['Mean', 'Max', 'Std'],
            y=pos_error_stats,
            name='Position Error (mm)',
            marker_color='#5DADE2',
            showlegend=True,
            legendgroup='statistics',
            hovertemplate='<b>Position Error</b><br>' +
                         'Statistic: %{x}<br>' +
                         'Value: %{y:.2f} mm<br>' +
                         '<extra></extra>'
        ), row=2, col=2)
        
        fig.add_trace(go.Bar(
            x=['Mean', 'Max', 'Std'],
            y=orient_error_stats,
            name='Orientation Error (deg)',
            marker_color='#F1948A',
            showlegend=True,
            legendgroup='statistics',
            yaxis='y4',
            hovertemplate='<b>Orientation Error</b><br>' +
                         'Statistic: %{x}<br>' +
                         'Value: %{y:.2f} deg<br>' +
                         '<extra></extra>'
        ), row=2, col=2)
        
        # Update layout
        fig.update_layout(
            title={
                'text': f'<b>Trajectory Following Error Analysis</b><br>' +
                       f'<span style="font-size:14px; color:#666;">{os.path.basename(self.csv_file)}</span>',
                'x': 0.5,
                'xanchor': 'center',
                'font': {'size': 20}
            },
            height=800,
            showlegend=True,
            legend=dict(
                orientation="h",
                yanchor="bottom",
                y=-0.3,
                xanchor="center",
                x=0.5,
                bgcolor='rgba(255,255,255,0.8)',
                bordercolor='rgba(0,0,0,0.2)',
                borderwidth=1,
                font=dict(size=14),
                groupclick="toggleitem"
            ),
            font=dict(family="Arial, sans-serif"),
            plot_bgcolor='rgba(0,0,0,0)',
            paper_bgcolor='rgba(0,0,0,0)'
        )
        
        # Update subplot titles to be centered
        for i in fig['layout']['annotations']:
            i['font'] = dict(size=14, color='#2C3E50')
            i['xanchor'] = 'center'
    
        # Update axis labels
        fig.update_yaxes(title_text="<b>Error (mm)</b>", row=1, col=1, 
                        showgrid=True, gridcolor='rgba(128,128,128,0.2)')
        fig.update_yaxes(title_text="<b>Error (mm)</b>", row=1, col=2, 
                        showgrid=True, gridcolor='rgba(128,128,128,0.2)')
        fig.update_yaxes(title_text="<b>Error (deg)</b>", row=2, col=1, 
                        showgrid=True, gridcolor='rgba(128,128,128,0.2)')
        fig.update_yaxes(title_text="<b>Error</b>", row=2, col=2, 
                        showgrid=True, gridcolor='rgba(128,128,128,0.2)')
    
        fig.update_xaxes(title_text="<b>Time (s)</b>", row=1, col=1, 
                        showgrid=True, gridcolor='rgba(128,128,128,0.2)')
        fig.update_xaxes(title_text="<b>Time (s)</b>", row=1, col=2, 
                        showgrid=True, gridcolor='rgba(128,128,128,0.2)')
        fig.update_xaxes(title_text="<b>Time (s)</b>", row=2, col=1, 
                        showgrid=True, gridcolor='rgba(128,128,128,0.2)')
        fig.update_xaxes(title_text="<b>Statistic</b>", row=2, col=2, 
                        showgrid=True, gridcolor='rgba(128,128,128,0.2)')
    
        return fig
    
    def create_velocity_comparison_plot(self) -> go.Figure:
        """Create velocity comparison plot"""
        fig = make_subplots(
            rows=2, cols=1,
            subplot_titles=('<b>Velocity Comparison</b>', '<b>Velocity Error</b>'),
            vertical_spacing=0.15
        )
        
        # Reference and actual velocities
        fig.add_trace(go.Scatter(
            x=self.data['playback_time'],
            y=self.data['ref_vel_norm'],
            mode='lines',
            name='Reference Velocity',
            line=dict(color='#2E86C1', width=2),
            showlegend=True,
            legendgroup='velocities',
            hovertemplate='<b>Reference Velocity</b><br>' +
                         'Time: %{x:.2f} s<br>' +
                         'Velocity: %{y:.4f} m/s<br>' +
                         '<extra></extra>'
        ), row=1, col=1)
        
        fig.add_trace(go.Scatter(
            x=self.data['playback_time'],
            y=self.data['actual_vel_norm'],
            mode='lines',
            name='Actual Velocity',
            line=dict(color='#E74C3C', width=2, dash='dash'),
            showlegend=True,
            legendgroup='velocities',
            hovertemplate='<b>Actual Velocity</b><br>' +
                         'Time: %{x:.2f} s<br>' +
                         'Velocity: %{y:.4f} m/s<br>' +
                         '<extra></extra>'
        ), row=1, col=1)
        
        # Velocity error
        vel_error = self.data['actual_vel_norm'] - self.data['ref_vel_norm']
        fig.add_trace(go.Scatter(
            x=self.data['playback_time'],
            y=vel_error,
            mode='lines',
            name='Velocity Error',
            line=dict(color='#8E44AD', width=3),
            fill='tonexty',
            fillcolor='rgba(142, 68, 173, 0.1)',
            showlegend=True,
            legendgroup='errors',
            hovertemplate='<b>Velocity Error</b><br>' +
                         'Time: %{x:.2f} s<br>' +
                         'Error: %{y:.4f} m/s<br>' +
                         '<extra></extra>'
        ), row=2, col=1)
        
        fig.update_layout(
            title={
                'text': f'<b>End-Effector Velocity Comparison</b><br>' +
                       f'<span style="font-size:14px; color:#666;">{os.path.basename(self.csv_file)}</span>',
                'x': 0.5,
                'xanchor': 'center',
                'font': {'size': 20}
            },
            height=600,
            showlegend=True,
            legend=dict(
                orientation="h",
                yanchor="bottom",
                y=-0.3,
                xanchor="center",
                x=0.5,
                bgcolor='rgba(255,255,255,0.8)',
                bordercolor='rgba(0,0,0,0.2)',
                borderwidth=1,
                font=dict(size=11),
                groupclick="toggleitem"
            ),
            font=dict(family="Arial, sans-serif"),
            plot_bgcolor='rgba(0,0,0,0)',
            paper_bgcolor='rgba(0,0,0,0)'
        )
        
        # Update subplot titles to be centered
        for i in fig['layout']['annotations']:
            i['font'] = dict(size=14, color='#2C3E50')
            i['xanchor'] = 'center'
    
        fig.update_yaxes(title_text="<b>Velocity (m/s)</b>", row=1, col=1, 
                        showgrid=True, gridcolor='rgba(128,128,128,0.2)')
        fig.update_yaxes(title_text="<b>Velocity Error (m/s)</b>", row=2, col=1, 
                        showgrid=True, gridcolor='rgba(128,128,128,0.2)')
        fig.update_xaxes(title_text="<b>Time (s)</b>", row=2, col=1, 
                        showgrid=True, gridcolor='rgba(128,128,128,0.2)')
        fig.update_xaxes(showgrid=True, gridcolor='rgba(128,128,128,0.2)', row=1, col=1)
        
        return fig
    
    def create_comprehensive_dashboard(self) -> go.Figure:
        """Create a comprehensive dashboard with key metrics"""
        fig = make_subplots(
            rows=2, cols=3,
            subplot_titles=(
                '<b>3D Trajectory Comparison</b>', 
                '<b>Z-Position Tracking</b>', 
                '<b>Quaternion Magnitude</b>',
                '<b>Position Error</b>', 
                '<b>Orientation Error</b>', 
                '<b>Performance Summary</b>'
            ),
            specs=[[{'type': 'scatter3d'}, {'type': 'scatter'}, {'type': 'scatter'}],
                   [{'type': 'scatter'}, {'type': 'scatter'}, {'type': 'bar'}]],
            horizontal_spacing=0.08,
            vertical_spacing=0.15
        )
        
        # 3D trajectory (simplified)
        fig.add_trace(go.Scatter3d(
            x=self.data['ref_eef_pos_x'],
            y=self.data['ref_eef_pos_y'],
            z=self.data['ref_eef_pos_z'],
            mode='lines',
            line=dict(color='#2E86C1', width=4),
            name='Reference Trajectory',
            showlegend=True,
            legendgroup='reference'
        ), row=1, col=1)
        
        fig.add_trace(go.Scatter3d(
            x=self.data['actual_eef_pos_x'],
            y=self.data['actual_eef_pos_y'],
            z=self.data['actual_eef_pos_z'],
            mode='lines',
            line=dict(color='#E74C3C', width=4),
            name='Actual Trajectory',
            showlegend=True,
            legendgroup='actual'
        ), row=1, col=1)
        
        # Position tracking (Z only for simplicity)
        fig.add_trace(go.Scatter(
            x=self.data['playback_time'],
            y=self.data['ref_eef_pos_z'],
            mode='lines',
            name='Reference Z',
            line=dict(color='#2E86C1', width=2),
            showlegend=True,
            legendgroup='reference'
        ), row=1, col=2)
        
        fig.add_trace(go.Scatter(
            x=self.data['playback_time'],
            y=self.data['actual_eef_pos_z'],
            mode='lines',
            name='Actual Z',
            line=dict(color='#E74C3C', width=2, dash='dash'),
            showlegend=True,
            legendgroup='actual'
        ), row=1, col=2)
        
        # Orientation tracking (quaternion magnitude)
        ref_quat_mag = np.sqrt(
            self.data['ref_eef_quat_x']**2 + self.data['ref_eef_quat_y']**2 + 
            self.data['ref_eef_quat_z']**2 + self.data['ref_eef_quat_w']**2
        )
        actual_quat_mag = np.sqrt(
            self.data['actual_eef_quat_x']**2 + self.data['actual_eef_quat_y']**2 + 
            self.data['actual_eef_quat_z']**2 + self.data['actual_eef_quat_w']**2
        )
        
        fig.add_trace(go.Scatter(
            x=self.data['playback_time'],
            y=ref_quat_mag,
            mode='lines',
            name='Reference |q|',
            line=dict(color='#2E86C1', width=2),
            showlegend=True,
            legendgroup='reference'
        ), row=1, col=3)
        
        fig.add_trace(go.Scatter(
            x=self.data['playback_time'],
            y=actual_quat_mag,
            mode='lines',
            name='Actual |q|',
            line=dict(color='#E74C3C', width=2, dash='dash'),
            showlegend=True,
            legendgroup='actual'
        ), row=1, col=3)
        
        # Position error
        fig.add_trace(go.Scatter(
            x=self.data['playback_time'],
            y=self.data['pos_error_norm'] * 1000,
            mode='lines',
            name='Position Error',
            line=dict(color='#8E44AD', width=3),
            showlegend=True,
            legendgroup='errors',
            fill='tonexty',
            fillcolor='rgba(142, 68, 173, 0.1)'
        ), row=2, col=1)
        
        # Orientation error
        fig.add_trace(go.Scatter(
            x=self.data['playback_time'],
            y=np.degrees(self.data['orientation_error_angle']),
            mode='lines',
            name='Orientation Error',
            line=dict(color='#D35400', width=3),
            showlegend=True,
            legendgroup='errors',
            fill='tonexty',
            fillcolor='rgba(211, 84, 0, 0.1)'
        ), row=2, col=2)
        
        # Summary statistics
        metrics = ['Pos RMSE\n(mm)', 'Pos Max\n(mm)', 'Orient RMSE\n(deg)', 'Orient Max\n(deg)']
        values = [
            np.sqrt(np.mean(self.data['pos_error_norm']**2)) * 1000,
            self.data['pos_error_norm'].max() * 1000,
            np.degrees(np.sqrt(np.mean(self.data['orientation_error_angle']**2))),
            np.degrees(self.data['orientation_error_angle'].max())
        ]
        
        # Create grouped bar chart with better colors
        fig.add_trace(go.Bar(
            x=metrics[:2],
            y=values[:2],
            name='Position Metrics',
            marker_color='#5DADE2',
            showlegend=True,
            legendgroup='metrics',
            hovertemplate='<b>%{x}</b><br>Value: %{y:.2f}<extra></extra>'
        ), row=2, col=3)
        
        fig.add_trace(go.Bar(
            x=metrics[2:],
            y=values[2:],
            name='Orientation Metrics',
            marker_color='#F1948A',
            showlegend=True,
            legendgroup='metrics',
            hovertemplate='<b>%{x}</b><br>Value: %{y:.2f}<extra></extra>'
        ), row=2, col=3)
        
        # Update layout with centered title and improved styling
        fig.update_layout(
            title={
                'text': f'<b>Trajectory Following Performance Dashboard</b><br>' +
                       f'<span style="font-size:14px; color:#666;">{os.path.basename(self.csv_file)}</span>',
                'x': 0.5,
                'xanchor': 'center',
                'font': {'size': 20}
            },
            height=900,
            showlegend=True,
            legend=dict(
                orientation="h",
                yanchor="bottom",
                y=-0.3,
                xanchor="center",
                x=0.5,
                bgcolor='rgba(255,255,255,0.8)',
                bordercolor='rgba(0,0,0,0.2)',
                borderwidth=1,
                font=dict(size=14),
                groupclick="toggleitem"
            ),
            font=dict(family="Arial, sans-serif"),
            plot_bgcolor='rgba(0,0,0,0)',
            paper_bgcolor='rgba(0,0,0,0)'
        )
        
        # Update subplot titles to be centered
        for i in fig['layout']['annotations']:
            i['font'] = dict(size=14, color='#2C3E50')
            i['xanchor'] = 'center'
        
        # Update 3D scene
        fig.update_scenes(
            xaxis_title='<b>X (m)</b>',
            yaxis_title='<b>Y (m)</b>',
            zaxis_title='<b>Z (m)</b>',
            aspectmode='cube',
            camera=dict(eye=dict(x=1.3, y=1.3, z=1.3))
        )
        
        # Update axis labels with better formatting
        fig.update_xaxes(title_text="<b>Time (s)</b>", row=1, col=2, showgrid=True, gridcolor='rgba(128,128,128,0.2)')
        fig.update_yaxes(title_text="<b>Z Position (m)</b>", row=1, col=2, showgrid=True, gridcolor='rgba(128,128,128,0.2)')
        
        fig.update_xaxes(title_text="<b>Time (s)</b>", row=1, col=3, showgrid=True, gridcolor='rgba(128,128,128,0.2)')
        fig.update_yaxes(title_text="<b>Quaternion Magnitude</b>", row=1, col=3, showgrid=True, gridcolor='rgba(128,128,128,0.2)')
        
        fig.update_xaxes(title_text="<b>Time (s)</b>", row=2, col=1, showgrid=True, gridcolor='rgba(128,128,128,0.2)')
        fig.update_yaxes(title_text="<b>Error (mm)</b>", row=2, col=1, showgrid=True, gridcolor='rgba(128,128,128,0.2)')
        
        fig.update_xaxes(title_text="<b>Time (s)</b>", row=2, col=2, showgrid=True, gridcolor='rgba(128,128,128,0.2)')
        fig.update_yaxes(title_text="<b>Error (degrees)</b>", row=2, col=2, showgrid=True, gridcolor='rgba(128,128,128,0.2)')
        
        fig.update_xaxes(title_text="<b>Performance Metrics</b>", row=2, col=3, showgrid=True, gridcolor='rgba(128,128,128,0.2)')
        fig.update_yaxes(title_text="<b>Value</b>", row=2, col=3, showgrid=True, gridcolor='rgba(128,128,128,0.2)')
        
        return fig

    def print_statistics(self):
        """Print comprehensive trajectory following statistics"""
        print("\n" + "="*80)
        print("TRAJECTORY FOLLOWING STATISTICS")
        print("="*80)
        print(f"File: {os.path.basename(self.csv_file)}")
        print(f"Duration: {self.data['playback_time'].max():.2f} seconds")
        print(f"Data Points: {len(self.data)}")
        print(f"Frequency: {len(self.data) / self.data['playback_time'].max():.1f} Hz")
        print()
        
        # Position error statistics
        pos_error = self.data['pos_error_norm'] * 1000  # Convert to mm
        print("📍 POSITION ERROR STATISTICS (mm):")
        print(f"  Mean:     {pos_error.mean():.3f}")
        print(f"  RMSE:     {np.sqrt(np.mean(pos_error**2)):.3f}")
        print(f"  Max:      {pos_error.max():.3f}")
        print(f"  Std:      {pos_error.std():.3f}")
        print(f"  95th %:   {np.percentile(pos_error, 95):.3f}")
        print()
        
        # Orientation error statistics
        orient_error = np.degrees(self.data['orientation_error_angle'])
        print("🧭 ORIENTATION ERROR STATISTICS (degrees):")
        print(f"  Mean:     {orient_error.mean():.3f}")
        print(f"  RMSE:     {np.sqrt(np.mean(orient_error**2)):.3f}")
        print(f"  Max:      {orient_error.max():.3f}")
        print(f"  Std:      {orient_error.std():.3f}")
        print(f"  95th %:   {np.percentile(orient_error, 95):.3f}")
        print()
        
        # Velocity statistics (if available)
        if 'ref_vel_norm' in self.data.columns:
            print("🚀 VELOCITY STATISTICS:")
            print(f"  Ref mean velocity:    {self.data['ref_vel_norm'].mean():.4f} m/s")
            print(f"  Actual mean velocity: {self.data['actual_vel_norm'].mean():.4f} m/s")
            print(f"  Ref max velocity:     {self.data['ref_vel_norm'].max():.4f} m/s")
            print(f"  Actual max velocity:  {self.data['actual_vel_norm'].max():.4f} m/s")
            print()
        
        # Trajectory characteristics
        print("📏 TRAJECTORY CHARACTERISTICS:")
        ref_distance = self.calculate_path_length('ref')
        actual_distance = self.calculate_path_length('actual')
        print(f"  Reference path length: {ref_distance:.4f} m")
        print(f"  Actual path length:    {actual_distance:.4f} m")
        print(f"  Path length error:     {abs(actual_distance - ref_distance):.4f} m")
        print()
        
        # Performance assessment
        print("🎯 PERFORMANCE ASSESSMENT:")
        if pos_error.mean() < 5.0:
            print("  Position tracking: ✅ EXCELLENT (< 5mm mean error)")
        elif pos_error.mean() < 10.0:
            print("  Position tracking: ✅ GOOD (< 10mm mean error)")
        elif pos_error.mean() < 20.0:
            print("  Position tracking: ⚠️ FAIR (< 20mm mean error)")
        else:
            print("  Position tracking: ❌ POOR (> 20mm mean error)")
            
        if orient_error.mean() < 2.0:
            print("  Orientation tracking: ✅ EXCELLENT (< 2° mean error)")
        elif orient_error.mean() < 5.0:
            print("  Orientation tracking: ✅ GOOD (< 5° mean error)")
        elif orient_error.mean() < 10.0:
            print("  Orientation tracking: ⚠️ FAIR (< 10° mean error)")
        else:
            print("  Orientation tracking: ❌ POOR (> 10° mean error)")
        
        print("="*80)
    
    def calculate_path_length(self, trajectory_type: str) -> float:
        """Calculate total path length for reference or actual trajectory"""
        if trajectory_type == 'ref':
            pos_x = self.data['ref_eef_pos_x'].values
            pos_y = self.data['ref_eef_pos_y'].values
            pos_z = self.data['ref_eef_pos_z'].values
        else:
            pos_x = self.data['actual_eef_pos_x'].values
            pos_y = self.data['actual_eef_pos_y'].values
            pos_z = self.data['actual_eef_pos_z'].values
        
        # Calculate distances between consecutive points
        distances = np.sqrt(
            np.diff(pos_x)**2 + 
            np.diff(pos_y)**2 + 
            np.diff(pos_z)**2
        )
        
        return np.sum(distances)

    def save_publication_plots(self, output_dir: str, base_name: str):
        """Save plots optimized for scientific publication"""
        
        # Publication settings for different journal formats
        configs = {
            'single_column': {
                'width': 350,    # Single column width (mm → pixels)
                'height': 250,
                'scale': 4,      # Very high resolution
                'format': 'pdf'
            },
            'double_column': {
                'width': 700,    # Double column width
                'height': 500,
                'scale': 3,
                'format': 'pdf'
            },
            'full_page': {
                'width': 1000,   # Full page width
                'height': 700,
                'scale': 3,
                'format': 'pdf'
            }
        }
        
        # Clean plots for publication (remove backgrounds, optimize fonts)
        publication_layout = {
            'font': dict(family="Times New Roman", size=12),  # Standard academic font
            'plot_bgcolor': 'white',
            'paper_bgcolor': 'white',
            'margin': dict(l=60, r=20, t=60, b=60),  # Proper margins
        }
        
        # Update all figures with publication layout
        plots = {
            'position_comparison': self.create_position_comparison_plot(),
            'error_analysis': self.create_error_analysis_plot(),
            'velocity_comparison': self.create_velocity_comparison_plot(),
            '3d_trajectory': self.create_3d_trajectory_comparison()
        }
        
        for plot_name, fig in plots.items():
            fig.update_layout(**publication_layout)
            
            # Choose appropriate size based on plot complexity
            if plot_name in ['position_comparison', 'error_analysis']:
                config = configs['double_column']
            elif plot_name == 'velocity_comparison':
                config = configs['single_column'] 
            else:
                config = configs['full_page']
            
            # Save with publication naming convention
            filename = f"{base_name}_{plot_name}_publication.pdf"
            fig.write_image(os.path.join(output_dir, filename), **config)
        
        print("📚 Publication-ready plots saved with academic formatting")
        print("💾 Saving publication-quality PDF plots...")
    
        # Publication-quality PDF settings
        pdf_config = {
            'width': 1200,      # Publication width
            'height': 800,      # Publication height  
            'scale': 3,         # High resolution scaling (300 DPI equivalent)
            'format': 'pdf',
            'engine': 'kaleido'
        }
        
        try:
            print("📄 Saving 2D plots as PDF...")
            plots['position_comparison'].write_image(os.path.join(output_dir, f"{base_name}_position_comparison.pdf"), **pdf_config)
            plots['quaternion_comparison'].write_image(os.path.join(output_dir, f"{base_name}_quaternion_comparison.pdf"), **pdf_config)
            plots['error_analysis'].write_image(os.path.join(output_dir, f"{base_name}_error_analysis.pdf"), **pdf_config)
            plots['velocity_comparison'].write_image(os.path.join(output_dir, f"{base_name}_velocity_comparison.pdf"), **pdf_config)
            
            # For 3D plot, save as static PDF view
            plots['3d_trajectory'].write_image(os.path.join(output_dir, f"{base_name}_3d_trajectory.pdf"), **pdf_config)
            
            # Dashboard as PDF (will be static but readable)
            plots['dashboard'].write_image(os.path.join(output_dir, f"{base_name}_dashboard.pdf"), 
                                        width=1600, height=1200, scale=2, format='pdf')
            
            print("✅ Publication-quality PDF plots saved successfully")
            print(f"📁 Files saved to: {output_dir}/")
            
            # List the saved files
            saved_files = [
                f"{base_name}_position_comparison.pdf",
                f"{base_name}_quaternion_comparison.pdf", 
                f"{base_name}_error_analysis.pdf",
                f"{base_name}_velocity_comparison.pdf",
                f"{base_name}_3d_trajectory.pdf",
                f"{base_name}_dashboard.pdf"
            ]
            
            print("📄 PDF files created:")
            for file in saved_files:
                file_path = os.path.join(output_dir, file)
                if os.path.exists(file_path):
                    print(f"  ✅ {file}")
                else:
                    print(f"  ❌ {file} - FAILED")
            
        except Exception as e:
            print(f"⚠️ Could not save PDF plots: {e}")
            print("💡 Make sure 'kaleido' is installed: pip install kaleido")
            print("💡 Check write permissions for the output directory")
            import traceback
            traceback.print_exc()

def main():
    parser = argparse.ArgumentParser(description="Trajectory Comparison Visualization")
    parser.add_argument("--csv", type=str, required=True,
                       help="Path to trajectory comparison CSV file")
    parser.add_argument("--output", type=str, default=None,
                       help="Output directory for HTML plots")
    parser.add_argument("--show", action="store_true",
                       help="Show interactive plots in browser")
    parser.add_argument("--stats", action="store_true", default=True,
                       help="Print trajectory statistics")
    parser.add_argument("--format", type=str, choices=['html', 'pdf', 'both'], default='html',
                       help="Output format: html, pdf, or both")
    
    args = parser.parse_args()
    
    try:
        print("🎯 Trajectory Comparison Visualization")
        print("="*60)
        
        # Create comparison object
        comparison = TrajectoryComparison(args.csv)
        
        # Load data
        if not comparison.load_data():
            return 1
        
        # Calculate additional metrics
        print("🔄 Calculating additional metrics...")
        comparison.calculate_additional_metrics()
        
        # Print statistics
        if args.stats:
            comparison.print_statistics()
        
        # Create plots
        print("📊 Creating visualizations...")
        
        # 3D trajectory comparison
        print("  - 3D Trajectory Comparison")
        traj_3d_fig = comparison.create_3d_trajectory_comparison()
        
        # Position comparison
        print("  - Position vs Time Comparison")
        pos_fig = comparison.create_position_comparison_plot()
        
        # Quaternion comparison
        print("  - Quaternion vs Time Comparison")
        quat_fig = comparison.create_quaternion_comparison_plot()
        
        # Error analysis
        print("  - Error Analysis")
        error_fig = comparison.create_error_analysis_plot()
        
        # Velocity comparison
        print("  - Velocity Comparison")
        vel_fig = comparison.create_velocity_comparison_plot()
        
        # Comprehensive dashboard
        print("  - Comprehensive Dashboard")
        dashboard_fig = comparison.create_comprehensive_dashboard()
        
        # Save plots if output directory specified
        if args.output:
            os.makedirs(args.output, exist_ok=True)
            base_name = os.path.splitext(os.path.basename(args.csv))[0]
            
            print(f"💾 Saving plots to {args.output}/ in {args.format} format...")
            
            if args.format in ['html', 'both']:
                traj_3d_fig.write_html(os.path.join(args.output, f"{base_name}_3d_trajectory.html"))
                pos_fig.write_html(os.path.join(args.output, f"{base_name}_position_comparison.html"))
                quat_fig.write_html(os.path.join(args.output, f"{base_name}_quaternion_comparison.html"))
                error_fig.write_html(os.path.join(args.output, f"{base_name}_error_analysis.html"))
                vel_fig.write_html(os.path.join(args.output, f"{base_name}_velocity_comparison.html"))
                dashboard_fig.write_html(os.path.join(args.output, f"{base_name}_dashboard.html"))
                print("✅ HTML plots saved successfully")
            
            if args.format in ['pdf', 'both']:
                print("💾 Saving publication-quality PDF plots...")
                
                # Publication-quality PDF settings
                pdf_config = {
                    'width': 1200,      # Publication width
                    'height': 800,      # Publication height  
                    'scale': 3,         # High resolution scaling (300 DPI equivalent)
                    'format': 'pdf',
                    'engine': 'kaleido'
                }
                
                try:
                    print("📄 Saving 2D plots as PDF...")
                    pos_fig.write_image(os.path.join(args.output, f"{base_name}_position_comparison.pdf"), **pdf_config)
                    quat_fig.write_image(os.path.join(args.output, f"{base_name}_quaternion_comparison.pdf"), **pdf_config)
                    error_fig.write_image(os.path.join(args.output, f"{base_name}_error_analysis.pdf"), **pdf_config)
                    vel_fig.write_image(os.path.join(args.output, f"{base_name}_velocity_comparison.pdf"), **pdf_config)
                    
                    # For 3D plot, save as static PDF view
                    traj_3d_fig.write_image(os.path.join(args.output, f"{base_name}_3d_trajectory.pdf"), **pdf_config)
                    
                    # Dashboard as PDF (will be static but readable)
                    dashboard_fig.write_image(os.path.join(args.output, f"{base_name}_dashboard.pdf"), 
                                            width=1600, height=1200, scale=2, format='pdf')
                    
                    print("✅ Publication-quality PDF plots saved successfully")
                    print(f"📁 Files saved to: {args.output}/")
                    
                    # List the saved files
                    saved_files = [
                        f"{base_name}_position_comparison.pdf",
                        f"{base_name}_quaternion_comparison.pdf", 
                        f"{base_name}_error_analysis.pdf",
                        f"{base_name}_velocity_comparison.pdf",
                        f"{base_name}_3d_trajectory.pdf",
                        f"{base_name}_dashboard.pdf"
                    ]
                    
                    print("📄 PDF files created:")
                    for file in saved_files:
                        file_path = os.path.join(args.output, file)
                        if os.path.exists(file_path):
                            print(f"  ✅ {file}")
                        else:
                            print(f"  ❌ {file} - FAILED")
                    
                except Exception as e:
                    print(f"⚠️ Could not save PDF plots: {e}")
                    print("💡 Make sure 'kaleido' is installed: pip install kaleido")
                    print("💡 Check write permissions for the output directory")
                    import traceback
                    traceback.print_exc()
        
        # Show plots if requested
        if args.show:
            print("🖥️ Opening interactive plots...")
            dashboard_fig.show()
            traj_3d_fig.show()
            pos_fig.show()
            quat_fig.show()
            error_fig.show()
            vel_fig.show()
        
        print("\n✅ Trajectory comparison visualization completed!")
        
    except KeyboardInterrupt:
        print("\n\n👋 Visualization interrupted by user")
        return 1
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())