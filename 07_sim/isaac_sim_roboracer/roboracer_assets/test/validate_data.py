#!/usr/bin/env python3
"""
Script to validate the saved hybrid neural ODE data and verify frame transformations.
"""

import numpy as np
import matplotlib.pyplot as plt
import os

def validate_saved_data(filename):
    """
    Load and validate the saved hybrid neural ODE data.
    """
    if not os.path.exists(filename):
        print(f"Error: File {filename} not found!")
        return False
    
    try:
        # Load the data
        data = np.load(filename)
        print(f"Successfully loaded data from {filename}")
        print(f"Available keys: {list(data.keys())}")
        
        # Extract data
        timestamps = data['timestamps']
        states = data['states']
        inputs = data['inputs']
        ground_plane_angle = data['ground_plane_angle']
        rotation_matrix = data['rotation_matrix']
        num_envs = data['num_envs']
        
        # Print basic information
        print(f"\nData Summary:")
        print(f"  Ground plane angle: {ground_plane_angle}°")
        print(f"  Number of environments: {num_envs}")
        print(f"  Simulation duration: {timestamps[-1]:.2f}s")
        print(f"  Time step: {timestamps[1] - timestamps[0]:.4f}s")
        print(f"  Total timesteps: {len(timestamps)}")
        
        # Validate shapes
        print(f"\nData Shapes:")
        print(f"  Timestamps: {timestamps.shape}")
        print(f"  States: {states.shape}")
        print(f"  Inputs: {inputs.shape}")
        print(f"  Rotation matrix: {rotation_matrix.shape}")
        
        # Check for NaN or infinite values
        print(f"\nData Quality Check:")
        print(f"  States - NaN count: {np.sum(np.isnan(states))}")
        print(f"  States - Inf count: {np.sum(np.isinf(states))}")
        print(f"  Inputs - NaN count: {np.sum(np.isnan(inputs))}")
        print(f"  Inputs - Inf count: {np.sum(np.isinf(inputs))}")
        
        # State variable analysis
        state_names = ['x_pos', 'y_pos', 'yaw', 'steering', 'velocity', 'side_slip', 'yaw_rate']
        print(f"\nState Variable Ranges:")
        for i, name in enumerate(state_names):
            min_val = np.min(states[:, :, i])
            max_val = np.max(states[:, :, i])
            mean_val = np.mean(states[:, :, i])
            std_val = np.std(states[:, :, i])
            print(f"  {name:12}: [{min_val:8.3f}, {max_val:8.3f}] μ={mean_val:8.3f} σ={std_val:6.3f}")
        
        # Control input analysis
        input_names = ['acceleration', 'steering_rate']
        print(f"\nControl Input Ranges:")
        for i, name in enumerate(input_names):
            min_val = np.min(inputs[:, :, i])
            max_val = np.max(inputs[:, :, i])
            mean_val = np.mean(inputs[:, :, i])
            std_val = np.std(inputs[:, :, i])
            print(f"  {name:12}: [{min_val:8.3f}, {max_val:8.3f}] μ={mean_val:8.3f} σ={std_val:6.3f}")
        
        # Validate frame transformation
        print(f"\nFrame Transformation Validation:")
        print(f"  Rotation matrix:")
        print(f"    {rotation_matrix}")
        
        # Check rotation matrix properties
        det = np.linalg.det(rotation_matrix)
        is_orthogonal = np.allclose(rotation_matrix @ rotation_matrix.T, np.eye(3))
        print(f"  Determinant: {det:.6f} (should be 1.0)")
        print(f"  Orthogonal: {is_orthogonal} (should be True)")
        
        # Analyze trajectory patterns
        print(f"\nTrajectory Analysis:")
        for env in range(min(num_envs, 3)):  # Analyze first 3 robots
            x_range = np.max(states[:, env, 0]) - np.min(states[:, env, 0])
            y_range = np.max(states[:, env, 1]) - np.min(states[:, env, 1])
            final_x = states[-1, env, 0] - states[0, env, 0]
            final_y = states[-1, env, 1] - states[0, env, 1]
            print(f"  Robot {env+1}: X-range={x_range:.3f}m, Y-range={y_range:.3f}m, "
                  f"Final displacement=({final_x:.3f}, {final_y:.3f})m")
        
        # Plot basic trajectory
        plt.figure(figsize=(12, 8))
        
        # Plot trajectories
        plt.subplot(2, 2, 1)
        for env in range(min(num_envs, 5)):
            plt.plot(states[:, env, 0], states[:, env, 1], alpha=0.7, label=f'Robot {env+1}')
        plt.xlabel('X Position [m] (Downhill)')
        plt.ylabel('Y Position [m] (Across Slope)')
        plt.title('Robot Trajectories in Inclined Frame')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        
        # Plot velocity over time
        plt.subplot(2, 2, 2)
        for env in range(min(num_envs, 5)):
            plt.plot(timestamps, states[:, env, 4], alpha=0.7, label=f'Robot {env+1}')
        plt.xlabel('Time [s]')
        plt.ylabel('Velocity [m/s]')
        plt.title('Velocity Over Time')
        plt.legend()
        plt.grid(True)
        
        # Plot yaw angle over time
        plt.subplot(2, 2, 3)
        for env in range(min(num_envs, 5)):
            plt.plot(timestamps, np.rad2deg(states[:, env, 2]), alpha=0.7, label=f'Robot {env+1}')
        plt.xlabel('Time [s]')
        plt.ylabel('Yaw Angle [deg]')
        plt.title('Yaw Angle Over Time')
        plt.legend()
        plt.grid(True)
        
        # Plot control inputs
        plt.subplot(2, 2, 4)
        for env in range(min(num_envs, 5)):
            plt.plot(timestamps, inputs[:, env, 0], alpha=0.7, label=f'Robot {env+1}')
        plt.xlabel('Time [s]')
        plt.ylabel('Acceleration [m/s²]')
        plt.title('Control Input: Acceleration')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        plt.show()
        
        print(f"\nData validation completed successfully!")
        return True
        
    except Exception as e:
        print(f"Error loading data: {e}")
        return False

def main():
    """
    Main function to validate saved data.
    """
    # Look for the data file
    data_file = "hybrid_neural_ode_data.npz"
    
    if not os.path.exists(data_file):
        print(f"Data file {data_file} not found in current directory.")
        print("Please run the simulation first to generate the data.")
        return
    
    print("=== Hybrid Neural ODE Data Validation ===")
    validate_saved_data(data_file)

if __name__ == "__main__":
    main()
