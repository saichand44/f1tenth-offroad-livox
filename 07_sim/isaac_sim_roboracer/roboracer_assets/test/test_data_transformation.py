#!/usr/bin/env python3
"""
Test script to validate the data transformation functions for the hybrid neural ODE model.
This script tests the transformation logic without requiring the full Isaac Lab environment.
"""

import numpy as np
import torch
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation


def get_gravity_vec(angle_in_deg, g_original):
    """
    Compute the new gravity vector that corresponds to the rotation of ground plane
    by "angle" in deg in positive Y axis
    """
    angle_in_rad = np.deg2rad(angle_in_deg)
    Ry = np.array([[np.cos(angle_in_rad), 0, np.sin(angle_in_rad)],
                   [0, 1, 0],
                   [-np.sin(angle_in_rad), 0, np.cos(angle_in_rad)]])
    g_transform = Ry.T @ g_original
    g_transform = g_transform.flatten()
    return (float(g_transform[0]), float(g_transform[1]), float(g_transform[2])), Ry.T


def transform_to_inclined_frame(positions, velocities, orientations, rotation_matrix):
    """
    Transform positions, velocities, and orientations from world frame to inclined surface frame.
    """
    # Convert rotation matrix to torch tensor
    R = torch.tensor(rotation_matrix, dtype=torch.float32, device=positions.device)
    
    # Transform positions and velocities
    transformed_positions = torch.matmul(positions, R.T)
    transformed_velocities = torch.matmul(velocities, R.T)
    
    # Extract yaw angles from quaternions in the inclined frame
    yaw_angles = torch.zeros(orientations.shape[0], device=orientations.device)
    
    for i in range(orientations.shape[0]):
        # Convert quaternion to rotation matrix
        quat = orientations[i]  # (w, x, y, z)
        w, x, y, z = quat[0], quat[1], quat[2], quat[3]
        
        # Rotation matrix from quaternion
        R_quat = torch.tensor([
            [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
        ], device=orientations.device)
        
        # Transform to inclined frame
        R_inclined = torch.matmul(R, R_quat)
        
        # Extract yaw angle (rotation around Z-axis in inclined frame)
        yaw_angles[i] = torch.atan2(R_inclined[1, 0], R_inclined[0, 0])
    
    return transformed_positions, transformed_velocities, yaw_angles


def compute_state_variables(positions, velocities, yaw_angles, steering_angles, linear_accelerations, rotation_matrix):
    """
    Compute the 7 state variables for the hybrid neural ODE model.
    """
    # Transform to inclined frame
    transformed_pos, transformed_vel, _ = transform_to_inclined_frame(
        positions, velocities, 
        torch.zeros(positions.shape[0], 4, device=positions.device), 
        rotation_matrix
    )
    
    # Extract x, y positions in inclined frame
    x_pos = transformed_pos[:, 0]
    y_pos = transformed_pos[:, 1]
    
    # Compute velocity magnitude in inclined frame (projected onto surface)
    velocity_magnitude = torch.sqrt(transformed_vel[:, 0]**2 + transformed_vel[:, 1]**2)
    
    # Compute side slip angle (angle between heading and velocity direction)
    velocity_direction = torch.atan2(transformed_vel[:, 1], transformed_vel[:, 0])
    side_slip = velocity_direction - yaw_angles
    
    # Normalize side slip angle to [-pi, pi]
    side_slip = torch.atan2(torch.sin(side_slip), torch.cos(side_slip))
    
    # Compute yaw rate (will be computed from consecutive yaw measurements)
    yaw_rate = torch.zeros_like(yaw_angles)
    
    # Stack all state variables
    states = torch.stack([
        x_pos, y_pos, yaw_angles, steering_angles, 
        velocity_magnitude, side_slip, yaw_rate
    ], dim=1)
    
    return states


def test_transformation():
    """
    Test the transformation functions with sample data.
    """
    print("Testing data transformation functions...")
    
    # Test parameters
    GROUND_PLANE_ANGLE = -20.0  # degrees
    num_envs = 3
    
    # Calculate rotation matrix
    G_ORIGINAL = np.array([0.0, 0.0, -9.81]).reshape(3, 1)
    G_TRANSFORM, G_R_P = get_gravity_vec(GROUND_PLANE_ANGLE, G_ORIGINAL)
    
    print(f"Ground plane angle: {GROUND_PLANE_ANGLE} degrees")
    print(f"Original gravity: {G_ORIGINAL.flatten()}")
    print(f"Transformed gravity: {G_TRANSFORM}")
    print(f"Rotation matrix shape: {G_R_P.shape}")
    
    # Generate sample data
    positions = torch.tensor([
        [1.0, 0.0, 0.5],
        [2.0, 1.0, 0.3],
        [0.5, -0.5, 0.8]
    ], dtype=torch.float32)
    
    velocities = torch.tensor([
        [2.0, 0.1, 0.0],
        [1.5, -0.2, 0.1],
        [3.0, 0.5, -0.1]
    ], dtype=torch.float32)
    
    # Sample quaternions (w, x, y, z) for different yaw angles
    orientations = torch.tensor([
        [0.9659, 0.0, 0.0, 0.2588],  # ~30 degrees yaw
        [0.8660, 0.0, 0.0, 0.5000],  # ~60 degrees yaw
        [1.0, 0.0, 0.0, 0.0]         # 0 degrees yaw
    ], dtype=torch.float32)
    
    steering_angles = torch.tensor([0.1, -0.2, 0.05], dtype=torch.float32)
    linear_accelerations = torch.zeros(3, 3, dtype=torch.float32)
    
    # Test transformation
    transformed_pos, transformed_vel, yaw_angles = transform_to_inclined_frame(
        positions, velocities, orientations, G_R_P
    )
    
    print(f"\nOriginal positions:\n{positions}")
    print(f"Transformed positions:\n{transformed_pos}")
    print(f"Original velocities:\n{velocities}")
    print(f"Transformed velocities:\n{transformed_vel}")
    print(f"Extracted yaw angles: {yaw_angles}")
    
    # Test state computation
    states = compute_state_variables(
        positions, velocities, yaw_angles, steering_angles, 
        linear_accelerations, G_R_P
    )
    
    print(f"\nComputed states shape: {states.shape}")
    print(f"States (x_pos, y_pos, yaw, steering, velocity, side_slip, yaw_rate):")
    state_names = ['x_pos', 'y_pos', 'yaw', 'steering', 'velocity', 'side_slip', 'yaw_rate']
    for i, name in enumerate(state_names):
        print(f"  {name}: {states[:, i]}")
    
    # Test data format for saving
    print(f"\nTesting data format for .npz saving...")
    timestamps = np.linspace(0, 1, 10)
    states_data = np.random.rand(10, num_envs, 7)  # 10 timesteps, 3 robots, 7 states
    inputs_data = np.random.rand(10, num_envs, 2)  # 10 timesteps, 3 robots, 2 inputs
    
    # Save test data
    np.savez('test_hybrid_neural_ode_data.npz',
             timestamps=timestamps,
             states=states_data,
             inputs=inputs_data,
             ground_plane_angle=GROUND_PLANE_ANGLE,
             rotation_matrix=G_R_P,
             num_envs=num_envs)
    
    print(f"Test data saved to test_hybrid_neural_ode_data.npz")
    
    # Load and verify
    loaded_data = np.load('test_hybrid_neural_ode_data.npz')
    print(f"Loaded data keys: {list(loaded_data.keys())}")
    print(f"Timestamps shape: {loaded_data['timestamps'].shape}")
    print(f"States shape: {loaded_data['states'].shape}")
    print(f"Inputs shape: {loaded_data['inputs'].shape}")
    
    print("\nTest completed successfully!")


if __name__ == "__main__":
    test_transformation()
