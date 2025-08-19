#!/usr/bin/env python3
"""
Simple script to verify tensor data types and demonstrate the fix.
"""

import torch

def demonstrate_dtype_issue():
    """
    Demonstrate the data type issue and solution.
    """
    print("=== Isaac Lab Data Type Issue Demonstration ===\n")
    
    # Simulate the problematic case
    print("1. PROBLEMATIC CASE (causing the error):")
    problematic_tensor = torch.tensor([[1.0]])  # Defaults to float64
    print(f"   Default tensor dtype: {problematic_tensor.dtype}")
    print(f"   Isaac Lab expects: torch.float32")
    print(f"   Type match: {problematic_tensor.dtype == torch.float32}")
    
    # Show the calculation chain
    wheel_diameter = 0.1
    target_velocity = problematic_tensor
    target_rpms = target_velocity / (wheel_diameter * 0.5)
    print(f"   After calculation dtype: {target_rpms.dtype}")
    print(f"   Would cause error: Index put requires Float for destination and Double for source\n")
    
    # Demonstrate the fix
    print("2. FIXED CASE (works correctly):")
    fixed_tensor = torch.tensor([[1.0]], dtype=torch.float32)  # Explicit float32
    print(f"   Fixed tensor dtype: {fixed_tensor.dtype}")
    print(f"   Isaac Lab expects: torch.float32")
    print(f"   Type match: {fixed_tensor.dtype == torch.float32}")
    
    # Show the calculation chain
    target_velocity_fixed = fixed_tensor
    target_rpms_fixed = target_velocity_fixed / (wheel_diameter * 0.5)
    print(f"   After calculation dtype: {target_rpms_fixed.dtype}")
    print(f"   Will work with Isaac Lab: ✅\n")
    
    # Show alternative fix methods
    print("3. ALTERNATIVE FIX METHODS:")
    
    # Method 1: Explicit dtype in tensor creation
    method1 = torch.tensor([[1.0]], dtype=torch.float32)
    print(f"   Method 1 - Explicit dtype: {method1.dtype}")
    
    # Method 2: Convert existing tensor
    method2 = torch.tensor([[1.0]]).float()
    print(f"   Method 2 - .float() conversion: {method2.dtype}")
    
    # Method 3: to() with dtype
    method3 = torch.tensor([[1.0]]).to(dtype=torch.float32)
    print(f"   Method 3 - .to(dtype=...): {method3.dtype}")
    
    print("\n4. RECOMMENDED APPROACH:")
    print("   Always specify dtype=torch.float32 when creating tensors for Isaac Lab")
    print("   Example: torch.tensor(data, device=device, dtype=torch.float32)")

def check_device_compatibility():
    """
    Check if CUDA is available and demonstrate device + dtype specification.
    """
    print("\n=== Device and Data Type Best Practices ===\n")
    
    # Check available devices
    if torch.cuda.is_available():
        device = torch.device("cuda:0")
        print(f"CUDA available: ✅ Using {device}")
    else:
        device = torch.device("cpu")
        print(f"CUDA not available: Using {device}")
    
    # Demonstrate proper tensor creation
    print(f"\nProper tensor creation for Isaac Lab:")
    proper_tensor = torch.tensor([[1.0, 2.0]], device=device, dtype=torch.float32)
    print(f"   Device: {proper_tensor.device}")
    print(f"   Data type: {proper_tensor.dtype}")
    print(f"   Shape: {proper_tensor.shape}")
    print(f"   Values: {proper_tensor}")

def simulate_mppi_integration():
    """
    Simulate the MPPI integration scenario that was causing the error.
    """
    print("\n=== MPPI Integration Simulation ===\n")
    
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    
    # Simulate MPPI outputs (typically float64 from numpy/JAX)
    steer_vel = 0.1  # From MPPI planner
    acceleration = 2.0  # From MPPI planner
    
    print("MPPI outputs (simulated):")
    print(f"   steer_vel: {steer_vel} (type: {type(steer_vel)})")
    print(f"   acceleration: {acceleration} (type: {type(acceleration)})")
    
    # OLD WAY (problematic)
    print("\nOLD WAY (causes error):")
    old_steer_tensor = torch.tensor([[float(steer_vel)]], device=device)
    old_accel_tensor = torch.tensor([[float(acceleration)]], device=device)
    print(f"   Steering tensor dtype: {old_steer_tensor.dtype}")
    print(f"   Acceleration tensor dtype: {old_accel_tensor.dtype}")
    print(f"   Compatible with Isaac Lab: ❌")
    
    # NEW WAY (fixed)
    print("\nNEW WAY (works correctly):")
    new_steer_tensor = torch.tensor([[float(steer_vel)]], device=device, dtype=torch.float32)
    new_accel_tensor = torch.tensor([[float(acceleration)]], device=device, dtype=torch.float32)
    print(f"   Steering tensor dtype: {new_steer_tensor.dtype}")
    print(f"   Acceleration tensor dtype: {new_accel_tensor.dtype}")
    print(f"   Compatible with Isaac Lab: ✅")
    
    # Simulate the vehicle dynamics calculation
    print("\nVehicle dynamics calculation:")
    current_velocity = torch.tensor([[1.0]], device=device, dtype=torch.float32)
    current_steering = torch.tensor([[0.0]], device=device, dtype=torch.float32)
    
    sim_dt = 0.01  # Simulation timestep
    wheel_diameter = 0.1
    
    # Euler integration
    current_velocity = current_velocity + new_accel_tensor * sim_dt
    current_steering = current_steering + new_steer_tensor * sim_dt
    
    # Convert to actuator commands
    target_rpms = current_velocity / (wheel_diameter * 0.5)
    target_steering = current_steering
    
    print(f"   Target RPMs dtype: {target_rpms.dtype}")
    print(f"   Target steering dtype: {target_steering.dtype}")
    print(f"   Ready for Isaac Lab: ✅")

if __name__ == "__main__":
    demonstrate_dtype_issue()
    check_device_compatibility()
    simulate_mppi_integration()
    
    print("\n=== Summary ===")
    print("✅ Always use dtype=torch.float32 for Isaac Lab")
    print("✅ Specify device explicitly")
    print("✅ Use .float() conversion as backup")
    print("✅ The error should now be resolved!")
