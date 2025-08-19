# Isaac Sim MPPI Integration

This directory contains scripts for integrating Model Predictive Path Integral (MPPI) control with Isaac Sim for autonomous F1TENTH racing simulation.

## Overview

The main script `isaac_sim_mppi_integration.py` provides a complete simulation environment that:
- Loads a race track from CSV waypoints
- Implements MPPI-based trajectory planning and control
- Visualizes waypoints and trajectory in Isaac Sim
- Collects simulation data for hybrid neural ODE training

## Features

- **MPPI Controller**: Advanced model predictive control for high-speed racing
- **3D Visualization**: Real-time waypoint markers, trajectory lines, and velocity arrows
- **Data Collection**: Automatic logging of states, controls, and MPPI outputs
- **Inclined Surface Support**: Configurable ground plane angle for off-road scenarios
- **Memory Optimized**: Reduced GPU memory usage for better performance

## Requirements

- Isaac Lab
- f1tenth_planning package
- PyTorch
- NumPy
- SciPy

## Usage

### Basic Run
```bash
python isaac_sim_mppi_integration.py --num_envs 1
```

### With Custom Parameters
```bash
python isaac_sim_mppi_integration.py --num_envs 1 --device cuda:0
```

## Configuration

Key parameters can be modified in the script:

- `GROUND_PLANE_ANGLE`: Surface inclination angle (degrees)
- `MIN_VEL`, `MAX_VEL`: Velocity limits (m/s)
- `MIN_STEER`, `MAX_STEER`: Steering angle limits (radians)
- `MAX_COUNT`: Maximum simulation steps

## Visualization

The script creates three types of visual markers:

1. **Green Spheres**: Waypoint positions
2. **Red Line**: Trajectory path
3. **Blue Cylinders**: Velocity direction arrows

Marker sizes can be adjusted by modifying the `marker_scale` parameters in the visualization function calls.

## Data Output

Simulation data is automatically saved as `isaac_sim_mppi_data.npz` containing:
- `timestamps`: Simulation time steps
- `states`: Robot state variables (position, velocity, orientation)
- `inputs`: Control inputs (acceleration, steering rate)
- `mppi_outputs`: MPPI-specific control commands
- `ground_plane_angle`: Surface inclination
- `rotation_matrix`: Coordinate transformation matrix

## File Structure

```
test/
├── README.md                           # This file
├── isaac_sim_mppi_integration.py      # Main simulation script
├── trajectory_log.csv                 # Race track waypoints (required)
└── isaac_sim_mppi_data.npz           # Output data (generated)
```

## Troubleshooting

### Common Issues

1. **Missing trajectory_log.csv**: Ensure the CSV file with waypoints is present
2. **MPPI planning failures**: Check f1tenth_planning package installation
3. **GPU memory errors**: Reduce `num_envs` or enable memory optimization flags

### Debug Mode

For debugging, you can:
- Add print statements to track MPPI state
- Reduce `MAX_COUNT` for shorter runs
- Monitor console output for visualization warnings

## Notes

- The script is optimized for single environment (`num_envs=1`) usage
- Waypoint visualization uses efficient sampling to avoid clutter
- Data is collected in the transformed coordinate frame for inclined surfaces
- The simulation automatically resets after `MAX_COUNT` steps

## Related Files

- `../mushr.py`: Vehicle configuration
- `../../examples/control/`: MPPI controller examples
- Data processing scripts for hybrid neural ODE training
