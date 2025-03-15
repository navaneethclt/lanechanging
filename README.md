# lanechanging
Trajectory generation for a lane changing 
# Optimized Trajectory Planning in MATLAB

## Overview
This MATLAB project implements an optimization-based trajectory planning algorithm to determine the optimal time required for a vehicle to transition between lanes while minimizing acceleration and jerk.

## Features
- Uses numerical optimization (`fminunc`) to find the optimal transition time.
- Computes and visualizes the trajectory of the vehicle.
- Plots velocity, curvature, and acceleration profiles.
- Implements a cost function considering acceleration, time, and jerk penalties.

## Dependencies
- MATLAB with Optimization Toolbox

## Usage
1. Run the script in MATLAB.
2. The optimization process finds the optimal transition time.
3. The script generates plots for:
   - Optimized trajectory
   - Velocity profile
   - Curvature profile
   - Acceleration profile

## Parameters
- **Lateral displacement (W):** 3.5 meters
- **Initial velocity (u):** 30 mph converted to m/s
- **Acceleration constraints (ax, ay):** Set values for acceleration components
- **Cost function weights:** `lambda1`, `lambda2`, `lambda3`
- **Yaw angle error (e1):** -5 degrees converted to radians

## Functions
- `cost_function.m`: Defines the cost function for optimization.
- `generate_trajectory.m`: Generates the optimized vehicle trajectory based on computed parameters.

## Output
- Optimized time and cost values displayed in the command window.
- Graphs depicting the vehicle’s motion characteristics.

