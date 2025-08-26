# SCARA Robot Motion Analysis and Simulation

A comprehensive MATLAB analysis of SCARA (Selective Compliance Assembly Robot Arm) robot kinematics, workspace, and trajectory planning using the EPSON G3-250 as reference model.

## Overview

This project implements forward/inverse kinematics, workspace analysis, and trajectory planning for an RRPR SCARA robot with focus on pick-and-place operations.

## Robot Specifications

**EPSON G3-250 SCARA Robot**
- Arm lengths: 120mm (J1) + 130mm (J2)
- Vertical stroke: 150mm (J3)
- Payload: 1kg nominal, 3kg maximum
- Repeatability: ±0.008mm (J1/J2), ±0.010mm (J3), ±0.005° (J4)
- Max speeds: 3350mm/s (tip), 1100mm/s (Z-axis), 3000°/s (rotation)

## Features

### Kinematics Analysis
- **Forward Kinematics**: Denavit-Hartenberg implementation
- **Inverse Kinematics**: Three methods comparison:
  - Analytical (closed-form solution)
  - Iterative (numerical optimization)
  - Point-and-direction (geometric approach)

### Workspace Analysis
- 3D workspace visualization
- Hollow cylinder characteristic shape
- Multi-axis projections (XY, XZ, YZ planes)
- Systematic configuration space sampling

### Trajectory Planning
- Pick-and-place task simulation
- Triangular/trapezoidal velocity profiles
- Minimum acceleration calculation
- 180° rotation capability

## Task Specification

**Pick-and-Place Operation:**
- Pickup: [0.1, 0.1, -0.11] m
- Drop-off: [-0.08, 0.12, -0.18] m
- Object: Cylindrical (250g, Ø20mm, h=30mm)
- End-effector: Mechanical gripper (500g, 100mm)
- Execution time: 1.5 seconds
- Rotation: 180° around Z-axis

## Repository Structure

```
├── forward_kinematics.m          # DH-based forward kinematics
├── inverse_kinematics/
│   ├── analytical_method.m       # Closed-form solution
│   ├── iterative_method.m        # Numerical optimization
│   └── point_direction_method.m  # Geometric approach
├── workspace_analysis.m          # 3D workspace generation
├── trajectory_planner.m          # Motion planning functions
├── scara_simulation.m            # Main simulation script
└── robotics_toolbox/             # Peter Corke toolbox examples
```

## Key Functions

- `cinematicaDiretta(q)` - Forward kinematics computation
- `cinematicainv(T04)` - Inverse kinematics solver
- `trajectory_planner(qi,qf,ti,tf,v_max)` - Optimal trajectory generation
- `DH(a,d,alfa,teta)` - Denavit-Hartenberg transformation matrix

## Results

- **Feasibility**: Task confirmed within robot capabilities (0.75kg < 1kg nominal payload)
- **Trajectory**: Successfully completed in 1.5s using triangular profiles
- **Accelerations**: J1: 2.469 rad/s², J2: -0.0487 rad/s², J3: 0.124 m/s², J4: 8.005 rad/s²
- **Method Comparison**: Point-and-direction offers best complexity/performance trade-off

## Limitations

- Kinematic analysis only (no dynamics)
- Missing torque/inertia data from manufacturer
- Simplified gripper model
- No collision detection
- Limited to two-point trajectories

## Requirements

- MATLAB R2019b or later
- Robotics Toolbox for MATLAB (Peter Corke) - optional for visualization
- Optimization Toolbox (for `fsolve`)

## Usage

```matlab
% Basic forward kinematics
q = [0, 0, 0.05, 0]; % Joint configuration
T = cinematicaDiretta(q);

% Inverse kinematics
T_desired = eye(4); % Target pose
q_solution = cinematicainv(T_desired);

% Trajectory planning
[a_min, pos, vel, acc, time] = trajectory_planner(q_start, q_end, 0, 1.5, v_max);
```

## Authors

**Master's in Automation Engineering - Robotics**  
*Applied Mechanics – Functional Design Course (2023/2024)*

- Donato Salamida
- Vitantonio Creanza  
- Giuseppe Lepore

**Supervisor:** Prof. Mario Massimo Foglia

## References

- Siciliano B., Sciavicco L., Villani L., Oriolo G. - *Robotics: Modelling, Planning and Control*, Springer, 2009
- Seiko Epson Corporation - *Epson G3 SCARA Robots Product Manual*
- Peter Corke - *Robotics Toolbox for MATLAB*

## License

Academic project for educational purposes.
