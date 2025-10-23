# Smart-Mobile-Manipulator
This project demonstrates an integrated mobile manipulation system combining autonomous navigation and manipulation using the Artificial Potential Field (APF) method. The simulation is implemented in PyBullet and showcases real-time obstacle avoidance, color-based sorting, and task prioritization.
## Overview

This project implements a **mobile manipulator system** using PyBullet physics simulation. The system combines:
- **Husky mobile base**: A four-wheeled differential drive robot for navigation
- **KUKA iiwa robotic arm**: A 7-DOF manipulator for pick-and-place operations
- **Intelligent obstacle avoidance**: Using potential field method
- **Path tracking**: Ensuring the robot follows optimal trajectories
- **Color-based sorting**: Picking colored cubes and placing them in matching bins

### Main Features
✅ Real-time obstacle avoidance using repulsive force fields  
✅ Optimal path tracking with deviation correction  
✅ Fast inverse kinematics for arm control  
✅ Strategic delays for physics stabilization  
✅ Distance-based task prioritization  
✅ Robust gripper constraint system  

### Working Video
[Watch the working video](./video_project.mp4)

##  System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                  MOBILE MANIPULATOR                     │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌──────────────┐         ┌─────────────────────────┐ │
│  │ HUSKY BASE   │◄────────┤  KUKA ARM (7-DOF)      │ │
│  │              │ Fixed   │  - IK Control          │ │
│  │ - 4 Wheels   │ Joint   │  - Gripper Constraint  │ │
│  │ - Diff Drive │         │  - Home Position       │ │
│  └──────────────┘         └─────────────────────────┘ │
│         ▲                            ▲                 │
│         │                            │                 │
│         ▼                            ▼                 │
│  ┌──────────────────────────────────────────────────┐ │
│  │          CONTROL SYSTEM                          │ │
│  │  - Obstacle Avoidance (Potential Fields)        │ │
│  │  - Path Tracking (Deviation Correction)         │ │
│  │  - Task Planning (Distance-based Priority)      │ │
│  └──────────────────────────────────────────────────┘ │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

---

## Dependencies & Setup

### Required Libraries
```python
import pybullet as p              # Physics simulation
import pybullet_data              # URDF models
import time                       # Timing control
import random                     # Random cube placement
import math                       # Mathematical operations
import numpy as np                # Numerical computations
```

### Installation
```bash
pip install pybullet numpy
```

### Running the Simulation
```bash
python mobile_manipulator.py
```

---

## Key Components

### 1. **Husky Mobile Base**
- **Type**: Four-wheeled differential drive robot
- **Location**: Initially at (1.0, 2.0, 0.1)
- **Wheels**: 
  - Left wheels: Joints [2, 4]
  - Right wheels: Joints [3, 5]
- **Control**: Velocity control on individual wheels

### 2. **KUKA iiwa Arm**
- **DOF**: 7 joints
- **Mounting**: Fixed constraint to Husky base at height 0.65m
- **End Effector**: Joint index 6
- **Home Position**: `[0, -0.5, 0, -1.5, 0, 1.5, 0]`
- **Control Method**: Position control with IK

### 3. **Environment Objects**

#### Colored Cubes
- **Size**: 0.1m × 0.1m × 0.1m
- **Mass**: 0.05 kg
- **Colors**: Red, Yellow, Green
- **Placement**: Random positions in workspace (2.5-4.0, 0.5-3.5)
- **Minimum Separation**: 0.5m between cubes

#### Sorting Bins
- **Size**: 0.15m × 0.15m × 0.1m
- **Locations**: 
  - Red bin: (5.0, 1.0)
  - Yellow bin: (5.0, 2.0)
  - Green bin: (5.0, 3.0)
- **Structure**: Four walls (0.02m thick) creating open-top containers

---

## Obstacle Avoidance Algorithm

The system uses an **Artificial Potential Field (APF)** method for obstacle avoidance. This creates virtual forces that guide the robot toward the goal while repelling it from obstacles.

### Core Concept

```
Total Force = Attractive Force(pulls toward goal) + Repulsive Forces(pushes away from obstacles)
```
**Parameters:**
- `ATTRACTION_GAIN = 8.0`: Controls how strongly robot is pulled to goal
- `target_distance = 0.77m`: Desired stopping distance from target
- `REPULSION_GAIN = 3.0`: Controls avoidance strength
- `INFLUENCE_DISTANCE = 0.7m`: Range of obstacle influence



### Obstacle Detection
### Visual Example

```
      Goal ★
        ↑
        │ F_attr (Attractive)
        │
    ┌───●───┐  Robot
    │       │
F_rep│←──────│  ← F_rep (Repulsive from obstacles)
    │       │
    └───────┘
        
    ⬤ ⬤     Obstacles
```

**Result**: Robot navigates around obstacles while moving toward goal.

---

## Path Tracking Mechanism

Path tracking ensures the robot follows a **straight-line path** from start to goal, correcting deviations caused by obstacle avoidance.

**Geometric Interpretation:**
```
Start ●─────────────────────────● Goal
      │                         │
      │  Ideal Path (straight)  │
      │                         │
      │        ⬤ Robot          │
      │       / ↑               │
      │      /  │ deviation     │
      │     /   │               │
      └────/────┘               
          /
    Actual path (curved due to obstacles)
```

#### 3. **Correction Force**
Apply a restoring force proportional to deviation.

**Parameters:**
- `PATH_TRACKING_GAIN = 6.0`: Controls correction strength
  - Higher value: Stronger correction, straighter path, but may oscillate
  - Lower value: Gentler correction, smoother but more deviation

#### 4. **How It Works - Step by Step**

**Scenario:** Robot needs to reach goal but obstacle is in the way

```
Step 1: Initial Path
Start ●─────────────────● Goal
           ⬤ Obstacle

Step 2: Obstacle Avoidance Pushes Robot Off Path
Start ●────┐            ● Goal
           └──⬤ Robot
           ⬤ Obstacle
           (deviation = +0.3m)

Step 3: Path Tracking Pulls Robot Back
Start ●────┐            ● Goal
           └──→ ⬤ Robot (correction force ←)
           ⬤ Obstacle

Step 4: Balanced Navigation
Start ●────╱──────────● Goal
          ╱  ⬤ Robot
         ╱
    ⬤ Obstacle (avoided)
    
Final: Smooth arc around obstacle, returning to path
```

### Benefits of Path Tracking

1. **Efficiency**: Minimizes total distance traveled
2. **Predictability**: Robot follows more consistent paths
3. **Recovery**: After avoiding obstacles, robot returns to optimal path
4. **Metrics**: Can measure and report maximum deviation


### Task Prioritization Strategy

**Why distance-based?**
1. **Efficiency**: Minimizes total travel distance
2. **Energy**: Less battery/fuel consumption
3. **Time**: Faster completion
4. **Practical**: Mimics human behavior

**Example Scenario:**
```
Robot at (1.0, 2.0)
Red cube:    (2.5, 1.5) → distance = 1.58m → Priority 1
Yellow cube: (3.8, 2.8) → distance = 2.92m → Priority 2
Green cube:  (3.2, 3.5) → distance = 2.65m → Priority 3 (after sorting)

Execution order: Red → Green → Yellow
```

### Detailed Task Flow

```
For each cube in priority order:
│
├─→ [NAVIGATION PHASE]
│   ├─ Calculate attractive force to cube
│   ├─ Calculate repulsive forces from other cubes
│   ├─ Apply path tracking correction (if enabled)
│   ├─ Control wheels for differential drive
│   └─ Repeat until within 0.65m of cube
│
├─→ [PICK PHASE]
│   ├─ Move arm to home position
│   ├─ Open gripper
│   ├─ Stabilize physics (10ms)
│   ├─ Approach: Move above cube (z + 0.5m)
│   ├─ Lower: Descend to grasp height (z + 0.2m)
│   ├─ Close gripper (create constraint)
│   ├─ Stabilize physics (10ms)
│   └─ Lift: Raise cube (z + 0.35m)
│
├─→ [TRANSPORT PHASE]
│   ├─ Calculate attractive force to bin
│   ├─ Calculate repulsive forces from remaining cubes
│   ├─ Apply path tracking correction (if enabled)
│   ├─ Control wheels for differential drive
│   └─ Repeat until within 0.75m of bin
│
└─→ [PLACE PHASE]
    ├─ Stabilize physics (10ms)
    ├─ Approach: Move above bin (z + 0.20m)
    ├─ Lower: Descend into bin (z + 0.1m)
    ├─ Release gripper (remove constraint)
    ├─ Stabilize physics (10ms)
    └─ Retract: Move up (z + 0.25m)
```

---

## 🚀 Advanced Features

### 1. Dynamic Obstacle List
- Cubes are only obstacles when NOT being approached
- Picked cubes are removed from obstacle list
- Prevents robot from avoiding the cube it needs to reach

### 2. Differential Drive Kinematics
The Husky uses differential drive, where motion is controlled by wheel velocities:

### 3. IK Solution Selection
Selects the IK solution that brings end-effector closest to target.

### 4. Constraint-Based Grasping
Instead of complex gripper finger simulation, the system uses a fixed constraint:
This "welds" the object to the end-effector, providing:
- Perfect grip (no slipping)
- No complex contact physics
- Computational efficiency

### 5. Physics Stabilization
Strategic delays at critical moments:
- **At pick location**: Prevents cube from sliding
- **After grasping**: Ensures constraint is stable
- **At bin location**: Prepares for precise placement
- **After dropping**: Allows cube to settle in bin

### 6. Velocity Clamping
Prevents unrealistic velocities and ensures stable motion.

### 7. Settling Detection
Requires multiple consecutive frames within tolerance before stopping, preventing premature stopping due to noise.

## Performance Metrics

The system can track:
- **Total distance traveled**: Sum of all navigation segments
- **Maximum path deviation**: How far from straight line
- **Task completion time**: Per cube and total
- **Closest obstacle approach**: Minimum distance during navigation
- **IK success rate**: Percentage of successful arm motions

---

## Troubleshooting

### Robot doesn't move
- Check wheel velocities are non-zero
- Verify friction is not too high
- Ensure simulation is stepping

### Arm doesn't reach target
- Check target is within MAX_ARM_REACH (0.82m)
- Verify no joint limits are violated
- Increase maxNumIterations for IK

### Cube slips from gripper
- Increase constraint maxForce 
- Add delays before/after grasping
- Check cube mass is not too high

### Robot oscillates near target
- Decrease ANGULAR_GAIN
- Increase SETTLE_THRESHOLD
- Reduce MAX_LINEAR_SPEED

### Robot collides with obstacles
- Increase REPULSION_GAIN
- Increase INFLUENCE_DISTANCE
- Decrease MAX_LINEAR_SPEED near obstacles

---

##  Future Enhancements

Possible improvements:
1. **Real-time obstacle detection** using sensors
2. **Obstacle Detection** for zero shot learning 
3. **Multi-robot coordination**
4. **Learning-based navigation** using RL          
5. **A* or RRT** for global path planning
6. **Real gripper simulation** with finger forces
7. **Visual servoing** for precise object grasping 

---

## License & Credits

This implementation uses:
- **PyBullet**: Physics simulation
- **Husky URDF**: Clearpath Robotics
- **KUKA iiwa URDF**: KUKA Robotics

---

## Support

For questions or issues, refer to:
- [PyBullet Documentation](https://pybullet.org)
- [KUKA iiwa Specifications](https://www.kuka.com)
- [Artificial Potential Fields Paper](https://ieeexplore.ieee.org/document/10115857)
