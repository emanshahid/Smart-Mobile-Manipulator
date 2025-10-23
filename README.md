# Smart-Mobile-Manipulator
# Mobile Manipulator with Obstacle Avoidance - Complete Documentation

## 📋 Table of Contents
1. [Overview](#overview)
2. [System Architecture](#system-architecture)
3. [Dependencies & Setup](#dependencies--setup)
4. [Key Components](#key-components)
5. [Obstacle Avoidance Algorithm](#obstacle-avoidance-algorithm)
6. [Path Tracking Mechanism](#path-tracking-mechanism)
7. [Function Reference](#function-reference)
8. [Configuration Parameters](#configuration-parameters)
9. [Workflow](#workflow)
10. [Advanced Features](#advanced-features)

---

## 🤖 Overview

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

---

## 🏗️ System Architecture

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

## 📦 Dependencies & Setup

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

## 🧩 Key Components

### 1. **Husky Mobile Base**
- **Type**: Four-wheeled differential drive robot
- **Location**: Initially at (1.0, 2.0, 0.1)
- **Wheels**: 
  - Left wheels: Joints [2, 4]
  - Right wheels: Joints [3, 5]
- **Control**: Velocity control on individual wheels
- **Dynamics**: 
  - Mass: 70 kg
  - High friction (100.0) for stability
  - Damping for smooth motion

### 2. **KUKA iiwa Arm**
- **DOF**: 7 joints
- **Mounting**: Fixed constraint to Husky base at height 0.65m
- **Orientation**: Rotated 180° around Z-axis
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

## 🎯 Obstacle Avoidance Algorithm

The system uses an **Artificial Potential Field (APF)** method for obstacle avoidance. This creates virtual forces that guide the robot toward the goal while repelling it from obstacles.

### Core Concept

```
Total Force = Attractive Force + Repulsive Forces
```

### Mathematical Formulation

#### 1. **Attractive Force** (Pulls toward goal)
```python
# Distance to goal
dx_goal = target_x - robot_x
dy_goal = target_y - robot_y
distance_to_goal = √(dx_goal² + dy_goal²)

# Distance error (how far from desired stopping distance)
distance_error = distance_to_goal - target_distance

# Attractive force (proportional to error)
F_attr_x = ATTRACTION_GAIN × (dx_goal / distance_to_goal) × distance_error
F_attr_y = ATTRACTION_GAIN × (dy_goal / distance_to_goal) × distance_error
```

**Parameters:**
- `ATTRACTION_GAIN = 8.0`: Controls how strongly robot is pulled to goal
- `target_distance = 0.77m`: Desired stopping distance from target

#### 2. **Repulsive Force** (Pushes away from obstacles)
```python
# For each obstacle
dx = robot_x - obstacle_x
dy = robot_y - obstacle_y
distance = √(dx² + dy²)

# Only applies if within influence distance
if distance < INFLUENCE_DISTANCE:
    # Force magnitude (inverse square law)
    magnitude = REPULSION_GAIN × (1/distance - 1/INFLUENCE_DISTANCE) × (1/distance²)
    
    # Force direction (away from obstacle)
    F_rep_x = (dx / distance) × magnitude
    F_rep_y = (dy / distance) × magnitude
```

**Parameters:**
- `REPULSION_GAIN = 3.0`: Controls avoidance strength
- `INFLUENCE_DISTANCE = 0.7m`: Range of obstacle influence

#### 3. **Combined Force & Motion**
```python
# Sum all forces
total_force_x = F_attr_x + Σ(F_rep_x_i)
total_force_y = F_attr_y + Σ(F_rep_y_i)

# Calculate desired heading
desired_yaw = atan2(total_force_y, total_force_x)
yaw_error = desired_yaw - current_yaw

# Control strategy
if |yaw_error| > 12°:
    # Pure rotation
    left_wheel_vel = -ANGULAR_GAIN × yaw_error
    right_wheel_vel = +ANGULAR_GAIN × yaw_error
else:
    # Forward motion with steering
    linear_speed = min(force_magnitude, MAX_LINEAR_SPEED)
    left_wheel_vel = linear_speed - ANGULAR_GAIN × yaw_error
    right_wheel_vel = linear_speed + ANGULAR_GAIN × yaw_error
```

### Obstacle Detection
```python
def get_obstacle_positions(exclude_cube_id=None):
    """
    Returns positions of all cubes except:
    - The target cube being approached
    - Cubes currently held by gripper
    """
    obstacles = []
    for cube_id, info in cubes.items():
        if cube_id != exclude_cube_id and not info['picked']:
            pos = get_position(cube_id)
            obstacles.append(pos[:2])  # Only X, Y coordinates
    return obstacles
```

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

## 🛤️ Path Tracking Mechanism

Path tracking ensures the robot follows a **straight-line path** from start to goal, correcting deviations caused by obstacle avoidance.

### Implementation (In Commented Version)

#### 1. **Path Definition**
At the start of motion, the system defines the reference path:

```python
# Starting position
start_x, start_y = robot_position

# Calculate path vector
path_dx = target_x - start_x
path_dy = target_y - start_y
path_length = √(path_dx² + path_dy²)

# Path unit vector (direction of straight line)
path_unit_x = path_dx / path_length
path_unit_y = path_dy / path_length

# Perpendicular vector (for deviation measurement)
perp_unit_x = -path_unit_y
perp_unit_y = path_unit_x
```

#### 2. **Deviation Calculation**
At each timestep, measure how far the robot has deviated from the straight path:

```python
# Current displacement from start
current_dx = robot_x - start_x
current_dy = robot_y - start_y

# Project displacement onto perpendicular axis
# This gives the SIGNED perpendicular distance from path
deviation = current_dx × perp_unit_x + current_dy × perp_unit_y

# Track maximum deviation
max_deviation = max(max_deviation, |deviation|)
```

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
Apply a restoring force proportional to deviation:

```python
# Path correction force (pulls robot back to path)
path_correction_x = -PATH_TRACKING_GAIN × deviation × perp_unit_x
path_correction_y = -PATH_TRACKING_GAIN × deviation × perp_unit_y

# Add to total force
total_force_x = attr_force_x + rep_force_x + path_correction_x
total_force_y = attr_force_y + rep_force_y + path_correction_y
```

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

### Why Two Versions?

**Active Version** (lines 302-389):
- Pure potential field method
- Simpler, faster computation
- Suitable for simple environments

**Commented Version** (lines 194-301):
- Includes path tracking correction
- Better for complex environments with multiple obstacles
- Provides deviation metrics
- More sophisticated navigation

---

## 📖 Function Reference

### Initialization Functions

#### `initialize_simulation()`
Sets up the PyBullet environment.

**Actions:**
- Connects to GUI
- Loads PyBullet data path
- Sets gravity to -9.8 m/s²
- Sets timestep to 1/240 seconds (240 Hz)
- Configures camera view

#### `create_environment()`
Creates the workspace with obstacles and targets.

**Returns:** Dictionary of cubes with their properties

**Creates:**
- Ground plane with high friction
- 3 colored bins (red, yellow, green) at x=5.0
- 3 colored cubes at random positions
- Ensures minimum 0.5m separation between cubes

#### `setup_mobile_base()`
Loads and configures the Husky robot.

**Configuration:**
- Initial position: (1.0, 2.0, 0.1)
- Wheel friction: 10.0
- Base mass: 70 kg
- Damping for stability

**Returns:** Husky body ID

#### `setup_arm()`
Loads KUKA arm and attaches it to Husky.

**Configuration:**
- Position: (1.0, 2.0, 0.65) - on top of Husky
- Orientation: 180° rotation around Z-axis
- Fixed constraint to Husky base
- Initial joint configuration: home position

**Returns:** KUKA body ID

---

### Navigation Functions

#### `move_toward_target_with_avoidance(target_pos, target_cube_id, target_distance)`
Main navigation function using potential field method.

**Parameters:**
- `target_pos`: (x, y, z) coordinates of destination
- `target_cube_id`: ID of cube being approached (excluded from obstacles)
- `target_distance`: Desired stopping distance from target

**Algorithm:**
1. Calculate attractive force to goal
2. Calculate repulsive forces from all obstacles
3. Combine forces to get desired heading
4. Control wheel velocities for differential drive
5. Repeat until within tolerance

**Control Parameters:**
```python
DISTANCE_TOLERANCE = 0.05      # Stop when within 5cm
MAX_LINEAR_SPEED = 40.0        # Maximum speed
ATTRACTION_GAIN = 8.0          # Goal attraction strength
REPULSION_GAIN = 3.0           # Obstacle repulsion strength
INFLUENCE_DISTANCE = 0.7       # Obstacle effect range
ANGULAR_GAIN = 4.0             # Turning responsiveness
SETTLE_THRESHOLD = 5           # Frames to confirm arrival
```

**Returns:** Implicitly returns when target reached

#### `set_wheel_velocities(left_vel, right_vel)`
Low-level wheel control for differential drive.

**Parameters:**
- `left_vel`: Velocity for left wheels [2, 4]
- `right_vel`: Velocity for right wheels [3, 5]

**Physics:**
```
Forward motion:   left_vel = right_vel = v
Turn right:       left_vel > right_vel
Turn left:        left_vel < right_vel
Rotate in place:  left_vel = -right_vel
```

#### `stop_base()`
Immediately stops all wheel motion by setting velocities to zero.

---

### Manipulation Functions

#### `pick_object_fast(object_id)`
Performs fast pick operation with strategic delays.

**Sequence:**
1. Move arm to home position
2. Open gripper
3. **DELAY 1**: Stabilize at pick location (0.01s)
4. Approach: Move to position above cube (z + 0.5m)
5. Lower: Descend to grasp height (z + 0.2m)
6. Close gripper and create constraint
7. **DELAY 2**: Secure grip (0.01s)
8. Lift: Raise cube (z + 0.35m)

**Physics Stabilization:**
The delays allow the physics engine to settle, preventing:
- Cube sliding during approach
- Gripper missing due to momentum
- Constraint failures

**Returns:** `True` on success

#### `place_object_fast(target_pos)`
Performs fast place operation with strategic delays.

**Sequence:**
1. **DELAY 3**: Stabilize at bin location (0.01s)
2. Approach: Move above bin (z + 0.20m)
3. Lower: Descend into bin (z + 0.1m)
4. Release gripper constraint
5. **DELAY 4**: Let cube settle (0.01s)
6. Retract: Move up (z + 0.25m)

**Returns:** `True` on success

#### `move_arm_fast(target_pos, target_orn, max_steps)`
Moves arm to target position using inverse kinematics.

**Parameters:**
- `target_pos`: Desired end-effector position (x, y, z)
- `target_orn`: Desired orientation (quaternion), default: downward facing
- `max_steps`: Maximum simulation steps (default: 1000)

**Algorithm:**
1. Stop mobile base (arm motion requires stable platform)
2. Calculate inverse kinematics for target pose
3. Test IK solution and select best (lowest error)
4. Apply joint position commands with high force (800N) and velocity (5.0 rad/s)
5. Step simulation until convergence (distance < 0.025m)

**IK Solver Configuration:**
```python
maxNumIterations = 100
residualThreshold = 1e-5
```

**Convergence Criterion:** End-effector within 2.5cm of target

**Returns:** `True` when target reached or max steps exceeded

#### `move_arm_to_home()`
Instantly resets arm to home configuration.

**Home Position:**
```python
[0, -0.5, 0, -1.5, 0, 1.5, 0]  # radians for joints 0-6
```

This configuration:
- Avoids singularities
- Keeps arm within workspace
- Positions end-effector forward
- Safe starting position for any task

---

### Gripper Functions

#### `open_gripper()`
Opens gripper (virtual) by setting state flag.
```python
gripper_state['is_closed'] = False
```

#### `close_gripper(target_object_id)`
Closes gripper and creates fixed constraint to object.

**Constraint Details:**
```python
constraint = createConstraint(
    parentBodyID = kuka,
    parentLinkIndex = END_EFFECTOR_INDEX (6),
    childBodyID = target_object_id,
    childLinkIndex = -1,
    jointType = FIXED,
    maxForce = 500
)
```

This "welds" the object to the end-effector, simulating a strong grip.

#### `release_gripper()`
Releases object by removing constraint.
```python
removeConstraint(gripper_state['constraint'])
```

---

### Utility Functions

#### `get_obstacle_positions(exclude_cube_id)`
Returns positions of all cubes that should be avoided.

**Filters out:**
- Target cube being approached (`exclude_cube_id`)
- Cubes currently held by gripper (`info['picked'] == True`)

**Returns:** List of (x, y) coordinates

#### `calculate_repulsive_force(robot_pos, obstacle_pos, influence_distance, repulsion_gain)`
Calculates repulsive force from a single obstacle using inverse-square law.

**Formula:**
```python
distance = ||robot_pos - obstacle_pos||

if distance > influence_distance:
    return (0, 0)  # No effect

magnitude = repulsion_gain × (1/d - 1/d₀) × (1/d²)
force_x = (dx / distance) × magnitude
force_y = (dy / distance) × magnitude
```

Where:
- `d` = current distance
- `d₀` = influence_distance
- `dx, dy` = direction vector (robot - obstacle)

**Returns:** (force_x, force_y) tuple

#### `stabilize_physics()`
Runs simulation for 0.1 seconds without robot commands.

**Purpose:**
- Let objects settle after motion
- Allow constraints to stabilize
- Prevent physics glitches

**Implementation:**
```python
steps = int(0.1 * 240)  # 24 steps at 240 Hz
for _ in range(steps):
    p.stepSimulation()
    time.sleep(1./240.)
```

#### `get_joint_positions()`
Returns current joint angles for all 7 arm joints.

**Returns:** List of 7 angles in radians

#### `print_environment_info()`
Prints debug information about robot and object positions.

---

## ⚙️ Configuration Parameters

### Simulation Settings
```python
GRAVITY = -9.8                    # m/s²
TIME_STEP = 1/240                 # seconds (240 Hz)
CAMERA_DISTANCE = 6.0             # meters
CAMERA_YAW = 45                   # degrees
CAMERA_PITCH = -35                # degrees
```

### Arm Constraints
```python
MAX_ARM_REACH = 0.82              # Maximum reachable distance
COMFORTABLE_REACH = 0.50          # Preferred working distance
IK_TOLERANCE = 0.08               # Acceptable IK error
END_EFFECTOR_INDEX = 6            # Link index for gripper
```

### Joint Limits (KUKA iiwa)
```python
JOINT_LIMITS = [
    (-2.967, 2.967),  # Joint 0: ±170°
    (-2.094, 2.094),  # Joint 1: ±120°
    (-2.967, 2.967),  # Joint 2: ±170°
    (-2.094, 2.094),  # Joint 3: ±120°
    (-2.967, 2.967),  # Joint 4: ±170°
    (-2.094, 2.094),  # Joint 5: ±120°
    (-3.054, 3.054),  # Joint 6: ±175°
]
JOINT_LIMIT_MARGIN = 0.15         # Safety margin
```

### Strategic Delays
```python
DELAY_AT_PICK_LOCATION = 0.01    # Pause at cube (10ms)
DELAY_AFTER_GRASP = 0.01         # Pause after picking (10ms)
DELAY_AT_BIN_LOCATION = 0.01     # Pause at bin (10ms)
DELAY_AFTER_DROP = 0.01          # Pause after placing (10ms)
```

### Navigation Parameters (Active Version)
```python
DISTANCE_TOLERANCE = 0.05         # Goal reached threshold (5cm)
MAX_LINEAR_SPEED = 40.0           # Maximum speed (m/s)
ATTRACTION_GAIN = 8.0             # Goal attraction strength
REPULSION_GAIN = 3.0              # Obstacle avoidance strength
INFLUENCE_DISTANCE = 0.7          # Obstacle effect range (m)
ANGULAR_GAIN = 4.0                # Turning responsiveness
SETTLE_THRESHOLD = 5              # Frames to confirm arrival
```

### Navigation Parameters (Path Tracking Version - Commented)
```python
DISTANCE_TOLERANCE = 0.04         # Goal reached threshold (4cm)
MAX_LINEAR_SPEED = 60.0           # Maximum speed (m/s)
ATTRACTION_GAIN = 10.0            # Goal attraction strength
REPULSION_GAIN = 3.5              # Obstacle avoidance strength
INFLUENCE_DISTANCE = 0.6          # Obstacle effect range (m)
PATH_TRACKING_GAIN = 6.0          # Path deviation correction
ANGULAR_GAIN = 5.0                # Turning responsiveness
SETTLE_THRESHOLD = 2              # Frames to confirm arrival
```

### Wheel Configuration
```python
LEFT_WHEELS = [2, 4]              # Joint indices
RIGHT_WHEELS = [3, 5]             # Joint indices
MAX_WHEEL_VELOCITY = 40.0         # rad/s
WHEEL_FRICTION = 10.0             # Lateral friction
```

### Object Properties
```python
CUBE_SIZE = 0.1                   # meters (10cm)
CUBE_MASS = 0.05                  # kg (50 grams)
BIN_SIZE = [0.15, 0.15, 0.1]      # meters
WALL_THICKNESS = 0.02             # meters
```

---

## 🔄 Workflow

### Main Execution Flow

```python
def main():
    # 1. INITIALIZATION
    initialize_simulation()
    create_environment()
    setup_mobile_base()
    setup_arm()
    move_arm_to_home()
    
    # 2. TASK PLANNING
    # Get robot position
    robot_pos = get_position(husky)
    
    # Calculate distances to all cubes
    cube_distances = []
    for cube_id, info in cubes.items():
        cube_pos = get_position(cube_id)
        distance = euclidean_distance(robot_pos, cube_pos)
        cube_distances.append((cube_id, info['color'], cube_pos, distance))
    
    # Sort by distance (nearest first)
    cube_distances.sort(key=lambda x: x[3])
    
    # 3. TASK EXECUTION LOOP
    for cube_id, color, cube_pos, distance in cube_distances:
        print(f"Task: {color} cube at {distance:.2f}m")
        
        # Navigate to cube
        move_toward_target_with_avoidance(cube_pos, cube_id, 0.65)
        
        # Pick cube
        pick_object_fast(cube_id)
        
        # Navigate to matching bin
        bin_pos = get_bin_position(color)
        move_toward_target_with_avoidance(bin_pos, None, 0.75)
        
        # Place cube
        place_object_fast(bin_pos)
    
    # 4. COMPLETION
    print("All tasks completed!")
    
    # Keep simulation running
    while True:
        p.stepSimulation()
```

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
```python
def get_obstacle_positions(exclude_cube_id=None):
```
- Cubes are only obstacles when NOT being approached
- Picked cubes are removed from obstacle list
- Prevents robot from avoiding the cube it needs to reach

### 2. Differential Drive Kinematics
The Husky uses differential drive, where motion is controlled by wheel velocities:

**Forward Motion:**
```python
left_vel = right_vel = v
```

**Turning (arc):**
```python
turning_radius = wheelbase / (2 × tan(steering_angle))
left_vel = v - ω × wheelbase/2
right_vel = v + ω × wheelbase/2
```

**Rotation in place:**
```python
left_vel = -ω
right_vel = +ω
```

### 3. IK Solution Selection
```python
# Test multiple IK solutions
for _ in range(1):
    joint_poses = calculateInverseKinematics(...)
    error = distance(actual_ee_pos, target_pos)
    if error < best_error:
        best_joint_poses = joint_poses
```
Selects the IK solution that brings end-effector closest to target.

### 4. Constraint-Based Grasping
Instead of complex gripper finger simulation, the system uses a fixed constraint:
```python
constraint = createConstraint(
    kuka, END_EFFECTOR_INDEX,
    cube_id, -1,
    JOINT_FIXED
)
```
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
```python
max_vel = 40.0
left_vel = max(-max_vel, min(max_vel, left_vel))
right_vel = max(-max_vel, min(max_vel, right_vel))
```
Prevents unrealistic velocities and ensures stable motion.

### 7. Settling Detection
```python
settled_count = 0
if within_tolerance:
    settled_count += 1
    if settled_count >= SETTLE_THRESHOLD:
        stop()
```
Requires multiple consecutive frames within tolerance before stopping, preventing premature stopping due to noise.

---

## 🔧 Tuning Guide

### For Faster Navigation
```python
MAX_LINEAR_SPEED = 60.0          # ↑ Increase
ANGULAR_GAIN = 6.0               # ↑ Increase
```

### For Smoother Paths
```python
PATH_TRACKING_GAIN = 8.0         # ↑ Increase
ATTRACTION_GAIN = 12.0           # ↑ Increase
```

### For Better Obstacle Avoidance
```python
REPULSION_GAIN = 4.0             # ↑ Increase
INFLUENCE_DISTANCE = 0.9         # ↑ Increase
```

### For More Precise Stops
```python
DISTANCE_TOLERANCE = 0.02        # ↓ Decrease
SETTLE_THRESHOLD = 10            # ↑ Increase
```

### For Faster Arm Motions
```python
maxVelocity = 6.0                # ↑ Increase (in move_arm_fast)
force = 1000                     # ↑ Increase (in move_arm_fast)
```

---

## 📊 Performance Metrics

The system can track:
- **Total distance traveled**: Sum of all navigation segments
- **Maximum path deviation**: How far from straight line
- **Task completion time**: Per cube and total
- **Closest obstacle approach**: Minimum distance during navigation
- **IK success rate**: Percentage of successful arm motions

---

## 🐛 Troubleshooting

### Robot doesn't move
- Check wheel velocities are non-zero
- Verify friction is not too high
- Ensure simulation is stepping

### Arm doesn't reach target
- Check target is within MAX_ARM_REACH (0.82m)
- Verify no joint limits are violated
- Increase maxNumIterations for IK

### Cube slips from gripper
- Increase constraint maxForce (currently 500)
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

## 📝 Future Enhancements

Possible improvements:
1. **Dynamic window approach** for local planning
2. **A* or RRT** for global path planning
3. **Real gripper simulation** with finger forces
4. **Visual servoing** for precise object grasping
5. **Multi-robot coordination**
6. **Learning-based navigation** using RL
7. **Real-time obstacle detection** using sensors

---

## 📄 License & Credits

This implementation uses:
- **PyBullet**: Physics simulation
- **Husky URDF**: Clearpath Robotics
- **KUKA iiwa URDF**: KUKA Robotics

---

## 📞 Support

For questions or issues, refer to:
- [PyBullet Documentation](https://pybullet.org)
- [KUKA iiwa Specifications](https://www.kuka.com)
- [Artificial Potential Fields Paper](https://ieeexplore.ieee.org/document/10115857)
