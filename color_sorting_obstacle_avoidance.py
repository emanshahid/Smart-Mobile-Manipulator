import pybullet as p
import pybullet_data
import time
import random
import math
import numpy as np

husky = None
kuka = None
cubes = {}
bin_positions = []
bin_colors = {}
ARM_JOINTS = list(range(7))
END_EFFECTOR_INDEX = 6

# ARM CONSTRAINTS
MAX_ARM_REACH = 0.82
COMFORTABLE_REACH = 0.50
IK_TOLERANCE = 0.08

# SINGULARITY DETECTION PARAMETERS
JOINT_LIMITS = [
    (-2.967, 2.967), (-2.094, 2.094), (-2.967, 2.967), (-2.094, 2.094),
    (-2.967, 2.967), (-2.094, 2.094), (-3.054, 3.054),
]
JOINT_LIMIT_MARGIN = 0.15
WRIST_SINGULARITY_THRESHOLD = 0.20
MANIPULABILITY_THRESHOLD = 0.01

gripper_state = {'constraint': None, 'is_closed': False}

LEFT_WHEELS = [2, 4]
RIGHT_WHEELS = [3, 5]

def initialize_simulation():
    """Initialize PyBullet simulation environment"""
    print("\n" + "="*70)
    print("🤖 OPTIMIZED MOBILE MANIPULATOR - STRATEGIC DELAYS")
    print("="*70)
    
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.8)
    p.setTimeStep(1./240.)
    
    p.resetDebugVisualizerCamera(
        cameraDistance=6.0, cameraYaw=45, cameraPitch=-35, 
        cameraTargetPosition=[3.0, 2.0, 0.5]
    )
    

def create_environment():

    """Create ground plane, bins, and cubes"""
    global bin_positions, bin_colors, cubes, bin_centers
    bin_centers = []

    plane = p.loadURDF("plane.urdf")
    p.changeDynamics(plane, -1, lateralFriction=2.0)
    
    bin_size = [0.15, 0.15, 0.1]
    wall_thickness = 0.02
    bin_colors = {"red": [1,0,0,1], "yellow": [1,1,0,1], "green": [0,1,0,1]}
    bin_positions = [[5.0, 1.0, bin_size[2]], [5.0, 2.0, bin_size[2]], [5.0, 3.0, bin_size[2]]]
    
    for color, pos in zip(bin_colors.values(), bin_positions):
        wall_dims = [
            [wall_thickness, bin_size[1], bin_size[2]], [wall_thickness, bin_size[1], bin_size[2]],
            [bin_size[0], wall_thickness, bin_size[2]], [bin_size[0], wall_thickness, bin_size[2]]
        ]
        wall_pos = [
            [-bin_size[0]+wall_thickness,0,0], [bin_size[0]-wall_thickness,0,0],
            [0,-bin_size[1]+wall_thickness,0], [0,bin_size[1]-wall_thickness,0]
        ]
        for dim, w in zip(wall_dims, wall_pos):
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=dim)
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=dim, rgbaColor=color)
            p.createMultiBody(
                baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                basePosition=[pos[0]+w[0], pos[1]+w[1], pos[2]+w[2]]
            )
    cube_size = 0.1
    cube_colors = {"red": [1,0,0,1], "yellow":[1,1,0,1], "green":[0,1,0,1]}
    cube_positions = []
    
    while len(cube_positions) < 3:
        x = random.uniform(2.5, 4.0)
        y = random.uniform(0.5, 3.5)
        pos = [x, y, cube_size]
        if all(((pos[0]-p[0])**2 + (pos[1]-p[1])**2) > 0.5 for p in cube_positions):
            cube_positions.append(pos)
    
    cubes = {}
    for color, pos in zip(cube_colors.keys(), cube_positions):
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[cube_size/2]*3)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[cube_size/2]*3, rgbaColor=cube_colors[color])
        cube_id = p.createMultiBody(
            baseMass=0.05, baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis, basePosition=pos
        )
        cubes[cube_id] = {"color": color, "picked": False}
    
    print("✓ Environment created")
    return cubes


def setup_mobile_base():
    """Load and configure Husky mobile base"""
    global husky
    orientation_husky = p.getQuaternionFromEuler([0, 0, 0])
    husky = p.loadURDF("husky/husky.urdf", [1.0, 2.0, 0.1], orientation_husky)

    for wheel in LEFT_WHEELS + RIGHT_WHEELS:
        p.setJointMotorControl2(husky, wheel, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
        p.changeDynamics(husky, wheel, lateralFriction=10, spinningFriction=0.001, rollingFriction=0.001)
    
    p.changeDynamics(husky, -1, mass=70.0, lateralFriction=100.0, linearDamping=1.0, angularDamping=1.0)
    
    print("✓ Husky loaded at (1.0, 2.0)")
    return husky


def setup_arm():
    """Load and configure KUKA arm"""
    global kuka
    orientation = p.getQuaternionFromEuler([0, 0, -math.pi])
    kuka = p.loadURDF("kuka_iiwa/model.urdf", [1.0, 2.0, 0.65], orientation)

    for i in range(-1, p.getNumJoints(kuka)):
        p.changeDynamics(kuka, i, mass=2.0)
    
    base_constraint = p.createConstraint(
        husky, -1, kuka, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0.2], [0, 0, 0]
    )
    p.changeConstraint(base_constraint, maxForce=10000)
    
    initial_joint_positions = [0, -0.5, 0, -1.5, 0, 1.5, 0]
    for i, joint_pos in enumerate(initial_joint_positions):
        p.resetJointState(kuka, i, joint_pos)
    
    print("✓ KUKA arm loaded and attached")
    return kuka


def print_environment_info():
    """Print debug information"""
    print("\n📊 Environment Setup:")
    husky_pos = p.getBasePositionAndOrientation(husky)[0]
    kuka_pos = p.getBasePositionAndOrientation(kuka)[0]
    print(f"  Husky: ({husky_pos[0]:.2f}, {husky_pos[1]:.2f})")
    print(f"  Kuka:  ({kuka_pos[0]:.2f}, {kuka_pos[1]:.2f})")
    
    print(f"\n  Cubes:")
    for cube_id, info in cubes.items():
        cube_pos = p.getBasePositionAndOrientation(cube_id)[0]
        print(f"    {info['color'].upper()}: ({cube_pos[0]:.2f}, {cube_pos[1]:.2f})")


def set_wheel_velocities(left_vel, right_vel):
    """Control Husky differential drive"""
    for wheel in LEFT_WHEELS:
        p.setJointMotorControl2(husky, wheel, p.VELOCITY_CONTROL, targetVelocity=left_vel, force=1000)
    for wheel in RIGHT_WHEELS:
        p.setJointMotorControl2(husky, wheel, p.VELOCITY_CONTROL, targetVelocity=right_vel, force=1000)


def stop_base():
    """Stop the mobile base"""
    set_wheel_velocities(0, 0)


def stabilize_physics():
    """Run simulation for a specified duration to stabilize physics"""
    steps = int(0.1* 240)  # 240 Hz simulation
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(1./24000.)


def move_toward_target_with_avoidance(target_pos, target_cube_id=None, target_distance=0.77):
    """Move to target with obstacle avoidance and path tracking"""
    DISTANCE_TOLERANCE = 0.04
    MAX_LINEAR_SPEED = 60.0
    
    ATTRACTION_GAIN = 10.0
    REPULSION_GAIN = 3.5
    INFLUENCE_DISTANCE = 0.6
    PATH_TRACKING_GAIN = 6.0
    ANGULAR_GAIN = 5.0
    
    start_pos = p.getBasePositionAndOrientation(husky)[0]
    start_x, start_y = start_pos[0], start_pos[1]
    
    path_dx = target_pos[0] - start_x
    path_dy = target_pos[1] - start_y
    path_length = math.sqrt(path_dx**2 + path_dy**2)
    
    path_unit_x = path_dx / path_length
    path_unit_y = path_dy / path_length
    perp_unit_x = -path_unit_y
    perp_unit_y = path_unit_x
    
    settled_count = 0
    SETTLE_THRESHOLD = 2
    max_deviation = 0.0
    
    print(f"🚗 Moving to ({target_pos[0]:.2f}, {target_pos[1]:.2f})...")
    
    while True:
        husky_pos, husky_orn = p.getBasePositionAndOrientation(husky)
        husky_x, husky_y = husky_pos[0], husky_pos[1]
        husky_yaw = p.getEulerFromQuaternion(husky_orn)[2]
        
        obstacles = get_obstacle_positions(exclude_cube_id=target_cube_id)
        
        dx_goal = target_pos[0] - husky_x
        dy_goal = target_pos[1] - husky_y
        distance_to_goal = math.sqrt(dx_goal**2 + dy_goal**2)
        distance_error = distance_to_goal - target_distance
        
        if abs(distance_error) <= DISTANCE_TOLERANCE:
            settled_count += 1
            if settled_count >= SETTLE_THRESHOLD:
                stop_base()
                print(f"✅ Reached target! Distance: {distance_to_goal:.3f}m, Max deviation: {max_deviation:.3f}m")
                break
        else:
            settled_count = 0
        
        # Path tracking
        current_dx = husky_x - start_x
        current_dy = husky_y - start_y
        deviation = current_dx * perp_unit_x + current_dy * perp_unit_y
        max_deviation = max(max_deviation, abs(deviation))
        path_correction_x = -PATH_TRACKING_GAIN * deviation * perp_unit_x
        path_correction_y = -PATH_TRACKING_GAIN * deviation * perp_unit_y
        
        # Attractive force
        attr_force_x = ATTRACTION_GAIN * (dx_goal / distance_to_goal) * distance_error
        attr_force_y = ATTRACTION_GAIN * (dy_goal / distance_to_goal) * distance_error
        
        # Repulsive forces
        total_rep_x = 0.0
        total_rep_y = 0.0
        closest_obstacle_dist = float('inf')
        
        for obs_pos in obstacles:
            rep_x, rep_y = calculate_repulsive_force(
                (husky_x, husky_y), obs_pos, INFLUENCE_DISTANCE, REPULSION_GAIN
            )
            total_rep_x += rep_x
            total_rep_y += rep_y
            dist_to_obs = math.sqrt((husky_x - obs_pos[0])**2 + (husky_y - obs_pos[1])**2)
            closest_obstacle_dist = min(closest_obstacle_dist, dist_to_obs)
        
        # Combined force
        total_force_x = attr_force_x + total_rep_x 
        total_force_y = attr_force_y + total_rep_y 
        
        desired_yaw = math.atan2(total_force_y, total_force_x)
        yaw_error = desired_yaw - husky_yaw
        while yaw_error > math.pi: yaw_error -= 2*math.pi
        while yaw_error < -math.pi: yaw_error += 2*math.pi
        
        force_magnitude = math.sqrt(total_force_x**2 + total_force_y**2)
        
        if closest_obstacle_dist < INFLUENCE_DISTANCE:
            speed_factor = closest_obstacle_dist / INFLUENCE_DISTANCE
            linear_speed = min(force_magnitude, MAX_LINEAR_SPEED * speed_factor)
        else:
            linear_speed = min(force_magnitude, MAX_LINEAR_SPEED)
        
        if abs(yaw_error) > 10 * (math.pi / 180):
            left_vel = -ANGULAR_GAIN * yaw_error
            right_vel = ANGULAR_GAIN * yaw_error
        else:
            left_vel = linear_speed - ANGULAR_GAIN * yaw_error
            right_vel = linear_speed + ANGULAR_GAIN * yaw_error
        
        max_vel = 60.0
        left_vel = max(-max_vel, min(max_vel, left_vel))
        right_vel = max(-max_vel, min(max_vel, right_vel))
        
        set_wheel_velocities(left_vel, right_vel)
        p.stepSimulation()
        time.sleep(1./24000.)

def move_arm_to_home():
    """Move arm to home position"""
    print(" Home position")
    joint_home = [0, -0.5, 0, -1.5, 0, 1.5, 0]
    for i, jp in enumerate(joint_home):
        p.resetJointState(kuka, i, jp)


def get_obstacle_positions(exclude_cube_id=None):
    """Get positions of all cubes except the one being manipulated"""
    obstacle_positions = []
    for cube_id, info in cubes.items():
        if cube_id != exclude_cube_id and not info.get('picked', False):
            pos, _ = p.getBasePositionAndOrientation(cube_id)
            obstacle_positions.append(pos[:2])
    return obstacle_positions


def calculate_repulsive_force(robot_pos, obstacle_pos, influence_distance=0.5, repulsion_gain=2.0):
    """Calculate repulsive force from an obstacle"""
    dx = robot_pos[0] - obstacle_pos[0]
    dy = robot_pos[1] - obstacle_pos[1]
    distance = math.sqrt(dx**2 + dy**2)
    
    if distance > influence_distance or distance < 0.01:
        return (0.0, 0.0)
    
    force_magnitude = repulsion_gain * (1.0/distance - 1.0/influence_distance) * (1.0/distance**2)
    force_x = (dx / distance) * force_magnitude
    force_y = (dy / distance) * force_magnitude
    
    return (force_x, force_y)


def pick_object_fast(object_id):
    """
    Fast pick with strategic delays only at critical moments
    """
    move_arm_to_home()
    obj_pos, _ = p.getBasePositionAndOrientation(object_id)
    print(f"\n📦 PICKING at ({obj_pos[0]:.2f}, {obj_pos[1]:.2f})")

    open_gripper()
    
    # DELAY 1: Small pause at pick location to stabilize
    print(f"⏱ Stabilizing at pick location ({DELAY_AT_PICK_LOCATION}s)...")
    stabilize_physics()
    
    # Update position after stabilization
    obj_pos, _ = p.getBasePositionAndOrientation(object_id)
    
    # Approach
    approach_pos = [obj_pos[0], obj_pos[1], obj_pos[2] +0.5]
    move_arm_fast(approach_pos)
    
    # Lower to grasp
    grasp_pos = [obj_pos[0], obj_pos[1], obj_pos[2] +0.2]
    move_arm_fast(grasp_pos)

    # GRASP
    close_gripper(object_id)
    
    # DELAY 2: Small pause after grasping to ensure secure grip
    print(f"⏱ Securing grip ({DELAY_AFTER_GRASP}s)...")
    stabilize_physics()

    # Lift
    lift_pos = [obj_pos[0], obj_pos[1], obj_pos[2] + 0.35]
    move_arm_fast(lift_pos)
    
    print(" PICKED")
    return True


def place_object_fast(target_pos):
    """
    Fast place with strategic delays only at critical moments
    """
    print(f"\n📍 PLACING at ({target_pos[0]:.2f}, {target_pos[1]:.2f})")

    # DELAY 3: Small pause at bin location to stabilize
    print(f"⏱ Stabilizing at bin location ({DELAY_AT_BIN_LOCATION}s)...")
    stabilize_physics()
    
    # Approach
    approach_pos = [target_pos[0], target_pos[1], target_pos[2] + 0.20]
    move_arm_fast(approach_pos)
    
    # Lower
    place_pos = [target_pos[0], target_pos[1], target_pos[2] + 0.1]
    move_arm_fast(place_pos)
    
    # Release
    release_gripper()
    
    # DELAY 4: Small pause after dropping to ensure cube settles
    print(f"⏱ Cube settling ({DELAY_AFTER_DROP}s)...")
    stabilize_physics()
    
    # Retract
    retract_pos = [target_pos[0], target_pos[1], target_pos[2] + 0.25]
    move_arm_fast(retract_pos)
    
    print("✅ PLACED")
    return True

def move_arm_fast(target_pos, target_orn=None, max_steps=500):
    """Move arm fast with minimal convergence waiting"""
    stop_base()
    
    if target_orn is None:
        target_orn = p.getQuaternionFromEuler([np.pi, 0, 0])
    
    # Get best IK solution
    best_joint_poses = None
    best_error = float('inf')
    for _ in range(1):
        joint_poses = p.calculateInverseKinematics(
            kuka, END_EFFECTOR_INDEX, target_pos, target_orn,
            maxNumIterations=100, residualThreshold=1e-5
        )
        
        current_states = get_joint_positions()
        for i in range(7):
            p.resetJointState(kuka, i, joint_poses[i])
            p.setJointMotorControl2(kuka, i, p.POSITION_CONTROL, targetPosition=joint_poses[i], force=500, maxVelocity=4.0)
        
        p.stepSimulation()
        
        ee_state = p.getLinkState(kuka, END_EFFECTOR_INDEX)
        ee_pos = ee_state[0]
        error = math.sqrt(sum((ee_pos[i] - target_pos[i])**2 for i in range(3)))
        
        if error < best_error:
            best_error = error
            best_joint_poses = joint_poses
        
        for i in range(7):
            p.resetJointState(kuka, i, current_states[i])
        
        if error < 0.0002:
            break
    
    joint_poses = best_joint_poses
    
    # Apply commands with high speed
    for i in range(7):
        p.setJointMotorControl2(
            kuka, i, p.POSITION_CONTROL,
            targetPosition=joint_poses[i],
            force=800,
            maxVelocity=5.0
        )
    
    # Wait for convergence
    for _ in range(max_steps):
        p.stepSimulation()
        time.sleep(1./2400.)
        
        ee_state = p.getLinkState(kuka, END_EFFECTOR_INDEX)
        ee_pos = ee_state[0]
        dist = math.sqrt(sum((ee_pos[i] - target_pos[i])**2 for i in range(3)))
        
        if dist < 0.025:
            return True
    
    return True


def open_gripper():
    """Open gripper"""
    gripper_state['is_closed'] = False


def close_gripper(target_object_id):
    """Close gripper and attach object"""
    gripper_state['constraint'] = p.createConstraint(
        kuka, END_EFFECTOR_INDEX, target_object_id, -1,
        p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0]
    )
    p.changeConstraint(gripper_state['constraint'], maxForce=500)
    gripper_state['is_closed'] = True


def release_gripper():
    """Release gripper"""
    if gripper_state['constraint'] is not None:
        p.removeConstraint(gripper_state['constraint'])
        gripper_state['constraint'] = None
        gripper_state['is_closed'] = False


def get_joint_positions():
    """Get current joint positions"""
    return [p.getJointState(kuka, i)[0] for i in range(7)]


def main():
    initialize_simulation()
    create_environment()
    setup_mobile_base()
    setup_arm()
    move_arm_to_home()
    print_environment_info()
    
    print("\n" + "="*70)
    print("🚀 OPTIMIZED OPERATIONS WITH STRATEGIC DELAYS")
    print("="*70)
    

    # Get robot’s current base position
    robot_pos = p.getBasePositionAndOrientation(husky)[0]

    # Compute distances of all cubes from the robot
    cube_distances = []
    for cid, info in cubes.items():
        cube_pos = p.getBasePositionAndOrientation(cid)[0]
        distance = ((cube_pos[0] - robot_pos[0])**2 + 
                    (cube_pos[1] - robot_pos[1])**2 + 
                    (cube_pos[2] - robot_pos[2])**2) ** 0.5
        cube_distances.append((cid, info['color'], cube_pos, distance))

    # Sort cubes by distance
    cube_distances.sort(key=lambda x: x[3])

    # Iterate through sorted cubes
    for i, (cube_id, color, cube_pos, dist) in enumerate(cube_distances):
        print("\n" + "="*70)
        print(f"🎯 TASK {i+1}: {color.upper()} CUBE (Distance: {dist:.2f} m)")
        print("="*70)
    
        # Move to cube
        move_toward_target_with_avoidance(cube_pos, cube_id, 0.65)
        stop_base()
    
        # Pick cube
        pick_object_fast(cube_id)
        cubes[cube_id]['picked'] = True
        if color == 'red':
            #  Move to bin (assuming bin_positions[0] matches cube order)
            move_toward_target_with_avoidance(bin_positions[0], None, 0.75)
    
            # Drop cube
            place_object_fast(bin_positions[0])
            cubes[cube_id]['picked'] = False
        elif color == 'yellow':
             # Move to bin (assuming bin_positions[1] matches cube order)
            move_toward_target_with_avoidance(bin_positions[1], None, 0.75)
            place_object_fast(bin_positions[1])
            cubes[cube_id]['picked'] = False
        else:
            move_toward_target_with_avoidance(bin_positions[2], None, 0.75)
            place_object_fast(bin_positions[2])
            cubes[cube_id]['picked'] = False
        


    print("\n" + "="*70)
    print("✅ ALL TASKS COMPLETED!")
    print("="*70)
    
    try:
        while True:
            p.stepSimulation()
            time.sleep(1./24000.)
    except KeyboardInterrupt:
        p.disconnect()


if __name__ == "__main__":
    main()