"""
Monte Carlo Localization (Particle Filter) - Integrated with Lab Tasks
Pioneer 3-DX - Webots Implementation for CSCI-529 Project 3.1
Improvements: Obstacle avoidance, better landmark detection, optimized resampling.
"""

from controller import Robot
import numpy as np
import matplotlib.pyplot as plt
import math
import random

# ==================== Simulation Parameters ====================
TIME_STEP = 64                  # ms
MAX_TIME = 120                  # seconds
ARENA_SIZE = 10.0               
NUM_PARTICLES = 500             
MOTION_NOISE_STD = 0.05         
SENSOR_NOISE_STD = 0.1
RESAMPLE_EVERY = 5              # steps
OBSTACLE_THRESHOLD = 0.25       # meters for obstacle avoidance

# Pioneer 3-DX parameters
WHEEL_RADIUS = 0.095            
AXLE_LENGTH = 0.33              

# ==================== Initialize Webots Robot ====================
robot = Robot()
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
max_vel = left_motor.getMaxVelocity()

left_encoder = robot.getDevice("left wheel sensor")
right_encoder = robot.getDevice("right wheel sensor")
left_encoder.enable(TIME_STEP)
right_encoder.enable(TIME_STEP)

lidar = robot.getDevice("lidar")
lidar.enable(TIME_STEP)

gps = robot.getDevice("gps")
gps.enable(TIME_STEP)

imu = robot.getDevice("inertial unit")
imu.enable(TIME_STEP)

# ==================== Landmarks ====================
landmarks = np.array([
    [5.0, 5.0], [5.0, -5.0], [-5.0, 5.0], [-5.0, -5.0],
    [0.0, 7.0], [7.0, 0.0], [-7.0, 0.0], [0.0, -7.0]
])

# ==================== Particle Initialization ====================
particles = np.zeros((NUM_PARTICLES, 3))  
particles[:, 0] = np.random.uniform(-ARENA_SIZE/2, ARENA_SIZE/2, NUM_PARTICLES)
particles[:, 1] = np.random.uniform(-ARENA_SIZE/2, ARENA_SIZE/2, NUM_PARTICLES)
particles[:, 2] = np.random.uniform(-np.pi, np.pi, NUM_PARTICLES)
weights = np.ones(NUM_PARTICLES) / NUM_PARTICLES

# Odometry
x_odom, y_odom, theta_odom = 0.0, 0.0, 0.0
prev_left_encoder, prev_right_encoder = 0.0, 0.0

# Trajectories
odom_trajectory = []
pf_trajectory = []
true_trajectory = []

# ==================== Helper Functions ====================
def normalize_angle(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi

def odometry_update(delta_left, delta_right):
    global x_odom, y_odom, theta_odom
    delta_s_left = delta_left * WHEEL_RADIUS
    delta_s_right = delta_right * WHEEL_RADIUS
    delta_s = (delta_s_left + delta_s_right) / 2
    delta_theta = (delta_s_right - delta_s_left) / AXLE_LENGTH
    x_odom += delta_s * math.cos(theta_odom + delta_theta/2)
    y_odom += delta_s * math.sin(theta_odom + delta_theta/2)
    theta_odom = normalize_angle(theta_odom + delta_theta)

def motion_update(particles, delta_s, delta_theta):
    for i in range(NUM_PARTICLES):
        noisy_ds = delta_s + random.gauss(0, MOTION_NOISE_STD)
        noisy_dt = delta_theta + random.gauss(0, MOTION_NOISE_STD/2)
        particles[i, 0] += noisy_ds * math.cos(particles[i, 2] + noisy_dt/2)
        particles[i, 1] += noisy_ds * math.sin(particles[i, 2] + noisy_dt/2)
        particles[i, 2] = normalize_angle(particles[i, 2] + noisy_dt)
    return particles

def detect_landmarks(lidar_values):
    detected = []
    min_gap = 0.1
    cluster = []
    angle_min = -2.09
    angle_step = 4.18 / (len(lidar_values)-1)
    for i, r in enumerate(lidar_values):
        if r < lidar.getMaxRange():
            if not cluster:
                cluster = [i]
            elif abs(r - lidar_values[cluster[-1]]) < min_gap:
                cluster.append(i)
            else:
                if len(cluster) >= 3:
                    center_idx = cluster[len(cluster)//2]
                    detected.append([lidar_values[center_idx], angle_min + center_idx*angle_step])
                cluster = [i]
    if cluster and len(cluster) >=3:
        center_idx = cluster[len(cluster)//2]
        detected.append([lidar_values[center_idx], angle_min + center_idx*angle_step])
    return detected

def measurement_update(particles, weights, detected_landmarks):
    if not detected_landmarks: return weights
    for i in range(NUM_PARTICLES):
        likelihood = 1.0
        for r_d, b_d in detected_landmarks:
            min_error = float('inf')
            for lm in landmarks:
                dx = lm[0] - particles[i,0]
                dy = lm[1] - particles[i,1]
                r_exp = math.sqrt(dx**2 + dy**2) + random.gauss(0, SENSOR_NOISE_STD)
                b_exp = normalize_angle(math.atan2(dy, dx) - particles[i,2])
                total_error = abs(r_exp - r_d) + abs(b_exp - b_d)
                min_error = min(min_error, total_error)
            likelihood *= math.exp(-min_error/(2*SENSOR_NOISE_STD**2))
        weights[i] *= likelihood
    weights += 1e-300
    weights /= np.sum(weights)
    return weights

def resample(particles, weights):
    indices = np.random.choice(np.arange(NUM_PARTICLES), size=NUM_PARTICLES, p=weights)
    particles[:] = particles[indices]
    weights.fill(1.0/NUM_PARTICLES)
    particles += np.random.normal(0,0.01, particles.shape)
    return particles, weights

def rmse(a,b):
    return np.sqrt(np.mean((np.array(a)-np.array(b))**2))

def obstacle_avoidance(lidar_values, vL, vR):
    min_range = min(lidar_values)
    if min_range < OBSTACLE_THRESHOLD:
        # Stop and rotate randomly
        if random.random() > 0.5:
            return -0.1, 0.1
        else:
            return 0.1, -0.1
    return vL, vR

# ==================== Visualization ====================
plt.ion()
fig, ax = plt.subplots(figsize=(10,10))

# ==================== Main Loop ====================
v_cmd = 0.2
w_cmd = 0.1
t = 0
step_count = 0
prev_left, prev_right = left_encoder.getValue(), right_encoder.getValue()

while robot.step(TIME_STEP) != -1 and t < MAX_TIME:
    dt = TIME_STEP/1000.0
    t += dt
    step_count +=1

    # Sensors
    left_pos = left_encoder.getValue()
    right_pos = right_encoder.getValue()
    lidar_values = lidar.getRangeImage()
    gps_pos = gps.getValues()
    true_trajectory.append([gps_pos[0], gps_pos[2]])

    # Odometry
    delta_left = left_pos - prev_left
    delta_right = right_pos - prev_right
    prev_left, prev_right = left_pos, right_pos
    odometry_update(delta_left, delta_right)
    odom_trajectory.append([x_odom, y_odom])

    delta_s_left = delta_left*WHEEL_RADIUS
    delta_s_right = delta_right*WHEEL_RADIUS
    delta_s = (delta_s_left + delta_s_right)/2
    delta_theta = (delta_s_right - delta_s_left)/AXLE_LENGTH

    # Particle filter
    particles = motion_update(particles, delta_s, delta_theta)
    detected_landmarks = detect_landmarks(lidar_values)
    weights = measurement_update(particles, weights, detected_landmarks)

    if step_count % RESAMPLE_EVERY == 0:
        particles, weights = resample(particles, weights)

    estimate = np.average(particles, axis=0, weights=weights)
    pf_trajectory.append(estimate[:2])

    # Velocity control with obstacle avoidance
    vL = (v_cmd - w_cmd*AXLE_LENGTH/2)/WHEEL_RADIUS
    vR = (v_cmd + w_cmd*AXLE_LENGTH/2)/WHEEL_RADIUS
    vL, vR = obstacle_avoidance(lidar_values, vL, vR)
    vL = np.clip(vL, -max_vel, max_vel)
    vR = np.clip(vR, -max_vel, max_vel)
    left_motor.setVelocity(vL)
    right_motor.setVelocity(vR)

    # Live plot every 5 steps to reduce slowdown
    if step_count % 5 == 0:
        ax.clear()
        ax.set_xlim(-ARENA_SIZE/2, ARENA_SIZE/2)
        ax.set_ylim(-ARENA_SIZE/2, ARENA_SIZE/2)
        ax.scatter(particles[:,0], particles[:,1], s=weights*1000, color='blue', alpha=0.5)
        if odom_trajectory:
            ax.plot(*zip(*odom_trajectory),'black', label='Odometry')
        if pf_trajectory:
            ax.plot(*zip(*pf_trajectory),'r-', label='MCL')
        if true_trajectory:
            ax.plot(*zip(*true_trajectory),'g-', label='True')
        ax.scatter(landmarks[:,0], landmarks[:,1], color='orange', s=100, marker='X', label='Landmarks')
        ax.legend()
        plt.pause(0.001)

# Stop robot
left_motor.setVelocity(0)
right_motor.setVelocity(0)

# Final evaluation
odom_rmse = rmse(odom_trajectory,true_trajectory)
pf_rmse = rmse(pf_trajectory,true_trajectory)
print(f"Odometry RMSE: {odom_rmse:.3f} m")
print(f"Particle Filter RMSE: {pf_rmse:.3f} m")

plt.ioff()
fig, ax_final = plt.subplots(figsize=(10,10))
ax_final.set_xlim(-ARENA_SIZE/2, ARENA_SIZE/2)
ax_final.set_ylim(-ARENA_SIZE/2, ARENA_SIZE/2)
ax_final.scatter(particles[:,0], particles[:,1], s=weights*1000, color='blue', alpha=0.5, label='Particles')
ax_final.plot(*zip(*odom_trajectory),'black', label='Odometry')
ax_final.plot(*zip(*pf_trajectory),'r-', label='MCL')
ax_final.plot(*zip(*true_trajectory),'g-', label='True')
ax_final.scatter(landmarks[:,0], landmarks[:,1], color='orange', s=100, marker='X', label='Landmarks')
ax_final.legend()
plt.show()
