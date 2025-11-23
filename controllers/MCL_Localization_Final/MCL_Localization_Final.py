from controller import Robot
import numpy as np
import matplotlib.pyplot as plt
import random
from math import atan2, sqrt, cos, sin, pi

# ---------------------- Robot setup ----------------------
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Wheel and motion parameters
wheel_radius = 0.095
wheel_base = 0.33
MAX_SPEED = 6.28  # rad/s

# Motors and encoders
left_motor = robot.getDevice('left wheel')
right_motor = robot.getDevice('right wheel')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0)
right_motor.setVelocity(0)

left_enc = robot.getDevice('left wheel sensor')
right_enc = robot.getDevice('right wheel sensor')
left_enc.enable(timestep)
right_enc.enable(timestep)

# GPS + IMU
gps = robot.getDevice('gps')
gps.enable(timestep)
imu = robot.getDevice('inertial unit')
imu.enable(timestep)

# LiDAR
lidar = robot.getDevice('lidar')
lidar.enable(timestep)
lidar.enablePointCloud()
fov = lidar.getFov()
res = lidar.getHorizontalResolution()
angles = np.linspace(-fov/2, fov/2, res)

# ---------------------- Landmarks ----------------------
landmarks = [
    [5.0, 5.0], [5.0, -5.0], [-5.0, 5.0], [-5.0, -5.0],
    [0.0, 7.0], [7.0, 0.0], [-7.0, 0.0], [0.0, -7.0]
]

# ---------------------- Particle filter ----------------------
num_particles = 500

def initialize_particles(x_init, y_init, theta_init):
    particles = []
    for _ in range(num_particles):
        x = x_init + random.uniform(-0.5, 0.5)
        y = y_init + random.uniform(-0.5, 0.5)
        theta = theta_init + random.uniform(-0.1, 0.1)
        weight = 1.0 / num_particles
        particles.append([x, y, theta, weight])
    return particles

def motion_update(particles, delta_s, delta_theta):
    for i in range(len(particles)):
        x, y, theta, w = particles[i]
        noisy_ds = delta_s + random.gauss(0, 0.01*abs(delta_s))
        noisy_dt = delta_theta + random.gauss(0, 0.01*abs(delta_theta) + 0.01)
        x += noisy_ds * cos(theta + noisy_dt/2)
        y += noisy_ds * sin(theta + noisy_dt/2)
        theta += noisy_dt
        while theta > pi: theta -= 2*pi
        while theta < -pi: theta += 2*pi
        particles[i] = [x, y, theta, w]
    return particles

def detect_landmarks(lidar_ranges, angles):
    # Naive nearest points to landmarks detection (placeholder)
    detected = []
    for r, a in zip(lidar_ranges, angles):
        if 0 < r < 6.0:
            detected.append((r, a))
    return detected

def measurement_update(particles, detected_landmarks):
    if not detected_landmarks:
        return particles
    for i in range(len(particles)):
        x, y, theta, w = particles[i]
        likelihood = 1.0
        for r_meas, b_meas in detected_landmarks:
            min_error = float('inf')
            for lx, ly in landmarks:
                dx = lx - x
                dy = ly - y
                range_exp = sqrt(dx*dx + dy*dy)
                bearing_exp = atan2(dy, dx) - theta
                while bearing_exp > pi: bearing_exp -= 2*pi
                while bearing_exp < -pi: bearing_exp += 2*pi
                range_error = abs(range_exp - r_meas)
                bearing_error = abs(bearing_exp - b_meas)
                bearing_error = min(bearing_error, 2*pi - bearing_error)
                total_error = range_error + bearing_error
                min_error = min(min_error, total_error)
            likelihood *= np.exp(-min_error)
        particles[i][3] = w * likelihood
    total_w = sum(p[3] for p in particles)
    if total_w > 0:
        for p in particles:
            p[3] /= total_w
    return particles

def resample(particles):
    new_particles = []
    cumulative_weights = []
    total = 0
    for p in particles:
        total += p[3]
        cumulative_weights.append(total)
    for _ in range(num_particles):
        r = random.random() * total
        idx = 0
        while idx < len(cumulative_weights)-1 and r > cumulative_weights[idx]:
            idx += 1
        x, y, theta, _ = particles[idx]
        new_particles.append([x, y, theta, 1.0/num_particles])
    return new_particles

def estimate_pose(particles):
    x_sum, y_sum, cos_sum, sin_sum = 0, 0, 0, 0
    for x, y, theta, w in particles:
        x_sum += x*w
        y_sum += y*w
        cos_sum += cos(theta)*w
        sin_sum += sin(theta)*w
    return x_sum, y_sum, atan2(sin_sum, cos_sum)

# ---------------------- Visualization ----------------------
plt.ion()
fig, ax = plt.subplots(figsize=(7,7))

# ---------------------- Robot motion control ----------------------
linear_speed = 2.0
turn_speed = 1.5
move_steps = 50
turn_duration = 20
step_counter = 0
turn_counter = 0
turning = False

# ---------------------- Initialization ----------------------
particles = initialize_particles(0.0, 0.0, 0.0)
prev_left = left_enc.getValue()
prev_right = right_enc.getValue()
pf_traj = []
true_traj = []

# ---------------------- Main loop ----------------------
while robot.step(timestep) != -1:
    # --- Odometry ---
    left_val = left_enc.getValue()
    right_val = right_enc.getValue()
    delta_left = left_val - prev_left
    delta_right = right_val - prev_right
    prev_left = left_val
    prev_right = right_val
    delta_s = wheel_radius * (delta_left + delta_right)/2
    delta_theta = wheel_radius * (delta_right - delta_left)/wheel_base
    particles = motion_update(particles, delta_s, delta_theta)

    # --- LiDAR and measurement update ---
    lidar_ranges = lidar.getRangeImage()
    detected_landmarks = detect_landmarks(lidar_ranges, angles)
    particles = measurement_update(particles, detected_landmarks)
    particles = resample(particles)

    # --- Estimate pose ---
    x_pf, y_pf, theta_pf = estimate_pose(particles)
    pf_traj.append([x_pf, y_pf])

    # --- Ground truth ---
    gps_pos = gps.getValues()
    true_traj.append([gps_pos[0], gps_pos[2]])

    # --- Visualization ---
    ax.clear()
    xs = [p[0] for p in particles]
    ys = [p[1] for p in particles]
    ws = [p[3]*1000 for p in particles]
    ax.scatter(xs, ys, s=ws, alpha=0.5, color='blue')
    ax.scatter(x_pf, y_pf, s=100, color='red', marker='*')
    ax.scatter(gps_pos[0], gps_pos[2], s=100, color='green', marker='x')
    if len(pf_traj) > 1:
        ax.plot([p[0] for p in pf_traj], [p[1] for p in pf_traj], 'r-')
        ax.plot([p[0] for p in true_traj], [p[1] for p in true_traj], 'g-')
    ax.set_xlim(-4, 4)
    ax.set_ylim(-4, 4)
    ax.set_title("MCL: Particle Filter Localization")
    ax.grid(True)
    plt.pause(0.001)

    # --- Motion control ---
    if turning:
        left_motor.setVelocity(-turn_speed)
        right_motor.setVelocity(turn_speed)
        turn_counter += 1
        if turn_counter >= turn_duration:
            turning = False
            turn_counter = 0
            step_counter = 0
    else:
        left_motor.setVelocity(linear_speed)
        right_motor.setVelocity(linear_speed)
        step_counter += 1
        if step_counter >= move_steps:
            turning = True
