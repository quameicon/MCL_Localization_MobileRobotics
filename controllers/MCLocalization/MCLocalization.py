"""
Monte Carlo Localization (Particle Filter) for Webots
- Uses GPS, IMU, LiDAR, and wheel encoders
- Particle filter localization
- Real-time visualization
- CSV logging and final plot saved with particle number in filename
"""

from controller import Robot
import numpy as np
import matplotlib.pyplot as plt
import math
import random
import os
import csv

# ---------------- Simulation parameters ----------------
TIME_STEP = 64
MAX_STEPS = 2000

# ---------------- User-editable parameters ----------------
NUM_PARTICLES = 600
SAVE_RESULTS = True  # Toggle saving CSV and plots
SNAP_EVERY = 250     # Save snapshots every N steps

motion_noise_start, motion_noise_end = 0.25, 0.03
turn_noise_start, turn_noise_end = 0.25, 0.01
sense_noise_start, sense_noise_end = 0.8, 0.12

# ---------------- Robot initialization ----------------
robot = Robot()

# Motors
left_motor = robot.getDevice("left wheel")
right_motor = robot.getDevice("right wheel")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Wheel encoders
left_enc = right_enc = None
try:
    left_enc = robot.getDevice("left wheel sensor")
    right_enc = robot.getDevice("right wheel sensor")
    left_enc.enable(TIME_STEP)
    right_enc.enable(TIME_STEP)
except Exception:
    left_enc = right_enc = None

# GPS
gps = None
try:
    gps = robot.getDevice("gps")
    if gps:
        gps.enable(TIME_STEP)
except Exception:
    gps = None

# Inertial Unit
imu = None
try:
    imu = robot.getDevice("inertial_unit")
    if imu:
        imu.enable(TIME_STEP)
except Exception:
    imu = None

# LiDAR
lidar = None
try:
    lidar = robot.getDevice("lidar")
    if lidar:
        lidar.enable(TIME_STEP)
        lidar.enablePointCloud()
except Exception:
    lidar = None

# Robot kinematics
WHEEL_RADIUS = 0.0975
AXLE_LENGTH = 0.331

# ---------------- Landmarks ----------------
webots_landmarks = [
    ("Pedestrian", 3.71, -4.42),
    ("OilBarrel", -2.99, -3.62),
    ("Wall_1", 3.42, -2.10),
    ("Wall_4", -1.60, -2.44),
    ("Wall_6", -1.03, 2.75),
    ("Wall_5", -2.77, 0.56),
    ("Wall_2", 0.90, -3.95),
    ("Wall_3", 3.72, 1.70),
    ("WoodenBox", 1.82, 1.45)
]
landmark_positions = [(x, z) for _, x, z in webots_landmarks]

# ---------------- Particle Filter ----------------
def init_particles():
    xs = [x for x,_ in landmark_positions]
    zs = [z for _,z in landmark_positions]
    min_x, max_x = min(xs)-1.0, max(xs)+1.0
    min_z, max_z = min(zs)-1.0, max(zs)+1.0
    w = 1.0 / NUM_PARTICLES
    return [[random.uniform(min_x,max_x), random.uniform(min_z,max_z),
             random.uniform(-math.pi, math.pi), w] for _ in range(NUM_PARTICLES)]

def normalize(angle):
    return (angle + math.pi) % (2*math.pi) - math.pi

def motion_step(particles, delta_s, delta_theta, motion_noise, turn_noise):
    for i in range(len(particles)):
        x, z, th, w = particles[i]
        ds = delta_s + random.gauss(0, motion_noise)
        dth = delta_theta + random.gauss(0, turn_noise)
        x += ds * math.cos(th + dth/2.0)
        z += ds * math.sin(th + dth/2.0)
        th = normalize(th + dth)
        particles[i] = [x, z, th, w]
    return particles

def measurement_update_range_only(particles, measurements, sense_noise):
    if not measurements:
        return particles
    for i in range(len(particles)):
        x, z, th, w = particles[i]
        prob = 1.0
        for r_meas, lm_idx in measurements:
            lx, lz = landmark_positions[lm_idx]
            expected = math.hypot(lx - x, lz - z)
            prob *= (1.0 / (math.sqrt(2*math.pi) * sense_noise)) * \
                    math.exp(-0.5*((r_meas-expected)/sense_noise)**2)
        particles[i][3] = w * prob
    total = sum(p[3] for p in particles)
    if total <= 0 or not np.isfinite(total):
        return init_particles()
    for p in particles:
        p[3] /= total
    return particles

def systematic_resample(particles):
    N = len(particles)
    weights = np.array([p[3] for p in particles], dtype=float)
    if weights.sum() <= 0 or not np.isfinite(weights.sum()):
        return init_particles()
    positions = (np.arange(N)+random.random())/N
    cumulative = np.cumsum(weights)
    new_particles = []
    idx = 0
    for pos in positions:
        while pos > cumulative[idx]:
            idx += 1
            if idx >= N:
                idx = N-1
                break
        x, z, th, _ = particles[idx]
        new_particles.append([x+random.gauss(0,0.001),
                              z+random.gauss(0,0.001),
                              normalize(th+random.gauss(0,0.001)), 1.0/N])
    return new_particles

def estimate_pose(particles):
    x = sum(p[0]*p[3] for p in particles)
    z = sum(p[1]*p[3] for p in particles)
    s = sum(math.sin(p[2])*p[3] for p in particles)
    c = sum(math.cos(p[2])*p[3] for p in particles)
    th = math.atan2(s, c)
    return (x, z, th)

# ---------------- File setup ----------------
RESULTS_DIR = r"C:\Users\Listowell Lord Adams\OneDrive - St. Francis Xavier University\Webot Labs\results"
os.makedirs(RESULTS_DIR, exist_ok=True)
RESULT_CSV = os.path.join(RESULTS_DIR, f"mcl_results_{NUM_PARTICLES}_particles.csv")
RESULT_PLOT = os.path.join(RESULTS_DIR, f"mcl_final_plot_{NUM_PARTICLES}_particles.png")

# ---------------- Initialize particles ----------------
particles = init_particles()

# ---------------- Visualization ----------------
plt.ion()
fig, ax = plt.subplots(figsize=(7,7))
ax.set_title("MCL — particles converge")
ax.set_xlabel("X [m]"); ax.set_ylabel("Z [m]")
ax.grid(True)
for nm, x, z in webots_landmarks:
    ax.scatter([x],[z],marker='s',s=80,color='orange')
    ax.text(x+0.05,z+0.05,nm,fontsize=8)
particle_scatter = ax.scatter([p[0] for p in particles],[p[1] for p in particles],
                              s=6, alpha=0.25, c='blue')
est_dot, = ax.plot([0.0],[0.0],marker='*',color='red',markersize=12,label='PF estimate')
true_dot, = ax.plot([0.0],[0.0],marker='o',color='green',markersize=6,label='True pose')
ax.legend(loc='upper right')

# ---------------- Odometry bookkeeping ----------------
prev_left = prev_right = None
odom_x = odom_z = odom_theta = 0.0
if left_enc and right_enc:
    prev_left = left_enc.getValue()
    prev_right = right_enc.getValue()

current_motion_noise = motion_noise_start
current_turn_noise = turn_noise_start
current_sense_noise = sense_noise_start

# CSV logging
csv_data = []

# ---------------- Main loop ----------------
step = 0
while robot.step(TIME_STEP) != -1 and step < MAX_STEPS:
    step += 1
    ratio = min(step/(MAX_STEPS*0.9),1.0)
    current_motion_noise = motion_noise_start*(1-ratio) + motion_noise_end*ratio
    current_turn_noise  = turn_noise_start*(1-ratio) + turn_noise_end*ratio
    current_sense_noise = sense_noise_start*(1-ratio) + sense_noise_end*ratio

    # Simple motion commands
    cycle = step%400
    if cycle<300:
        v_cmd = 0.08; omega_cmd = 0.0
    else:
        v_cmd = 0.0; omega_cmd = 0.4 if ((step//400)%2==0) else -0.4
    v_left = (v_cmd-(omega_cmd*AXLE_LENGTH/2))/WHEEL_RADIUS
    v_right= (v_cmd+(omega_cmd*AXLE_LENGTH/2))/WHEEL_RADIUS
    v_left = max(min(v_left,6.28),-6.28)
    v_right= max(min(v_right,6.28),-6.28)
    left_motor.setVelocity(v_left)
    right_motor.setVelocity(v_right)

    # ---------------- Odometry ----------------
    delta_s = delta_theta = 0.0
    if left_enc and right_enc:
        l = left_enc.getValue()
        r = right_enc.getValue()
        dl = (l-prev_left)*WHEEL_RADIUS
        dr = (r-prev_right)*WHEEL_RADIUS
        prev_left, prev_right = l, r
        delta_s = (dl+dr)/2
        delta_theta = (dr-dl)/AXLE_LENGTH
    else:
        dt = TIME_STEP/1000
        delta_s = v_cmd*dt
        delta_theta = omega_cmd*dt
    odom_theta += delta_theta
    odom_x += delta_s*math.cos(odom_theta)
    odom_z += delta_s*math.sin(odom_theta)

    # ---------------- True pose ----------------
    true_x = odom_x; true_z = odom_z; true_theta = odom_theta
    if gps:
        try:
            g = gps.getValues()
            true_x, true_z = g[0], g[2]
        except: pass
    if imu:
        try:
            true_theta = imu.getRollPitchYaw()[2]
        except: pass

    # ---------------- LiDAR measurements ----------------
    measurements = []
    lidar_min, lidar_max = float('nan'), float('nan')
    if lidar:
        try:
            cloud = lidar.getPointCloud()
            xs = [pt.x for pt in cloud]
            if xs:
                lidar_min = min(xs)
                lidar_max = max(xs)
            for idx, pt in enumerate(cloud):
                measurements.append((pt.x, idx % len(landmark_positions)))  # naive mapping
        except: pass

    # ---------------- Particle filter ----------------
    particles = motion_step(particles, delta_s, delta_theta, current_motion_noise, current_turn_noise)
    particles = measurement_update_range_only(particles, [(math.hypot(lx-true_x,lz-true_z),i)
                                                          for i,(lx,lz) in enumerate(landmark_positions)],
                                             current_sense_noise)
    if step % 3 == 0:
        particles = systematic_resample(particles)
    est_x, est_z, est_th = estimate_pose(particles)

    # ---------------- Console output ----------------
    rmse = math.sqrt((est_x-true_x)**2 + (est_z-true_z)**2)
    print(f"Step {step}: GPS=({true_x:.2f},{true_z:.2f}) "
          f"IMU yaw={true_theta:.3f} "
          f"Odometry=({odom_x:.3f},{odom_z:.3f}) "
          f"Predicted=({est_x:.2f},{est_z:.2f},{est_th:.3f}) "
          f"LIDAR min/max X={lidar_min:.2f}/{lidar_max:.2f} "
          f"RMSE={rmse:.3f}")

    # ---------------- Visualization ----------------
    particle_scatter.set_offsets(np.array([[p[0],p[1]] for p in particles]))
    est_dot.set_data([est_x],[est_z])
    true_dot.set_data([true_x],[true_z])
    ax.set_title(f"Step {step} | Estimated: ({est_x:.2f},{est_z:.2f}) True: ({true_x:.2f},{true_z:.2f})")
    plt.pause(0.001)

    # CSV logging
    csv_data.append([step, true_x, true_z, true_theta, est_x, est_z, est_th])

    # Snapshot
    if SAVE_RESULTS and SNAP_EVERY>0 and step%SNAP_EVERY==0:
        fig.savefig(os.path.join(RESULTS_DIR,f"mcl_step_{step}_{NUM_PARTICLES}.png"),dpi=180)

# Stop motors
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# ---------------- Save CSV and final plot ----------------
if SAVE_RESULTS:
    with open(RESULT_CSV,'w',newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Step","True_X","True_Z","True_Theta","Est_X","Est_Z","Est_Theta"])
        writer.writerows(csv_data)

    plt.ioff()
    fig2, ax2 = plt.subplots(figsize=(8,8))
    ax2.set_title(f"MCL Final - Particles and Estimate ({NUM_PARTICLES} particles)")
    ax2.set_xlabel("X [m]"); ax2.set_ylabel("Z [m]"); ax2.grid(True)
    for nm,x,z in webots_landmarks:
        ax2.scatter([x],[z],marker='s',s=100,color='orange')
        ax2.text(x+0.03,z+0.03,nm,fontsize=8)
    ax2.scatter([p[0] for p in particles],[p[1] for p in particles],s=6,alpha=0.45,color='blue')
    ax2.scatter([est_x],[est_z],marker='*',s=200,color='red',label='PF estimate')
    ax2.scatter([true_x],[true_z],marker='o',s=60,color='green',label='True pose')
    ax2.legend()
    fig2.savefig(RESULT_PLOT,dpi=200)
    plt.show(block=True)

print(f"MCL finished. Total steps: {step}")
