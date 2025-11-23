# figure8_controller_mc_avoidance.py
from controller import Robot
import math, csv, os, random

# --- Robot parameters ---
wheel_radius = 0.033   # meters
wheel_base = 0.15      # meters

# --- Motion parameters ---
radius = 0.2           # radius of each loop (meters)
period = 30.0          # seconds per full loop
duration = 60.0        # total run time (longer to test avoidance)
linear_vel = 0.1       # m/s

# --- Inverse kinematics ---
def inverse_kinematics(v, omega):
    v_left = (2.0 * v - omega * wheel_base) / (2.0 * wheel_radius)
    v_right = (2.0 * v + omega * wheel_base) / (2.0 * wheel_radius)
    return v_left, v_right

# --- Initialize robot ---
robot = Robot()
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_encoder = robot.getDevice("left wheel sensor")
right_encoder = robot.getDevice("right wheel sensor")
left_encoder.enable(timestep)
right_encoder.enable(timestep)

# --- Enable proximity sensors ---
sensor_names = [f"ps{i}" for i in range(8)]
sensors = []
for name in sensor_names:
    s = robot.getDevice(name)
    s.enable(timestep)
    sensors.append(s)

# --- Odometry state ---
x, y, theta = 0.0, 0.0, 0.0
prev_left = 0.0
prev_right = 0.0

# --- Logging setup ---
log = []
save_path = os.path.expanduser("~/Desktop/trajectory_mc.csv")

# --- Monte Carlo obstacle avoidance ---
def monte_carlo_avoidance(sensor_values, num_samples=50):
    """
    Sample steering angles and choose the safest direction.
    Returns an angular velocity modifier (omega_offset).
    """
    # Normalize sensor readings
    front_left = (sensor_values[7] + sensor_values[6]) / 2
    front_right = (sensor_values[0] + sensor_values[1]) / 2
    front = max(front_left, front_right)

    # Threshold for obstacle detection
    if front < 100:
        return 0.0  # no obstacle nearby

    # Monte Carlo sampling: try random small turns
    candidates = []
    for _ in range(num_samples):
        # Sample angle between -π/3 and +π/3 radians
        delta_angle = random.uniform(-math.pi / 3, math.pi / 3)
        # Estimate "collision risk" based on sensor readings and direction
        # Turning toward side with higher distance is safer
        risk = (
            front_left if delta_angle < 0 else front_right
        )  # smaller reading = higher risk
        candidates.append((delta_angle, risk))

    # Choose turn direction with maximum "safe distance" (lowest risk)
    best_turn = max(candidates, key=lambda c: c[1])
    omega_offset = best_turn[0] * 0.5  # scale down for smoother motion
    return omega_offset

# --- Main control loop ---
start_time = robot.getTime()
while robot.step(timestep) != -1:
    current_time = robot.getTime()
    elapsed_time = current_time - start_time
    if elapsed_time > duration:
        break

    # Base figure-eight angular velocity
    t = (elapsed_time % period) * 2.0 * math.pi / period
    angular_vel = linear_vel / radius if t < math.pi else -linear_vel / radius

    # Get sensor readings
    sensor_values = [s.getValue() for s in sensors]

    # --- Monte Carlo avoidance adjustment ---
    omega_offset = monte_carlo_avoidance(sensor_values)
    angular_vel += omega_offset

    # Compute wheel velocities
    vL, vR = inverse_kinematics(linear_vel, angular_vel)
    left_motor.setVelocity(vL)
    right_motor.setVelocity(vR)

    # --- Odometry update ---
    left_pos = left_encoder.getValue()
    right_pos = right_encoder.getValue()
    dL = (left_pos - prev_left) * wheel_radius
    dR = (right_pos - prev_right) * wheel_radius
    prev_left, prev_right = left_pos, right_pos
    d = (dL + dR) / 2.0
    dtheta = (dR - dL) / wheel_base
    theta += dtheta
    x += d * math.cos(theta)
    y += d * math.sin(theta)

    # Log data
    log.append([elapsed_time, x, y, theta, omega_offset])

# Stop robot
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Save trajectory
with open(save_path, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['time', 'x', 'y', 'theta', 'omega_offset'])
    writer.writerows(log)

print(f"Trajectory data saved to {save_path}")
