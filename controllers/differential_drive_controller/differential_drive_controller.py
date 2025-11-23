# figure8_controller.py
from controller import Robot
import math, csv, os

# --- Robot parameters ---
wheel_radius = 0.033   # meters
wheel_base = 0.15      # meters (distance between wheels)

# --- Motion parameters ---
radius = 0.2           # radius of each loop (meters)
period = 30.0          # seconds per full loop
duration = 29.0        # total run time
linear_vel = 0.1       # m/s

# --- Inverse kinematics ---
def inverse_kinematics(v, omega):
    v_left = (2.0 * v - omega * wheel_base) / (2.0 * wheel_radius)
    v_right = (2.0 * v + omega * wheel_base) / (2.0 * wheel_radius)
    return v_left, v_right

# --- Init robot ---
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

# --- Odometry state ---
x, y, theta = 0.0, 0.0, 0.0
prev_left = 0.0
prev_right = 0.0

# --- Logging ---
log = []
save_dir = os.path.expanduser("~/Desktop")
os.makedirs(save_dir, exist_ok=True)  # Ensure Desktop directory exists
save_path = os.path.join(save_dir, "trajectory.csv")

# Print current working directory for debugging
print("Current working directory:", os.getcwd())
print(f"Attempting to save trajectory to: {save_path}")

# --- Main loop ---
start_time = robot.getTime()
while robot.step(timestep) != -1:
    current_time = robot.getTime()
    elapsed_time = current_time - start_time
    if elapsed_time > duration:
        break

    # phase in [0, 2π)
    t = (elapsed_time % period) * 2.0 * math.pi / period
    if t < math.pi:
        angular_vel = linear_vel / radius   # CCW
    else:
        angular_vel = -linear_vel / radius  # CW

    # wheel velocities
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

    log.append([elapsed_time, x, y, theta])

# stop robot
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# save log
try:
    with open(save_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'x', 'y', 'theta'])
        writer.writerows(log)
    print(f"Trajectory data saved to {save_path}")
except Exception as e:
    print(f"Failed to save trajectory data to {save_path}: {e}")
    # Fallback to current directory
    fallback_path = "trajectory.csv"
    print(f"Attempting to save to fallback path: {fallback_path}")
    try:
        with open(fallback_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'x', 'y', 'theta'])
            writer.writerows(log)
        print(f"Trajectory data saved to {fallback_path}")
    except Exception as e:
        print(f"Failed to save to fallback path: {e}")
