from controller import Robot
import math
import numpy as np
import matplotlib.pyplot as plt

TIME_STEP = 32

robot = Robot()

# --------------------- GET LIDAR ---------------------
lidar = robot.getDevice("lidar")
lidar.enable(TIME_STEP)
lidar.enablePointCloud()      # <---- IMPORTANT

# --------------------- GPS + IMU ---------------------
gps = robot.getDevice("gps")
gps.enable(TIME_STEP)

imu = robot.getDevice("inertial_unit")
imu.enable(TIME_STEP)

# --------------------- LIDAR PROPERTIES ---------------------
fov = lidar.getFov()                # radians
res = lidar.getHorizontalResolution()
angles = np.linspace(-fov/2, fov/2, res)

print("LiDAR FOV:", fov, "Resolution:", res)

# --------------------- REAL-TIME PLOT ---------------------
plt.ion()
fig, ax = plt.subplots(figsize=(7,7))
sc = ax.scatter([], [], s=5)
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.set_title("LiDAR Point Cloud")
ax.set_xlabel("X")
ax.set_ylabel("Z")
ax.grid(True)

# ------------------------------------------------------
# Convert lidar range + angle to world coordinates
# ------------------------------------------------------
def lidar_to_world(ranges, robot_x, robot_z, robot_theta):
    xs = []
    zs = []

    for r, ang in zip(ranges, angles):
        if r <= 0 or r > 50:     # ignore invalid beams
            continue

        # Local (robot frame)
        lx = r * math.cos(ang)
        lz = r * math.sin(ang)

        # Rotate into world frame
        wx = robot_x + (lx * math.cos(robot_theta) - lz * math.sin(robot_theta))
        wz = robot_z + (lx * math.sin(robot_theta) + lz * math.cos(robot_theta))

        xs.append(wx)
        zs.append(wz)

    return xs, zs


# --------------------- MAIN LOOP ---------------------
while robot.step(TIME_STEP) != -1:

    # robot pose from GPS + IMU
    pos = gps.getValues()
    robot_x = pos[0]
    robot_z = pos[2]
    robot_theta = imu.getRollPitchYaw()[2]   # yaw

    # --- get LiDAR ranges ---
    ranges = lidar.getRangeImage()

    # Debug: print nearest obstacle
    print("Min range:", min(ranges))

    # --- convert to world points ---
    xs, zs = lidar_to_world(ranges, robot_x, robot_z, robot_theta)

    # --- update plot ---
    sc.set_offsets(np.column_stack((xs, zs)))
    ax.set_title(f"LiDAR Point Cloud — {len(xs)} points")
    plt.pause(0.001)
