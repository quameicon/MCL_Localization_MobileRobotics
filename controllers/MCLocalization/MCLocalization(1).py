# File: MCLLocalization.py
"""
Main controller for Monte Carlo Localization (professional version).
Place this file and the companion modules in the same controller folder inside Webots.
Files included in this textdoc:
- MCLLocalization.py  (this file, main loop)
- motion_model.py
- sensor_model.py
- resampling.py
- map_utils.py
- utils.py
- requirements.txt (at bottom)

Notes:
- This controller is written for Webots R2025a and a Pioneer3dx robot with devices named exactly:
  GPS, InertialUnit, Lidar, left wheel motor "left wheel motor", right wheel motor "right wheel motor"
  Adjust device names to match your robot if different.
- The code is modular and well commented. It's ready to run but you may want to tune parameters.
"""

from controller import Robot, Motor
import math, time, os
from motion_model import apply_odometry_to_particles
from sensor_model import update_weights_with_lidar
from resampling import systematic_resample
from map_utils import LikelihoodField
import utils

# ------------------ Configuration ------------------
TIME_STEP = 64  # ms
NUM_PARTICLES = 500
WHEEL_RADIUS = 0.0975  # meters (Pioneer typical)
WHEEL_BASE = 0.331  # meters (distance between wheels)
MAX_LIDAR_RANGE = 6.0
LOG_DIR = 'results'
# ---------------------------------------------------


def ensure_log_dir():
    if not os.path.exists(LOG_DIR):
        os.makedirs(LOG_DIR)


class MCLController:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep()) if self.robot.getBasicTimeStep() else TIME_STEP
        # Devices
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        self.imu = self.robot.getDevice('inertial unit')
        self.imu.enable(self.timestep)
        self.lidar = self.robot.getDevice('lidar')
        self.lidar.enable(self.timestep)
        # enable point cloud for some webots versions
        try:
            self.lidar.enablePointCloud()
        except Exception:
            pass
        # Motors / encoders
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_enc = self.robot.getDevice('left wheel sensor')
        self.right_enc = self.robot.getDevice('right wheel sensor')
        self.left_enc.enable(self.timestep)
        self.right_enc.enable(self.timestep)
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        # Particles state
        self.num_particles = NUM_PARTICLES
        self.particles = utils.create_uniform_particles(self.num_particles, x_range=(-3.0,3.0), y_range=(-3.0,3.0))

        # previous encoder values for odometry
        self.prev_left = self.left_enc.getValue()
        self.prev_right = self.right_enc.getValue()

        # Map / likelihood field
        # This creates an internal likelihood field using an occupancy grid. You can provide your own map if you like.
        self.likelihood = LikelihoodField(cell_size=0.05, xlim=(-3.5,3.5), ylim=(-3.5,3.5))

        # Logging
        ensure_log_dir()
        self.logfile = open(os.path.join(LOG_DIR, 'mcl_log.csv'), 'w')
        self.logfile.write('time,x_est,y_est,theta_est,x_gps,y_gps\n')

    def step(self):
        return self.robot.step(self.timestep)

    def run(self):
        start_time = self.robot.getTime()
        while self.step() != -1:
            # read sensors
            gps_v = self.gps.getValues()
            x_gps = gps_v[0]
            y_gps = gps_v[2]
            rpy = self.imu.getRollPitchYaw()
            yaw = rpy[2]
            ranges = self.lidar.getRangeImage()

            # odometry
            cur_left = self.left_enc.getValue()
            cur_right = self.right_enc.getValue()
            delta_left = cur_left - self.prev_left
            delta_right = cur_right - self.prev_right
            self.prev_left = cur_left
            self.prev_right = cur_right

            # motion update
            apply_odometry_to_particles(self.particles, delta_left, delta_right,
                                        wheel_radius=WHEEL_RADIUS, wheel_base=WHEEL_BASE,
                                        noise_std={'rot':0.02, 'trans':0.01})

            # measurement update (lidar)
            update_weights_with_lidar(self.particles, ranges, self.likelihood,
                                      max_range=MAX_LIDAR_RANGE, sigma=0.2, beam_step=4)

            # normalize weights
            utils.normalize_particles(self.particles)

            # resample
            self.particles = systematic_resample(self.particles)

            # estimate pose
            x_est, y_est, theta_est = utils.estimate_pose(self.particles)

            # logging
            t = self.robot.getTime() - start_time
            self.logfile.write(f"{t:.3f},{x_est:.4f},{y_est:.4f},{theta_est:.4f},{x_gps:.4f},{y_gps:.4f}\n")

            # simple explorer behavior to make robot move during experiments (VERY simple)
            self.left_motor.setVelocity(2.0)
            self.right_motor.setVelocity(2.0)

        self.logfile.close()


if __name__ == '__main__':
    controller = MCLController()
    controller.run()


# ------------------------------------------------------------------
# File: motion_model.py
"""
Odometry-based motion model for differential-drive robot.
Applies disturbance/noise and updates particle poses in place.
"""
import math, random

def apply_odometry_to_particles(particles, delta_left, delta_right, wheel_radius, wheel_base, noise_std=None):
    """
    delta_left/right are wheel rotations (radians) measured from encoders for the last timestep.
    We convert to linear displacements using wheel_radius.
    particles: list of dicts with keys 'x','y','theta','w'
    noise_std: dict with 'rot' and 'trans' standard deviations
    """
    if noise_std is None:
        noise_std = {'rot':0.01, 'trans':0.01}
    dl = delta_left * wheel_radius
    dr = delta_right * wheel_radius
    dc = (dl + dr) / 2.0
    dtheta = (dr - dl) / wheel_base

    for p in particles:
        # add noise sampled per-particle
        trans_noise = random.gauss(0, noise_std['trans'])
        rot_noise = random.gauss(0, noise_std['rot'])
        # apply motion
        p['x'] += (dc + trans_noise) * math.cos(p['theta'])
        p['y'] += (dc + trans_noise) * math.sin(p['theta'])
        p['theta'] = utils_angle_normalize(p['theta'] + dtheta + rot_noise)


def utils_angle_normalize(a):
    while a > math.pi:
        a -= 2*math.pi
    while a <= -math.pi:
        a += 2*math.pi
    return a


# ------------------------------------------------------------------
# File: sensor_model.py
"""
Measurement model using a precomputed likelihood field and (coarsely) simulating lidar beams.
This implementation uses a simple projection: for a subset of beams, compute expected endpoint
and lookup the likelihood from the likelihood field. It's optimized for speed and clarity.
"""
import math
import utils


def update_weights_with_lidar(particles, ranges, likelihood_field, max_range=6.0, sigma=0.2, beam_step=1):
    """
    particles: list of {'x','y','theta','w'}
    ranges: array-like lidar ranges in Webots order (0..N-1)
    likelihood_field: LikelihoodField instance with method likelihood_at(x,y)
    sigma: measurement noise for gaussian
    beam_step: skip beams to speed up (e.g., 4 means use every 4th beam)
    """
    nbeams = len(ranges)
    for p in particles:
        weight = 1.0
        # iterate over subset of beams
        for i in range(0, nbeams, beam_step):
            r = ranges[i]
            if r == float('inf') or r <= 0 or r > max_range:
                continue
            # compute beam angle relative to robot
            # Webots lidar 0-based index angles go from -fov/2 to +fov/2 typically; but to stay robust we compute from index
            angle = (i / nbeams) * 2.0*math.pi - math.pi  # approximate for 360deg lidars
            # transform to global
            bx = p['x'] + r * math.cos(p['theta'] + angle)
            by = p['y'] + r * math.sin(p['theta'] + angle)
            # use likelihood field
            q = likelihood_field.likelihood_at(bx, by, sigma)
            weight *= q
            # avoid weights going to zero
            if weight == 0:
                weight = 1e-300
        p['w'] = weight


# ------------------------------------------------------------------
# File: resampling.py
"""
Systematic (low-variance) resampling implementation.
"""
import random


def systematic_resample(particles):
    N = len(particles)
    weights = [p['w'] for p in particles]
    # normalize weights safely
    total = sum(weights)
    if total <= 0:
        # reset equal weights
        for p in particles:
            p['w'] = 1.0 / N
        return particles.copy()
    weights = [w/total for w in weights]
    positions = [(random.random() + i) / N for i in range(N)]
    indexes = []
    cumsum = 0.0
    i = 0
    cumw = weights[0]
    for pos in positions:
        while pos > cumw:
            i += 1
            cumw += weights[i]
        indexes.append(i)
    # create new particle set
    new_particles = []
    for idx in indexes:
        p = particles[idx].copy()
        p['w'] = 1.0 / N
        new_particles.append(p)
    return new_particles


# ------------------------------------------------------------------
# File: map_utils.py
"""
Simple likelihood field implementation.
- Builds a coarse occupancy grid and precomputes distance transform (Euclidean)
- Provides likelihood_at(x,y,sigma) returning a probability-like value
"""
import numpy as np
from scipy import ndimage

class LikelihoodField:
    def __init__(self, cell_size=0.05, xlim=(-3.5,3.5), ylim=(-3.5,3.5)):
        self.cell_size = cell_size
        self.xmin, self.xmax = xlim
        self.ymin, self.ymax = ylim
        self.nx = int((self.xmax - self.xmin) / cell_size)
        self.ny = int((self.ymax - self.ymin) / cell_size)
        # for simplicity start with empty occupancy and add walls/boxes heuristically
        self.occupancy = np.zeros((self.nx, self.ny), dtype=np.uint8)
        # add boundary walls
        self._draw_boundary()
        # add some internal obstacles if desired (this is placeholder — you can use a real map)
        self._draw_sample_obstacles()
        # compute distance transform
        self._compute_distance_transform()

    def _draw_boundary(self):
        self.occupancy[0,:] = 1
        self.occupancy[-1,:] = 1
        self.occupancy[:,0] = 1
        self.occupancy[:,-1] = 1

    def _draw_sample_obstacles(self):
        # draw three boxes roughly matching the world layout — tune or replace with your own
        def world_to_idx(x,y):
            ix = int((x - self.xmin) / self.cell_size)
            iy = int((y - self.ymin) / self.cell_size)
            return ix, iy
        for cx, cy, size in [(-1.0, -2.4, 0.5), (1.9, -0.65, 0.5), (0.0, 2.4, 0.5)]:
            half = int((size/2.0) / self.cell_size)
            ix, iy = world_to_idx(cx, cy)
            self.occupancy[max(0,ix-half):min(self.nx,ix+half), max(0,iy-half):min(self.ny,iy+half)] = 1

    def _compute_distance_transform(self):
        # compute distance to nearest occupied cell (in meters)
        occupied = self.occupancy == 1
        dt = ndimage.distance_transform_edt(~occupied) * self.cell_size
        self.distance = dt

    def likelihood_at(self, x, y, sigma=0.2):
        # if outside map return small probability
        if x < self.xmin or x >= self.xmax or y < self.ymin or y >= self.ymax:
            return 0.01
        ix = int((x - self.xmin) / self.cell_size)
        iy = int((y - self.ymin) / self.cell_size)
        d = float(self.distance[ix, iy])
        # gaussian of the distance
        p = math.exp(-(d*d) / (2.0 * sigma * sigma))
        return max(p, 1e-6)


# ------------------------------------------------------------------
# File: utils.py
"""
Common utilities: particle creation, normalization, pose estimation, angle helpers.
"""
import math, random


def create_uniform_particles(N, x_range=(-3,3), y_range=(-3,3)):
    particles = []
    for _ in range(N):
        particles.append({'x': random.uniform(*x_range),
                          'y': random.uniform(*y_range),
                          'theta': random.uniform(-math.pi, math.pi),
                          'w': 1.0/N})
    return particles


def normalize_particles(particles):
    total = sum(p['w'] for p in particles)
    if total <= 0:
        N = len(particles)
        for p in particles:
            p['w'] = 1.0/N
        return
    for p in particles:
        p['w'] /= total


def estimate_pose(particles):
    x = 0.0
    y = 0.0
    sin_sum = 0.0
    cos_sum = 0.0
    for p in particles:
        w = p['w']
        x += p['x'] * w
        y += p['y'] * w
        sin_sum += math.sin(p['theta']) * w
        cos_sum += math.cos(p['theta']) * w
    theta = math.atan2(sin_sum, cos_sum)
    return x, y, theta


def angle_normalize(a):
    while a > math.pi:
        a -= 2*math.pi
    while a <= -math.pi:
        a += 2*math.pi
    return a


# ------------------------------------------------------------------
# File: requirements.txt
numpy
scipy

# End of textdoc
