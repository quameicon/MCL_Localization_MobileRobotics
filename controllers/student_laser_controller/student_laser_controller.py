"""
Student Laser Beam Model Controller for E-puck Robot

MOBILE ROBOTICS LAB: Robot Perception and Sensor Models - Laser Version

Your task is to complete the beam model implementation by filling in the TODO sections.
This controller demonstrates probabilistic sensor modeling for robot navigation using
laser distance sensors instead of proximity sensors.

The beam model accounts for four types of sensor measurement errors:
1. Correct measurements with noise (z_hit)
2. Short readings due to unexpected objects (z_short)
3. Failure readings at max range (z_max)
4. Random readings due to sensor failures (z_rand)

Student: [Your Name Here]
"""

import math
import random
from controller import Robot, DistanceSensor, Motor


class BeamModel:
    """
    Probabilistic beam model for sensor measurements.

    This class implements the four-component beam model that accounts for
    different types of sensor errors and uncertainties.
    """

    def __init__(
        self,
        z_hit=0.7,
        z_short=0.1,
        z_max=0.1,
        z_rand=0.1,
        sigma_hit=0.02,
        lambda_short=2.0,
    ):
        """
        Initialize beam model parameters.

        Args:
            z_hit: Weight for correct measurements with noise (0.7 = 70%)
            z_short: Weight for short readings (0.1 = 10%)
            z_max: Weight for max range readings (0.1 = 10%)
            z_rand: Weight for random readings (0.1 = 10%)
            sigma_hit: Standard deviation for measurement noise (0.02m = 2cm)
            lambda_short: Parameter for exponential distribution of short readings
        """
        # Store parameters
        self.z_hit = z_hit
        self.z_short = z_short
        self.z_max = z_max
        self.z_rand = z_rand
        self.sigma_hit = sigma_hit
        self.lambda_short = lambda_short

        # Normalize weights to sum to 1.0
        total = z_hit + z_short + z_max + z_rand
        self.z_hit /= total
        self.z_short /= total
        self.z_max /= total
        self.z_rand /= total

        print(
            f"Beam Model initialized with weights: hit={self.z_hit:.2f}, "
            f"short={self.z_short:.2f}, max={self.z_max:.2f}, rand={self.z_rand:.2f}"
        )

    def p_hit(self, z, z_expected, z_max):
        """
        TODO 1: Calculate probability of correct measurement with Gaussian noise.

        This component models the case where the sensor reading is close to the
        expected (true) distance but with some Gaussian noise.

        Args:
            z: Actual sensor measurement
            z_expected: Expected/true distance to obstacle
            z_max: Maximum sensor range

        Returns:
            Probability density for this measurement under the hit component

        Hint: Use Gaussian probability density function:
        p = (1 / (sigma * sqrt(2*pi))) * exp(-0.5 * ((z - z_expected) / sigma)^2)

        Remember: Only return probability if z is within reasonable range [0, z_max]
        """
        # TODO: Implement this function
        # Check if measurement is in valid range
        if z < 0 or z > z_max:
            return 0.0

        # Your code here - calculate Gaussian probability density
        p = (1 / (self.sigma_hit * math.sqrt(2 * math.pi))) * math.exp(-0.5 * ((z - z_expected) / self.sigma_hit)**2)
        # Use self.sigma_hit as the standard deviation

        return p  # Replace this line with your implementation

    def p_short(self, z, z_expected, z_max):
        """
        TODO 2: Calculate probability of short readings using exponential distribution.

        This component models cases where the sensor reading is shorter than expected,
        often due to unexpected objects, cross-talk, or specular reflections.

        Args:
            z: Actual sensor measurement
            z_expected: Expected/true distance to obstacle
            z_max: Maximum sensor range

        Returns:
            Probability density for this measurement under the short component

        Hint: For z < z_expected, use exponential distribution:
        p = lambda * exp(-lambda * z) / (1 - exp(-lambda * z_expected))
        The denominator normalizes the distribution over [0, z_expected]
        """
        # TODO: Implement this function
        if z < 0 or z > z_max or z >= z_expected:
            return 0.0

        # Your code here - calculate exponential probability density
        u = self.lambda_short * math.exp(-1 * self.lambda_short * z)
        d = 1 - math.exp(-1 * self.lambda_short * z_expected)
        p = u / d
        # Use self.lambda_short as the lambda parameter
        # Remember to normalize over the range [0, z_expected]

        return p  # Replace this line with your implementation


    def p_max(self, z, z_expected, z_max):

        tolerance = 0.01  # Small threshold to handle float inaccuracies
    
    # If measurement is very close to z_max → return high probability
        if abs(z - z_max) < tolerance:
            return 1.0
        else:
            return 0.0
    def p_rand(self, z, z_expected, z_max):
        """
        TODO 4: Calculate probability of random readings.

        This component models completely random sensor failures that can
        return any value within the sensor's range with equal probability.

        Args:
            z: Actual sensor measurement
            z_expected: Expected/true distance to obstacle
            z_max: Maximum sensor range

        Returns:
            Probability density for this measurement under the random component

        Hint: Uniform distribution over [0, z_max] has probability density 1/z_max
        """
        # TODO: Implement this function
        if z < 0 or z > z_max:
            return 0.0

        # Your code here - uniform distribution probability density

        return 0.0  # Replace this line with your implementation

    def beam_model_prob(self, z, z_expected, z_max):
        """
        TODO 5: Calculate total beam model probability by combining all components.

        This is the main function that combines all four error components
        using the learned weights to compute the overall probability.

        Args:
            z: Actual sensor measurement
            z_expected: Expected/true distance to obstacle
            z_max: Maximum sensor range

        Returns:
            Total probability of this measurement according to the beam model

        Hint: Weighted sum of all components:
        p_total = z_hit * p_hit + z_short * p_short + z_max * p_max + z_rand * p_rand
        """
        # TODO: Implement this function
        # Calculate each component probability
        hit_prob = self.p_hit(z, z_expected, z_max)
        short_prob = self.p_short(z, z_expected, z_max)
        max_prob = self.p_max(z, z_expected, z_max)
        rand_prob = self.p_rand(z, z_expected, z_max)

        # Your code here - combine using weights
        total_prob = 0.0  # Replace with weighted combination

        return total_prob


class StudentLaserController:
    """
    Robot controller that uses beam model for laser sensor-based navigation.
    """

    def __init__(self):
        # Initialize robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Initialize laser distance sensors
        self.laser_sensors = []
        sensor_names = [
            "laser0",
            "laser1",
            "laser2",
            "laser3",
            "laser4",
            "laser5",
            "laser6",
            "laser7",
        ]

        for name in sensor_names:
            sensor = self.robot.getDevice(name)
            sensor.enable(self.timestep)
            self.laser_sensors.append(sensor)

        self.sensor_lookup_tables = [
            self.robot.getDevice(sensor).getLookupTable() for sensor in sensor_names
        ]

        # Initialize GPS and Compass for robot pose
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timestep)
        self.compass = self.robot.getDevice("compass")
        self.compass.enable(self.timestep)

        # Initialize motors
        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        # Initialize beam model
        self.beam_model = BeamModel()

        # Robot parameters
        self.max_speed = 6.28  # rad/s
        self.sensor_max_range = 1.0  # 1m for laser distance sensors

        # Navigation parameters
        self.probability_threshold = 0.5  # Threshold for trusting sensor readings
        self.min_obstacle_distance = 0.3  # 30cm minimum distance for laser sensors

        # Environment obstacles (from world file)
        self.obstacles = [
            {"type": "box", "position": [0.3, 0.3], "size": [0.2, 0.2]},  # Red box
            {"type": "box", "position": [-0.4, 0.2], "size": [0.15, 0.3]},  # Green box
            {"type": "box", "position": [0.6, -0.3], "size": [0.25, 0.15]},  # Blue box
            {"type": "box", "position": [0, 0.6], "size": [0.8, 0.1]},  # Purple wall
            {"type": "box", "position": [-0.7, -0.1], "size": [0.1, 0.4]},  # Cyan wall
            {
                "type": "cylinder",
                "position": [-0.2, -0.4],
                "radius": 0.08,
            },  # Yellow pipe
            # Arena walls
            {"type": "wall", "position": [1.0, 0], "size": [0.1, 2.0]},  # East wall
            {"type": "wall", "position": [-1.0, 0], "size": [0.1, 2.0]},  # West wall
            {"type": "wall", "position": [0, 1.0], "size": [2.0, 0.1]},  # North wall
            {"type": "wall", "position": [0, -1.0], "size": [2.0, 0.1]},  # South wall
        ]

        # Sensor angles relative to robot (laser sensor configuration)
        self.sensor_angles = [
            0.0,  # laser0 - front
            0.785,  # laser1 - front-right
            1.571,  # laser2 - right
            2.356,  # laser3 - back-right
            3.142,  # laser4 - back
            3.927,  # laser5 - back-left
            4.712,  # laser6 - left
            5.498,  # laser7 - front-left
        ]

        print("Student Laser Controller initialized!")
        print("Sensor max range:", self.sensor_max_range, "meters")

    def sensor_value_to_distance(self, sensor_value, sensor_index):
        """
        Convert raw sensor value to distance in meters.
        Laser distance sensors typically return distance directly in meters.
        """
        if sensor_value <= 0 or sensor_value >= 1000:
            return self.sensor_max_range

        # Helper: linear interpolation on a sorted table of (raw, dist) pairs
        def interp_lookup(raw, table):
            for i in range(len(table) // 3 - 1):
                x0, y0 = table[i * 3], table[i * 3 + 1]
                x1, y1 = table[(i + 1) * 3], table[(i + 1) * 3 + 1]
                if y0 >= raw >= y1:
                    t = (raw - y0) / (y1 - y0) if y1 != y0 else 0.0
                    return x0 + t * (x1 - x0)
            return None

        table = self.sensor_lookup_tables[sensor_index]

        interpolated = interp_lookup(sensor_value, table)
        return max(0.0, min(interpolated, self.sensor_max_range))

    def get_robot_pose(self):
        """
        Get current robot position and orientation from GPS and Compass sensors.
        """
        # Get position from GPS
        gps_values = self.gps.getValues()
        robot_x = gps_values[0]
        robot_y = gps_values[1]

        # Get orientation from Compass
        compass_values = self.compass.getValues()
        # Calculate angle from compass vector (north is compass_values[0], east is compass_values[1])
        robot_theta = math.atan2(compass_values[0], compass_values[1])

        return [robot_x, robot_y, robot_theta]  # x, y, theta

    def get_expected_distance(self, sensor_index):
        """
        Calculate expected distance by ray casting in the direction of the specified sensor.
        Uses axis-aligned rectangle (box/wall) and circle (cylinder) intersection tests
        to find the nearest obstacle along the sensor ray, limited by sensor_max_range.
        """
        robot_x, robot_y, robot_theta = self.get_robot_pose()

        # Direction of this sensor in world frame (robot_theta + sensor relative angle)
        ray_angle = robot_theta + self.sensor_angles[sensor_index]
        dx = math.cos(ray_angle)
        dy = math.sin(ray_angle)
        max_r = self.sensor_max_range

        # Helper: ray vs axis-aligned bounding box (AABB) using slab method.
        def ray_intersect_aabb(xmin, xmax, ymin, ymax, ox, oy, dx, dy):
            # Returns the smallest non-negative t such that (ox,oy) + t*(dx,dy) intersects the box,
            # or None if no intersection within positive ray.
            # Handle near-zero directions
            EPS = 1e-9

            if abs(dx) < EPS:
                if ox < xmin or ox > xmax:
                    return None
                tx_min = -math.inf
                tx_max = math.inf
            else:
                tx1 = (xmin - ox) / dx
                tx2 = (xmax - ox) / dx
                tx_min = min(tx1, tx2)
                tx_max = max(tx1, tx2)

            if abs(dy) < EPS:
                if oy < ymin or oy > ymax:
                    return None
                ty_min = -math.inf
                ty_max = math.inf
            else:
                ty1 = (ymin - oy) / dy
                ty2 = (ymax - oy) / dy
                ty_min = min(ty1, ty2)
                ty_max = max(ty1, ty2)

            t_enter = max(tx_min, ty_min)
            t_exit = min(tx_max, ty_max)

            # Valid intersection if exit >= enter and exit >= 0
            if t_exit < max(t_enter, 0.0):
                return None

            # If origin is inside box, t_enter may be negative -> distance 0
            if t_enter < 0.0 <= t_exit:
                return 0.0

            return t_enter if t_enter >= 0.0 else None

        # Helper: ray vs circle (cylinder top-down)
        def ray_intersect_circle(cx, cy, radius, ox, oy, dx, dy):
            # Solve quadratic for intersection of ray with circle centered at (cx,cy)
            # (ox + t*dx - cx)^2 + (oy + t*dy - cy)^2 = radius^2
            ocx = ox - cx
            ocy = oy - cy
            a = dx * dx + dy * dy
            b = 2 * (dx * ocx + dy * ocy)
            c = ocx * ocx + ocy * ocy - radius * radius

            discriminant = b * b - 4 * a * c
            if discriminant < 0:
                return None

            sqrt_d = math.sqrt(discriminant)
            t1 = (-b - sqrt_d) / (2 * a)
            t2 = (-b + sqrt_d) / (2 * a)

            # We want smallest non-negative t
            candidates = [t for t in (t1, t2) if t >= 0.0]
            if not candidates:
                # If both negative, ray intersects circle behind origin
                return None
            return min(candidates)

        min_distance = max_r

        # Cast ray against each obstacle
        for obstacle in self.obstacles:
            t_hit = None

            if obstacle["type"] == "box" or obstacle["type"] == "wall":
                box_x = obstacle["position"][0]
                box_y = obstacle["position"][1]
                half_width = obstacle["size"][0] / 2.0
                half_height = obstacle["size"][1] / 2.0

                xmin = box_x - half_width
                xmax = box_x + half_width
                ymin = box_y - half_height
                ymax = box_y + half_height

                t_hit = ray_intersect_aabb(
                    xmin, xmax, ymin, ymax, robot_x, robot_y, dx, dy
                )

            elif obstacle["type"] == "cylinder":
                cx = obstacle["position"][0]
                cy = obstacle["position"][1]
                radius = obstacle["radius"]
                t_hit = ray_intersect_circle(cx, cy, radius, robot_x, robot_y, dx, dy)

            # If we got a hit and it's within sensor range, update min_distance
            if t_hit is not None and t_hit <= max_r:
                if t_hit < min_distance:
                    min_distance = t_hit

        return min_distance

    def distance_to_obstacle(self, robot_x, robot_y, obstacle):
        """
        Calculate Euclidean distance from robot to nearest point on obstacle.
        """
        if obstacle["type"] == "box" or obstacle["type"] == "wall":
            return self.distance_to_box(robot_x, robot_y, obstacle)
        elif obstacle["type"] == "cylinder":
            return self.distance_to_cylinder(robot_x, robot_y, obstacle)
        return float("inf")

    def distance_to_box(self, robot_x, robot_y, box):
        """
        Calculate distance from robot to nearest point on box.
        """
        box_x = box["position"][0]
        box_y = box["position"][1]
        half_width = box["size"][0] / 2
        half_height = box["size"][1] / 2

        # Find closest point on box to robot
        closest_x = max(box_x - half_width, min(robot_x, box_x + half_width))
        closest_y = max(box_y - half_height, min(robot_y, box_y + half_height))

        # Calculate distance
        dx = robot_x - closest_x
        dy = robot_y - closest_y
        return math.sqrt(dx * dx + dy * dy)

    def distance_to_cylinder(self, robot_x, robot_y, cylinder):
        """
        Calculate distance from robot to nearest point on cylinder.
        """
        cx = cylinder["position"][0]
        cy = cylinder["position"][1]
        radius = cylinder["radius"]

        # Distance from robot to cylinder center
        dx = robot_x - cx
        dy = robot_y - cy
        center_distance = math.sqrt(dx * dx + dy * dy)

        # Distance to cylinder surface
        return max(0, center_distance - radius)

    def analyze_sensors(self):
        """
        Analyze all sensor readings using the beam model.
        Print detailed analysis for educational purposes.
        """
        print("\n=== Laser Sensor Analysis ===")

        for i, sensor in enumerate(self.laser_sensors):
            raw_value = sensor.getValue()
            measured_distance = self.sensor_value_to_distance(raw_value, i)
            expected_distance = self.get_expected_distance(i)

            # Calculate beam model probability
            beam_prob = self.beam_model.beam_model_prob(
                measured_distance, expected_distance, self.sensor_max_range
            )

            # Calculate individual components for educational insight
            p_hit = self.beam_model.p_hit(
                measured_distance, expected_distance, self.sensor_max_range
            )
            p_short = self.beam_model.p_short(
                measured_distance, expected_distance, self.sensor_max_range
            )
            p_max = self.beam_model.p_max(
                measured_distance, expected_distance, self.sensor_max_range
            )
            p_rand = self.beam_model.p_rand(
                measured_distance, expected_distance, self.sensor_max_range
            )

            print(
                f"Sensor {i}: Raw={raw_value:4.0f}, Dist={measured_distance:.3f}m, "
                f"Expected={expected_distance:.3f}m, BeamProb={beam_prob:.3f}"
            )
            print(
                f"  Components: hit={p_hit:.3f}, short={p_short:.3f}, "
                f"max={p_max:.3f}, rand={p_rand:.3f}"
            )

    def simple_navigation(self):
        """
        Simple obstacle avoidance without beam model (for comparison).
        """
        # Get front sensor readings
        front_left = self.sensor_value_to_distance(self.laser_sensors[0].getValue(), 0)
        front_right = self.sensor_value_to_distance(self.laser_sensors[7].getValue(), 7)

        # Simple threshold-based avoidance - increased threshold for laser sensors
        obstacle_threshold = 0.3  # 30cm for laser sensors

        if front_left < obstacle_threshold or front_right < obstacle_threshold:
            # Turn away from obstacle
            if front_left < front_right:
                self.left_motor.setVelocity(0.5 * self.max_speed)
                self.right_motor.setVelocity(-0.5 * self.max_speed)
            else:
                self.left_motor.setVelocity(-0.5 * self.max_speed)
                self.right_motor.setVelocity(0.5 * self.max_speed)
        else:
            # Move forward
            self.left_motor.setVelocity(0.5 * self.max_speed)
            self.right_motor.setVelocity(0.5 * self.max_speed)

    def navigate_with_beam_model(self):
        """
        TODO 6: Implement navigation using beam model probabilities.

        Use the beam model to make more intelligent navigation decisions
        that account for sensor uncertainty and reliability.

        Hint:
        - Calculate beam model probabilities for key sensors
        - Weight navigation decisions by probability/confidence
        - Consider sensor reliability when making turns
        - Use probability thresholds instead of distance thresholds
        """
        # Get sensor readings and calculate beam model probabilities
        front_sensors = [0, 7]  # Front left and front right

        # TODO: Calculate beam model probabilities for front sensors
        # front_probs = []
        # for sensor_idx in front_sensors:
        #     raw_value = self.laser_sensors[sensor_idx].getValue()
        #     measured_dist = self.sensor_value_to_distance(raw_value, i)
        #     expected_dist = self.get_expected_distance(sensor_idx)
        #     prob = self.beam_model.beam_model_prob(measured_dist, expected_dist, self.sensor_max_range)
        #     front_probs.append(prob)

        # TODO: Implement probability-based navigation logic
        # Consider:
        # - How reliable are the sensor readings? (high beam model probability = more reliable)
        # - Should you trust a low-probability reading?
        # - How to combine multiple sensor probabilities?

        # For now, fall back to simple navigation
        # Replace this with your beam model navigation
        self.simple_navigation()

        # TODO: Print your navigation decision and reasoning
        # print(f"Navigation Decision: [your decision] (reasoning: [your reasoning])")

    def run(self):
        """
        Main control loop.
        """
        step_count = 0

        print("Starting Student Laser Controller...")
        print("Watch the console for sensor analysis and navigation decisions!")

        while self.robot.step(self.timestep) != -1:
            self.analyze_sensors()
            # Navigate using beam model
            self.navigate_with_beam_model()


# Create and run the controller
if __name__ == "__main__":
    controller = StudentLaserController()
    controller.run()
