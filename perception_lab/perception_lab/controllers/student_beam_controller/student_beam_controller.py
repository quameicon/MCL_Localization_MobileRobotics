import math
import random
from controller import Robot, DistanceSensor, Motor


class BeamModel:
    """Probabilistic beam model for sensor measurements."""

    def __init__(
        self,
        z_hit=0.7,
        z_short=0.1,
        z_max=0.1,
        z_rand=0.1,
        sigma_hit=0.02,
        lambda_short=2.0,
    ):
        self.z_hit = z_hit
        self.z_short = z_short
        self.z_max = z_max
        self.z_rand = z_rand
        self.sigma_hit = sigma_hit
        self.lambda_short = lambda_short

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
        if z < 0 or z > z_max:
            return 0.0
        coeff = 1.0 / (self.sigma_hit * math.sqrt(2.0 * math.pi))
        exponent = -0.5 * ((z - z_expected) / self.sigma_hit) ** 2
        return coeff * math.exp(exponent)

    def p_short(self, z, z_expected, z_max):
        if z < 0 or z >= z_expected:
            return 0.0
        norm = 1.0 - math.exp(-self.lambda_short * z_expected)
        return (self.lambda_short * math.exp(-self.lambda_short * z)) / norm

    def p_max(self, z, z_expected, z_max):
        tolerance = 0.001
        if abs(z - z_max) < tolerance:
            return 1.0
        return 0.0

    def p_rand(self, z, z_expected, z_max):
        if z < 0 or z > z_max:
            return 0.0
        return 1.0 / z_max

    def beam_model_prob(self, z, z_expected, z_max):
        hit_prob = self.p_hit(z, z_expected, z_max)
        short_prob = self.p_short(z, z_expected, z_max)
        max_prob = self.p_max(z, z_expected, z_max)
        rand_prob = self.p_rand(z, z_expected, z_max)
        total_prob = (
            self.z_hit * hit_prob
            + self.z_short * short_prob
            + self.z_max * max_prob
            + self.z_rand * rand_prob
        )
        return total_prob


class StudentBeamController:
    """Robot controller using beam model for navigation."""

    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        self.proximity_sensors = []
        sensor_names = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]
        for name in sensor_names:
            sensor = self.robot.getDevice(name)
            sensor.enable(self.timestep)
            self.proximity_sensors.append(sensor)

        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        self.beam_model = BeamModel()
        self.max_speed = 6.28
        self.sensor_max_range = 0.12

    def sensor_value_to_distance(self, sensor_value):
        if sensor_value == 0:
            return self.sensor_max_range
        distance = (1.0 - (sensor_value / 4095.0)) * self.sensor_max_range
        return max(0.0, min(distance, self.sensor_max_range))

    def get_expected_distance(self, sensor_index):
        if sensor_index in [0, 7]:
            return 0.08
        elif sensor_index in [1, 6]:
            return 0.06
        else:
            return 0.05

    def analyze_sensors(self):
        print("\n=== Sensor Analysis ===")
        for i, sensor in enumerate(self.proximity_sensors):
            raw_value = sensor.getValue()
            measured_distance = self.sensor_value_to_distance(raw_value)
            expected_distance = self.get_expected_distance(i)

            beam_prob = self.beam_model.beam_model_prob(
                measured_distance, expected_distance, self.sensor_max_range
            )

            p_hit = self.beam_model.p_hit(measured_distance, expected_distance, self.sensor_max_range)
            p_short = self.beam_model.p_short(measured_distance, expected_distance, self.sensor_max_range)
            p_max = self.beam_model.p_max(measured_distance, expected_distance, self.sensor_max_range)
            p_rand = self.beam_model.p_rand(measured_distance, expected_distance, self.sensor_max_range)

            print(
                f"Sensor {i}: Raw={raw_value:4.0f}, Dist={measured_distance:.3f}m, "
                f"Expected={expected_distance:.3f}m, BeamProb={beam_prob:.3f}"
            )
            print(f"  Components: hit={p_hit:.3f}, short={p_short:.3f}, max={p_max:.3f}, rand={p_rand:.3f}")

    def simple_navigation(self):
        front_left = self.sensor_value_to_distance(self.proximity_sensors[0].getValue())
        front_right = self.sensor_value_to_distance(self.proximity_sensors[7].getValue())
        obstacle_threshold = 0.05

        if front_left < obstacle_threshold or front_right < obstacle_threshold:
            if front_left < front_right:
                self.left_motor.setVelocity(0.5 * self.max_speed)
                self.right_motor.setVelocity(-0.5 * self.max_speed)
            else:
                self.left_motor.setVelocity(-0.5 * self.max_speed)
                self.right_motor.setVelocity(0.5 * self.max_speed)
        else:
            self.left_motor.setVelocity(0.5 * self.max_speed)
            self.right_motor.setVelocity(0.5 * self.max_speed)

    def navigate_with_beam_model(self):
        # Example beam-model-based front sensor navigation
        front_sensors = [0, 7]
        probs = []
        distances = []

        for idx in front_sensors:
            raw = self.proximity_sensors[idx].getValue()
            dist = self.sensor_value_to_distance(raw)
            expected = self.get_expected_distance(idx)
            prob = self.beam_model.beam_model_prob(dist, expected, self.sensor_max_range)
            probs.append(prob)
            distances.append(dist)

        # Decide turn direction based on probabilities and distances
        prob_threshold = 0.2  # low probability means sensor reading is unreliable
        if any(p < prob_threshold for p in probs):
            # Treat unreliable readings as obstacles
            if distances[0] < distances[1]:
                self.left_motor.setVelocity(0.5 * self.max_speed)
                self.right_motor.setVelocity(-0.5 * self.max_speed)
                decision = "Turn right (front left unreliable)"
            else:
                self.left_motor.setVelocity(-0.5 * self.max_speed)
                self.right_motor.setVelocity(0.5 * self.max_speed)
                decision = "Turn left (front right unreliable)"
        else:
            # Move forward
            self.left_motor.setVelocity(0.5 * self.max_speed)
            self.right_motor.setVelocity(0.5 * self.max_speed)
            decision = "Move forward (sensors reliable)"

        print(f"Navigation Decision: {decision}, Probabilities: {probs}, Distances: {distances}")

    def run(self):
        step_count = 0
        print("Starting Student Beam Controller...")
        while self.robot.step(self.timestep) != -1:
            step_count += 1
            if step_count % 10 == 0:
                self.analyze_sensors()
            self.navigate_with_beam_model()


if __name__ == "__main__":
    controller = StudentBeamController()
    controller.run()
