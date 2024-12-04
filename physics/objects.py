import math


class Obstacle:
    def __init__(
        self,
        a: [float, float],
        b: [float, float],
        width: float = 8
     ):
        self.set_pos(a, b)
        self.width = width

    def set_pos(self, a: [float, float], b: [float, float]):
        self.a = a
        self.b = b

        slope_angle = math.atan2(b[1] - a[1],  b[0] - a[0])

        # Calculate slope angle and its sin and cos for future use
        self.slope_angle = slope_angle
        self.math_cache = [
            math.cos(-self.slope_angle),
            math.sin(-self.slope_angle),
        ]

        # A and B that rotated by -slope_angle
        ra = self.apply_rotation(*a)
        rb = self.apply_rotation(*b)

        self.rotated_points = [ra, rb]

        self.rotated_minx = min(ra[0], rb[0])
        self.rotated_maxx = max(ra[0], rb[0])

    # Rotates points by self.slope_angle radians
    def apply_rotation(self, x: float, y: float) -> [float, float]:
        return [
            x * self.math_cache[0] - y * self.math_cache[1],
            x * self.math_cache[1] + y * self.math_cache[0],
        ]


class Object:
    def __init__(
        self,
        x: float,
        y: float,
        r: float = 8,
        velocity: float = 16,
        acceleration: float = 8,
        pathfinding_velocity: float = 16,
        target_radius: float = None,
    ):
        self.r = r
        if target_radius is not None:
            self.target_radius = target_radius
        else:
            self.target_radius = r
        self.set_pos(x, y)
        self.velocity = velocity
        self.acceleration = acceleration
        self.pathfinding_velocity = pathfinding_velocity

        self.v = None
        self.target = None
        self.reached = False
        self.path_found = False
        self.path = []
        self.path_length = 0

    def set_pos(self, x: float, y: float):
        self.x = x
        self.y = y

    def set_target(self, x: float, y: float):
        self.v = [0, 0]
        self.target = [x, y]
        self.reached = False
        self.path_found = False
        self.path = [(self.x, self.y)]
        self.path_length = 0

    # Clears target.
    def unset_target(self):
        self.target = None
        self.v = None

    def append_path(self):
        distance_to_previous_point = math.sqrt(
            (self.x - self.path[-1][0]) ** 2 +
            (self.y - self.path[-1][1]) ** 2
        )
        self.path_length += distance_to_previous_point
        self.path.append((
            self.x,
            self.y,
            distance_to_previous_point
        ))

    # Calculates the distance to the target.
    # returns distance
    def distance_to_target(self):
        return math.sqrt(
                (self.x - self.target[0]) ** 2 + (self.y - self.target[1]) ** 2
            )

    # Calculates the distance and angle to a object.
    # returns [distance, angle]
    def distance_to_object(self, other):
        return [
            math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2),
            math.atan2(self.y - other.y, self.x - other.x)
        ]

    # Calculates the distance and angle to an line.
    # This code is highly optimized; therefore, it might be difficult to read.
    # returns [distance, angle]
    def distance_to_obstacle(self, obs: Obstacle):
        rotated_point = obs.apply_rotation(self.x, self.y)

        ax = obs.rotated_points[0][0]
        bx = obs.rotated_points[1][0]

        if obs.rotated_minx <= rotated_point[0] <= obs.rotated_maxx:
            # Point's distance to the obstacle (line), which is equal to y
            # difference in the rotated plane. The y-coordinate of both ends
            # of the obstacle should be approximately same.
            dy = rotated_point[1] - obs.rotated_points[0][1]

            res = [
                abs(dy),
                obs.slope_angle + math.copysign(math.pi / 2, dy)
            ]
        else:
            # whether or not A is closer than B
            is_a = abs(rotated_point[0] - ax) < abs(rotated_point[0] - bx)

            dx = self.x - (obs.a[0] if is_a else obs.b[0])
            dy = self.y - (obs.a[1] if is_a else obs.b[1])

            res = [
                math.sqrt(dx ** 2 + dy ** 2),
                math.atan2(dy, dx)
             ]

        res[0] -= obs.width + self.r
        if res[0] < 0:
            res[0] = 0

        return res
