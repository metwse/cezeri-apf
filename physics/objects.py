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
        target_velocity: float = 16
    ):
        self.v = None
        self.target = None
        self.r = r
        self.set_pos(x, y)
        self.target_velocity = target_velocity

    def set_pos(self, x: float, y: float):
        self.x = x
        self.y = y

    def set_target(self, x: float, y: float):
        self.target = [x, y]
        self.v = [0, 0]

    # Clears target.
    def unset_target(self):
        self.target = None
        self.v = None

    # Calculates angle and distance to line.
    # This code is highly optimized, so it might be difficult to read.
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
