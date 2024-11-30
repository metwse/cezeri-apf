from .objects import Obstacle, Object

import math


class Engine:
    def __init__(
            self,
            # size of the arena
            size: [int, int],
            # frequency of the physics simulation
            frequency: float = 500,
            # maximum number of iterations
            max_iter: int = 50000,
            # saves location to path once per resolution iterations
            resolution: int = 10,
            repulsive_gain: float = 64,
            obstacle_avoidence_radius: float = 32,
            object_avoidence_radius: float = 32,
    ):
        self.objects = []
        self.obstacles = []

        self.frequency = frequency
        self.size = size

        self.max_iter = max_iter
        self.resolution = resolution
        self.repulsive_gain = repulsive_gain
        self.obstacle_avoidence_radius = obstacle_avoidence_radius
        self.object_avoidence_radius = object_avoidence_radius

    # Runs the physics engine.
    def find_path(self):
        self.path_found = sum(obj.path_found for obj in self.objects)
        for obj in self.objects:
            if obj.target is None:
                self.path_found += 1
        iterations = 1

        while (
            self.path_found != len(self.objects) and
            iterations < self.max_iter
        ):
            self.update(1 / self.frequency, iterations)
            iterations += 1

    # Applies forces and updates objects.
    def update(self, dt, iterations):
        for obj in self.objects:
            if obj.target is None or obj.path_found:
                continue

            # distance to the target
            dx = obj.target[0] - obj.x
            dy = obj.target[1] - obj.y

            # Resize the distance vector to match the obj.pathfinding_velocity
            # magnitude.
            d = math.sqrt(dx ** 2 + dy ** 2) / obj.pathfinding_velocity
            if d != 0:
                dx /= d
                dy /= d

                # Apply acceleration using the resized vector.
                a = [dx, dy]
                for axis in range(2):
                    a[axis] -= obj.v[axis]

                # Avoid from obstacles
                for obs in self.obstacles:
                    [distance, angle] = obj.distance_to_obstacle(obs)
                    if (self.obstacle_avoidence_radius > distance > 0):
                        force = (
                            obj.pathfinding_velocity * self.repulsive_gain /
                            distance ** 2
                        )
                        a[0] += math.cos(angle) * force
                        a[1] += math.sin(angle) * force

                # Avoid from other objects
                for other_obj in self.objects:
                    if other_obj == obj:
                        continue
                    [distance, angle] = obj.distance_to_object(other_obj)
                    if (self.object_avoidence_radius > distance > 0):
                        force = (
                            obj.pathfinding_velocity * self.repulsive_gain /
                            distance ** 2
                        )
                        a[0] += math.cos(angle) * force
                        a[1] += math.sin(angle) * force

                obj.x += obj.v[0] * dt + a[0] * dt ** 2 / 2
                obj.y += obj.v[1] * dt + a[1] * dt ** 2 / 2

                for axis in range(2):
                    obj.v[axis] += a[axis] * dt

            if obj.distance_to_target() < obj.target_radius:
                obj.path_found = True
                obj.append_path()
                self.path_found += 1
            else:
                if iterations % self.resolution == 0:
                    obj.append_path()

    # Creates a obstacle.
    def new_obstacle(
        self,
        b: [float, float],
        a: [float, float],
        width: float = 8
    ):
        obstacle = Obstacle(a, b, width=width)
        self.obstacles.append(obstacle)
        return obstacle

    # Creates a object.
    def new_object(
        self,
        x: float,
        y: float,
        r: float = 8,
        # the speed and acceleration while heading toward the target
        velocity: float = 64,
        acceleration: float = 64,
        # the speed while looking for target
        pathfinding_velocity: float = 16,
        target_radius: float = None,
    ):
        obj = Object(
            x, y,
            r=r,
            velocity=velocity,
            pathfinding_velocity=pathfinding_velocity,
            target_radius=target_radius,
            acceleration=acceleration
        )
        self.objects.append(obj)
        return obj
