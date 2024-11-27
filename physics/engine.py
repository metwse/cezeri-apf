from .objects import Obstacle, Object

import time

import math

import threading


class Engine:
    def __init__(self, size: [int, int], frequency: float):
        self.objects = []
        self.obstacles = []

        self.frequency = frequency
        self.size = size

    # Runs the physics engine.
    def run(self):
        self.t = time.time()
        while True:
            time.sleep(1 / self.frequency)
            self.update()

    def run_multithreaded(self):
        threading.Thread(target=self.run).start()

    # Applies forces and updates objects.
    def update(self):
        dt = time.time() - self.t
        self.t = time.time()

        for obj in self.objects:
            if obj.target is None:
                continue

            # distance to the target
            dx = obj.target[0] - obj.x
            dy = obj.target[1] - obj.y

            # Resize the distance vector to match the obj.target_velocity
            # magnitude.
            d = math.sqrt(dx ** 2 + dy ** 2) / obj.target_velocity
            dx /= d
            dy /= d

            # Apply acceleration using the resized vector.
            a = [dx, dy]
            for axis in range(2):
                a[axis] -= obj.v[axis]

            # Avoid obstacles
            for obs in self.obstacles:
                [distance, angle] = obj.distance_to_obstacle(obs)
                if (32 > distance > 0):
                    a[0] += (
                        math.cos(angle) * obj.target_velocity * 64 /
                        distance ** 2
                    )
                    a[1] += (
                        math.sin(angle) * obj.target_velocity * 64 /
                        distance ** 2
                    )

            obj.x += obj.v[0] * dt + a[0] * dt ** 2 / 2
            obj.y += obj.v[1] * dt + a[1] * dt ** 2 / 2

            for axis in range(2):
                obj.v[axis] += a[axis] * dt

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
        target_velocity: float = 16
    ):
        obj = Object(x, y, r=r, target_velocity=target_velocity)
        self.objects.append(obj)
        return obj
