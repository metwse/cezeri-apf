from physics import Engine

import tkinter as tk

import math

import threading

import time


def interpolation(a, b, t):
    return (b - a) * t + a


class UI:
    def __init__(
            self,
            engine: Engine,
            fps: float = 75
    ):
        self.engine = engine
        self.fps = fps

        self.root = tk.Tk()
        self.root.title("APF")
        self.root.resizable(0, 0)
        self.root.geometry(f"{engine.size[0]}x{engine.size[1]}")

        self.canvas = tk.Canvas(background="black")
        self.canvas.pack(fill=tk.BOTH, expand=1)

    def mainloop(self):
        self.draw(recursive=True)
        self.root.mainloop()

    def draw(self, recursive=False):
        if recursive:
            self.root.after(
                int(1000 / self.fps), lambda: self.draw(recursive=True)
            )

        self.canvas.delete("all")

        # Draws objstacles
        for obs in self.engine.obstacles:
            self.canvas.create_line(
                *obs.a, *obs.b,
                fill="white", width=obs.width,
                capstyle=tk.ROUND
            )

        for obj in self.engine.objects:
            # Draw the target if exists
            if obj.target is not None:
                self.canvas.create_oval(
                    obj.target[0] - obj.r, obj.target[1] - obj.r,
                    obj.target[0] + obj.r, obj.target[1] + obj.r,
                    fill="red"
                )

            self.canvas.create_oval(
                obj.x - obj.r, obj.y - obj.r, obj.x + obj.r, obj.y + obj.r,
                fill="green"
            )

            for obs in self.engine.obstacles:
                [distance, angle] = obj.distance_to_obstacle(obs)
                if distance > self.engine.obstacle_avoidence_radius:
                    continue

                # Draw the forces from the obstacles.
                self.canvas.create_line(
                    obj.x, obj.y,
                    obj.x + math.cos(angle) * obj.r,
                    obj.y + math.sin(angle) * obj.r,
                    fill="white"
                )

        self.root.update()

    def walk_path_multithreaded(self):
        threading.Thread(target=self.walk_path).start()

    def walk_path(self):
        while True:
            engine = self.engine
            self.reached = sum(obj.reached for obj in engine.objects)

            # Initialize objects for animation.
            for obj in engine.objects:
                if obj.target is None:
                    self.reached += 1
                    continue
                obj._a_last_index = 0
                obj._a_last_interpolation = 0
                obj._a_remaining_distance = obj.path_length
                obj._a_velocity = 0
                obj._a_stopping_distance = 0

            if self.reached == len(engine.objects):
                time.sleep(0.01)
                continue

            t = time.time()
            # Loop until all objects have reached to their targets.
            while self.reached < len(engine.objects):
                dt = time.time() - t
                t = time.time()
                for obj in engine.objects:
                    if obj.reached or obj.target is None:
                        continue

                    # Decelerate object at the end.
                    if obj._a_remaining_distance < obj._a_stopping_distance:
                        obj._a_velocity -= obj.acceleration * dt
                        if obj._a_velocity < 0:
                            obj._a_velocity = float('inf')
                    # Accelerate object at the beginning.
                    elif obj._a_velocity < obj.velocity:
                        obj._a_velocity += obj.acceleration * dt
                        stopping_time = obj._a_velocity / obj.acceleration
                        obj._a_stopping_distance = (
                            obj._a_velocity * stopping_time / 2
                        )

                    last_distance = obj.path[obj._a_last_index + 1][2]
                    total_disatance = (
                        (1 - obj._a_last_interpolation) *
                        last_distance
                    )

                    target_distance = dt * obj._a_velocity
                    while total_disatance <= target_distance:
                        if obj._a_last_index >= len(obj.path) - 2:
                            break
                        obj._a_last_index += 1

                        last_distance = obj.path[obj._a_last_index + 1][2]
                        total_disatance += last_distance
                    else:
                        obj._a_last_interpolation = i = 1 - (
                            (total_disatance - target_distance) /
                            last_distance
                        )
                        obj._a_remaining_distance -= target_distance

                        prev = obj.path[obj._a_last_index]
                        next = obj.path[obj._a_last_index + 1]

                        obj.x = interpolation(prev[0], next[0], i)
                        obj.y = interpolation(prev[1], next[1], i)
                        continue

                    obj.x = obj.path[-1][0]
                    obj.y = obj.path[-1][1]
                    obj.reached = True
                    self.reached += 1

                time.sleep(1 / self.fps)

            if self.on_reached is not None:
                self.on_reached()
