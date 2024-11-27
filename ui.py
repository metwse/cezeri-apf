from physics import Engine

import tkinter as tk

import math


class UI:
    def __init__(self, engine: Engine):
        self.engine = engine

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
            self.root.after(13, lambda: self.draw(recursive=True))

        self.canvas.delete("all")

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
                if distance > 96:
                    continue

                # Draw the forces from the obstacles.
                self.canvas.create_line(
                    obj.x, obj.y,
                    obj.x + math.cos(angle) * obj.r,
                    obj.y + math.sin(angle) * obj.r,
                    fill="white"
                )

        self.root.update()
