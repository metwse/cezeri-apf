from ui import UI
from physics import Engine


if __name__ == "__main__":
    engine = Engine(size=[512, 512], frequency=1000)
    ui = UI(engine)

    obstacles = [
        [[100, 400], [100, 350]],
        [[100, 400], [300, 400]],
        [[400, 430], [400, 330]],
        [[300, 230], [400, 330]],
        [[250, 130], [400, 90]],
        [[100, 50], [50, 200]]
    ]

    for obs in obstacles:
        engine.new_obstacle(*obs)

    objects = [
        ui.engine.new_object(32, 32, pathfinding_velocity=64, velocity=128),
        ui.engine.new_object(32, 480, pathfinding_velocity=64, velocity=128),
    ]

    targets = [
        [(480, 480), (32, 32)],
        [(480, 32), (232, 280)],
        [(32, 480), (480, 32)]
    ]

    def next_target():
        if len(targets) == 0:
            return
        for (i, target) in enumerate(targets.pop()):
            objects[i].set_target(*target)
            engine.find_path()

    next_target()
    ui.on_reached = next_target

    engine.find_path()
    ui.walk_path_multithreaded()
    ui.mainloop()
