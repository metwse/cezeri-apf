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

    obj = ui.engine.new_object(32, 480, target_velocity=64)
    obj.set_target(480, 32)

    ui.root.bind("<Button-1>", lambda event: obj.set_pos(event.x, event.y)) 

    engine.run_multithreaded()
    ui.mainloop()
