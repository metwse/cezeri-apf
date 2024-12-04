from physics import Engine
from ui import UI


def apf(
    objects: [(float, float)],
    targets: [(float, float)],
    padding="None",
    step_size=5,
    arena_size=[512, 512],
    obstacles: [((float, float), (float, float))] = [],
    physics_frequency=512,
    return_engine=False
):
    engine = Engine(arena_size, frequency=physics_frequency)
    for obj in objects:
        engine.new_object(*obj)
    for obs in obstacles:
        engine.new_obstacle(*obs)

    for (i, target) in enumerate(targets):
        engine.objects[i].set_target(*target)

    engine.find_path()

    return (
        [engine, engine.walk_path(step_size=10)]
        if return_engine else
        engine.walk_path(step_size=10)
    )


if __name__ == "__main__":
    [engine, path] = apf(
        [(32, 32)], [(480, 480)],
        obstacles=[
            [[100, 400], [100, 350]],
            [[100, 400], [300, 400]],
            [[400, 430], [400, 330]],
            [[300, 230], [400, 330]],
            [[250, 130], [400, 90]],
            [[100, 50], [50, 200]]
        ],
        return_engine=True
    )

    ui = UI(engine)
    ui.draw_path(path)

    ui.mainloop()
