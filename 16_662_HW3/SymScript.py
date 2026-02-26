import SymPlanner as sp

# DO NOT MODIFY - start
# -------------------------------------------------------------------------------
def make_Room_domain():
    # Move from a room to the hallway
    go_to_hall = sp.ActionSchema(
        name="go-to-hall",
        parameters=("?r",),
        preconds=(("in", ("robot", "?r")),),
        add_effects=(("in", ("robot", "hall")),),
        del_effects=(("in", ("robot", "?r")),),
    )

    # Move from the hallway to a room
    go_to_room = sp.ActionSchema(
        name="go-to-room",
        parameters=("?r",),
        preconds=(
            ("in", ("robot", "hall")),
            ("room", ("?r",)),
        ),
        add_effects=(("in", ("robot", "?r")),),
        del_effects=(("in", ("robot", "hall")),),
    )

    # Pick up an object in the current room (unlimited carry)
    pick = sp.ActionSchema(
        name="pick",
        parameters=("?o", "?l"),
        preconds=(
            ("in", ("robot", "?l")),
            ("in", ("?o", "?l")),
        ),
        add_effects=(("carrying", ("?o",)),),
        del_effects=(("in", ("?o", "?l")),),
    )

    # Place an object the robot is carrying into the current room
    place = sp.ActionSchema(
        name="place",
        parameters=("?o", "?l"),
        preconds=(
            ("in", ("robot", "?l")),
            ("carrying", ("?o",)),
            ("room", ("?l",)),  # disallow placing in the hall
        ),
        add_effects=(("in", ("?o", "?l")),),
        del_effects=(("carrying", ("?o",)),),
    )

    return [go_to_hall, go_to_room, pick, place]

# DO NOT MODIFY - end
# -------------------------------------------------------------------------------

def make_Block_domain():
    # TODO 1.1: Define action schemas for different actions that you think would be required to solve problems in the Block World Domain.
    # Hint: Check the problem in make_Block_problem() to get an idea of the actions required to solve it.
    return NotImplementedError


def make_Room_problem():

    init = {
        # Static facts
        ("room", ("kitchen",)),
        ("room", ("garden",)),
        ("room", ("office",)),
        ("room", ("living",)),
        # Robot start
        ("in", ("robot", "kitchen")),
        # Objects start
        ("in", ("knife", "office")),
        ("in", ("paper", "office")),
        ("in", ("lemon", "living")),
        ("in", ("strawberry", "garden")),
    }

    goal = {
        ("in", ("lemon", "garden")),
        ("in", ("knife", "kitchen")),
        ("in", ("strawberry", "kitchen")),
        ("in", ("robot", "kitchen")),
        # (paper can be anywhere)
    }
    return init, goal


def make_Block_problem():
    init = {
        ("on", ("a", "b")),
        ("on", ("b", "table")),
        ("on", ("c", "table")),
        ("clear", ("a",)),
        ("clear", ("c",)),
    }
    goal = {
        ("on", ("a", "b")),
        ("on", ("b", "c")),
    }
    return init, goal


def make_Block2_problem():
    # TODO 1.4: Write out the initial and goal states for the problem illustrated in the handout.
    init = {}
    goal = {}
    return init, goal


if __name__ == "__main__":

    # This block solves the Room problem. Run it after you have implemented discrete search and then feel free to comment it out.
    planner = sp.Planner(make_Room_domain())
    init, goal = make_Room_problem()
    plan = planner.plan(init, goal)  # <-- TODO 1.2: Implement in SymPlanner.py
    if plan is None:
        print("No plan found.")
    else:
        print("Plan:")
        for step, act in enumerate(plan, 1):
            print(f"{step:02d}. {act.label()}")

    # TODO 1.3: Solve the Block problem using make_Block_domain() and sp.Planner().

    # TODO 1.4: Solve the Block2 problem.
