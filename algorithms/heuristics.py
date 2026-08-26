import math
from typing import Tuple
from algorithms import utils
from algorithms.problems import SystemRepairProblem


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def manhattanHeuristic(state, problem):
    """
    The Manhattan distance heuristic.

    Baseline rule for this workshop: estimate the direct distance to the next
    mandatory target:
    - K if the robot does not have the kit yet.
    - the nearest pending T if the robot has the kit and systems remain.
    - C if all systems have been repaired.
    """
    x0, y0 = state[0]
    if not state[1]:
        x1, y1 = problem.kitPosition
        return abs(x0 - x1) + abs(y0 - y1)
    else:
        if not state[2]:
            x1, y1 = problem.controlPosition
            return abs(x0 - x1) + abs(y0 - y1)
        mini = math.inf
        for s in state[2]:
            x1, y1 = s
            distance = abs(x0 - x1) + abs(y0 - y1)
            if distance < mini:
                mini = distance
        return mini


def euclideanHeuristic(state, problem):
    """
    The Euclidean distance heuristic.

    Baseline rule for this workshop: estimate the direct distance to the next
    mandatory target:
    - K if the robot does not have the kit yet.
    - the nearest pending T if the robot has the kit and systems remain.
    - C if all systems have been repaired.
    """
    x0, y0 = state[0]
    if not state[1]:
        x1, y1 = problem.kitPosition
        return math.sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2)
    else:
        if not state[2]:
            x1, y1 = problem.controlPosition
            return math.sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2)
        mini = math.inf
        for s in state[2]:
            x1, y1 = s
            distance = math.sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2)
            if distance < mini:
                mini = distance
        return mini
   
   
   
   
def systemRepairHeuristic(
    state: Tuple[Tuple, bool, Tuple], problem: SystemRepairProblem
):
    """
    Your heuristic for the SystemRepairProblem.

    state: (position, hasKit, pendingSystems)
    problem: SystemRepairProblem instance

    This must be admissible and preferably consistent.

    Hints:
    - Use problem.heuristicInfo to cache expensive computations
    - Go with some simple heuristics first, then build up to more complex ones
    - Consider the kit, pending systems, and the final return to control center
    - Balance heuristic strength vs. computation time (do experiments!)
    """
    # TODO: Add your code here
    utils.raiseNotDefined()
