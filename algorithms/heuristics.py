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

def manhattanDistance(a, b):
    """
    Manhattan distance between two (x, y) positions.
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def manhattanHeuristic(state, problem):
    """
    The Manhattan distance heuristic.

    Baseline rule for this workshop: estimate the direct distance to the next
    mandatory target:
    - K if the robot does not have the kit yet.
    - the nearest pending T if the robot has the kit and systems remain.
    - C if all systems have been repaired.
    """
    if not state[1]:
        return manhattanDistance(state[0], problem.kitPosition)
    else:
        if not state[2]:
            return manhattanDistance(state[0], problem.controlPosition)
        mini = math.inf
        for s in state[2]:
            distance = manhattanDistance(state[0], s)
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
    if not state[2]:
        return manhattanDistance(state[0], problem.controlPosition)
    if state[1]:
        return max( manhattanDistance(state[0], t) 
        + manhattanDistance(t, problem.controlPosition) for t in state[2])

    return manhattanDistance(state[0], problem.kitPosition) + max( manhattanDistance(problem.kitPosition, t) 
    + manhattanDistance(t, problem.controlPosition) for t in state[2])
