from algorithms.problems import SearchProblem
import algorithms.utils as utils
from world.game import Directions
from algorithms.heuristics import nullHeuristic


def tinyDiagnosticSearch(problem: SearchProblem):
    """
    Returns a hard-coded sequence of moves for the tinyDiagnostic layout.
    For any other station layout, the sequence of moves will be incorrect.
    """
    s = Directions.SOUTH
    e = Directions.EAST
    return [s, e, s, e, e, e, e, s, e, e, s, s, e, s, s, e, s, e, e, e, e, e, e, e]


def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    # TODO: Add your code here
    utils.raiseNotDefined()


def breadthFirstSearch(problem: SearchProblem):
    """
    Search the shallowest nodes in the search tree first.
    """
    # TODO: Add your code here
    utils.raiseNotDefined()


def uniformCostSearch(problem: SearchProblem):
    """
    Search the node of least total cost first.
    """

    # TODO: Add your code here
    utils.raiseNotDefined()


def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """
    Search the node that has the lowest combined cost and heuristic first.

        Pick up the repair kit K, repair all damaged systems T, and finish at C.
    
        State: (position, hasKit, pendingSystems)
        - position: (x, y) tuple
        - hasKit: True if the robot already picked up the repair kit
        - pendingSystems: tuple with the T positions that still need repair
    
        Goal: all systems repaired and robot at C.
        Movement cost is uniform in this problem.
        
        get succ
        
         Returns successor states, the actions they require, and a unit cost.
            
        """
        
    start = problem.getStartState()
    pq = utils.PriorityQueue()
    pq.push((start, []), heuristic(start, problem))
    visited = set()

    while not pq.isEmpty():
        state, actions = pq.pop()
        if state in visited:     
            continue
        visited.add(state)

        if problem.isGoalState(state):
            print(actions, problem.getCostOfActions(actions))
            return actions

        for s, a, c in problem.getSuccessors(state):
            if s not in visited:
                
                pq.push((s, actions + [a]), len(actions) + 1 + heuristic(s, problem))
    return []



# Abbreviations (you can use them for the -f option in main.py)
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
