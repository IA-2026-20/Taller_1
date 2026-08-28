from algorithms.problems import SearchProblem
import algorithms.utils as utils
from world.game import Directions
from algorithms.utils import Stack, Queue
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
    frontera = Stack()
    frontera.push((problem.getStartState(), []))
    visitados = set()

    while not frontera.isEmpty():
        estado, acciones = frontera.pop()

        if problem.isGoalState(estado):
            return acciones

        if estado not in visitados:
            visitados.add(estado)
            for siguiente, accion, z in problem.getSuccessors(estado):
                if siguiente not in visitados:
                    frontera.push((siguiente, acciones + [accion]))

    return []


def breadthFirstSearch(problem: SearchProblem):
    """
    Search the shallowest nodes in the search tree first.
    """
    frontera = Queue()
    estado_ini = problem.getStartState()
    frontera.push((estado_ini, []))
    visitados = set([estado_ini])

    while not frontera.isEmpty():
        estado, acciones = frontera.pop()

        if problem.isGoalState(estado):
            return acciones

        for siguiente, accion, z in problem.getSuccessors(estado):
            if siguiente not in visitados:
                visitados.add(siguiente)
                frontera.push((siguiente, acciones + [accion]))

    return []


def uniformCostSearch(problem: SearchProblem):
    """
    Search the node of least total cost first.
    """
    frontera = utils.PriorityQueue()
    estadoini = problem.getStartState()
    frontera.push((estadoini, [], 0), 0)

    visited = {}

    while not frontera.isEmpty():
        estado, acciones, costo = frontera.pop()

        if problem.isGoalState(estado):
            return acciones

        if estado in visited and visited[estado] <= costo:
            continue
        visited[estado] = costo

        for successor, action, stepCost in problem.getSuccessors(estado):
            newCost = costo + stepCost
            if successor not in visited or newCost < visited[successor]:
                frontera.push((successor, acciones + [action], newCost), newCost)

    return []


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
