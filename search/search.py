# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

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
    "*** YOUR CODE HERE ***"
    actions = []
    fringe = util.Stack()
    visited = {}
    parent = {}
    fringe.push(problem.getStartState())

    while True:
        if fringe.isEmpty():
            return Exception("Search failed")
        
        state = fringe.pop()
        if state not in visited:
            visited[state] = True
            if problem.isGoalState(state):
                while state != problem.getStartState():
                    state, move = parent[state]
                    actions.append(move)
                actions.reverse()
                return actions
            
            successors = problem.getSuccessors(state)
            for successor in successors:
                if successor[0] not in visited:
                    parent[successor[0]] = (state, successor[1])
                    fringe.push(successor[0])

    util.raiseNotDefined()

def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    actions = []
    fringe = util.Queue()
    fringe.push(problem.getStartState())
    visited = {}
    neighbours = {problem.getStartState():True}
    parent = {}


    while True:
        if fringe.isEmpty():
            return Exception("Search failed again")

        state = fringe.pop()
        del neighbours[state]
        if state not in visited:
            visited[state] = True
            if problem.isGoalState(state):
                while state != problem.getStartState():
                    state, move, cost = parent[state]
                    actions.append(move)
                actions.reverse()
                return actions

            successors = problem.getSuccessors(state)
            for successor in successors:
                if successor[0] not in visited and successor[0] not in neighbours:
                    neighbours[successor[0]] = True
                    parent[successor[0]] = (state, successor[1], successor[2])
                    fringe.push(successor[0])
                        
    util.raiseNotDefined()

def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    actions = []
    fringe = util.PriorityQueue()
    fringe.push(problem.getStartState(), 0)
    visited = {}
    neighbours = {}
    costs = {problem.getStartState(): 0}
    parent = {}

    while True:
        if fringe.isEmpty():
            return Exception("Search failed")

        state= fringe.pop()
        if state not in visited:
            visited[state] = True
            if problem.isGoalState(state):
                while state != problem.getStartState():
                    state, move = parent[state]
                    actions.append(move)
                actions.reverse()
                return actions
            successors = problem.getSuccessors(state)
            for successor in successors:
                if successor[0] not in visited and successor[0] not in neighbours:
                    neighbours[successor[0]] = True
                    parent[successor[0]] = (state, successor[1])
                    costs[successor[0]] = costs[state] + successor[2]
                    fringe.push(successor[0], costs[successor[0]])
                elif successor[0] in visited or successor[0] in neighbours:
                    if costs[successor[0]] > costs[state] + successor[2]:
                        parent[successor[0]] = (state, successor[1])
                        costs[successor[0]] = costs[state] + successor[2]
                        fringe.update(successor[0], costs[successor[0]])

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    actions = []
    fringe = util.PriorityQueue()
    fringe.push(problem.getStartState(), 0)
    visited = {}
    neighbours = {}
    costs = {problem.getStartState(): 0}
    parent = {}

    while True:
            if fringe.isEmpty():
                return Exception("Search failed")

            state= fringe.pop()
            if state not in visited:
                visited[state] = True 
                if problem.isGoalState(state):
                    while state != problem.getStartState():
                        state, move = parent[state]
                        actions.append(move)
                    actions.reverse()
                    return actions
                successors = problem.getSuccessors(state)
                for successor in successors:
                    if successor[0] not in visited and successor[0] not in neighbours:
                        neighbours[successor[0]] = True
                        parent[successor[0]] = (state, successor[1])
                        costs[successor[0]] = costs[state] + successor[2] 
                        fringe.push(successor[0], costs[successor[0]] + heuristic(successor[0], problem))
                    elif successor[0] in visited or successor[0] in neighbours:
                        if costs[successor[0]] > costs[state] + successor[2] + heuristic(successor[0], problem):
                            parent[successor[0]] = (state, successor[1])
                            costs[successor[0]] = costs[state] + successor[2] + heuristic(successor[0], problem)
                            fringe.update(successor[0], costs[successor[0]])

    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
