# multiAgents.py
# --------------
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


from util import manhattanDistance
from game import Directions
import random, util
import math

from game import Agent
from pacman import GameState

class ReflexAgent(Agent):
    """
    A reflex agent chooses an action at each choice point by examining
    its alternatives via a state evaluation function.

    The code below is provided as a guide.  You are welcome to change
    it in any way you see fit, so long as you don't touch our method
    headers.
    """


    def getAction(self, gameState: GameState):
        """
        You do not need to change this method, but you're welcome to.

        getAction chooses among the best options according to the evaluation function.

        Just like in the previous project, getAction takes a GameState and returns
        some Directions.X for some X in the set {NORTH, SOUTH, WEST, EAST, STOP}
        """
        # Collect legal moves and successor states
        legalMoves = gameState.getLegalActions()

        # Choose one of the best actions
        scores = [self.evaluationFunction(gameState, action) for action in legalMoves]
        bestScore = max(scores)
        bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
        chosenIndex = random.choice(bestIndices) # Pick randomly among the best

        "Add more of your code here if you want to"

        return legalMoves[chosenIndex]

    def evaluationFunction(self, currentGameState: GameState, action):
        """
        Design a better evaluation function here.

        The evaluation function takes in the current and proposed successor
        GameStates (pacman.py) and returns a number, where higher numbers are better.

        The code below extracts some useful information from the state, like the
        remaining food (newFood) and Pacman position after moving (newPos).
        newScaredTimes holds the number of moves that each ghost will remain
        scared because of Pacman having eaten a power pellet.

        Print out these variables to see what you're getting, then combine them
        to create a masterful evaluation function.
        """
        # Useful information you can extract from a GameState (pacman.py)
        successorGameState = currentGameState.generatePacmanSuccessor(action)
        newPos = successorGameState.getPacmanPosition()
        newFood = successorGameState.getFood()
        newGhostStates = successorGameState.getGhostStates()
        newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]

        "*** YOUR CODE HERE ***"

        print(successorGameState)
        print(newPos)
        print(newFood)
        print(newGhostStates[0])
        print(newScaredTimes)
        score = 0

        foodList = newFood.asList()
        foodDist = [manhattanDistance(newPos, foodPos) for foodPos in foodList]
        if len(foodDist) == 0:
            return 0
        
        closestFood = min(foodDist)
        print(closestFood)
        
        for ghost in successorGameState.getGhostPositions():
            if (manhattanDistance(newPos, ghost) < 2):
                return -math.inf
        # exit()
        return successorGameState.getScore() + 1/closestFood

def scoreEvaluationFunction(currentGameState: GameState):
    """
    This default evaluation function just returns the score of the state.
    The score is the same one displayed in the Pacman GUI.

    This evaluation function is meant for use with adversarial search agents
    (not reflex agents).
    """
    return currentGameState.getScore()

class MultiAgentSearchAgent(Agent):
    """
    This class provides some common elements to all of your
    multi-agent searchers.  Any methods defined here will be available
    to the MinimaxPacmanAgent, AlphaBetaPacmanAgent & ExpectimaxPacmanAgent.

    You *do not* need to make any changes here, but you can if you want to
    add functionality to all your adversarial search agents.  Please do not
    remove anything, however.

    Note: this is an abstract class: one that should not be instantiated.  It's
    only partially specified, and designed to be extended.  Agent (game.py)
    is another abstract class.
    """

    def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
        self.index = 0 # Pacman is always agent index 0
        self.evaluationFunction = util.lookup(evalFn, globals())
        self.depth = int(depth)

class MinimaxAgent(MultiAgentSearchAgent):
    """
    Your minimax agent (question 2)
    """

    def getAction(self, gameState: GameState):
        """
        Returns the minimax action from the current gameState using self.depth
        and self.evaluationFunction.

        Here are some method calls that might be useful when implementing minimax.

        gameState.getLegalActions(agentIndex):
        Returns a list of legal actions for an agent
        agentIndex=0 means Pacman, ghosts are >= 1

        gameState.generateSuccessor(agentIndex, action):
        Returns the successor game state after an agent takes an action

        gameState.getNumAgents():
        Returns the total number of agents in the game

        gameState.isWin():
        Returns whether or not the game state is a winning state

        gameState.isLose():
        Returns whether or not the game state is a losing state
        """
        "*** YOUR CODE HERE ***"

        return self.minimax(gameState, 0, 0)[1]
        util.raiseNotDefined()

    def terminalState(self, gameState: GameState, agent, depth):
        if depth == self.depth or gameState.isWin() \
        or gameState.isLose() or len(gameState.getLegalActions(agent)) == 0:
            return True
        return False

    def minimax(self, gameState: GameState, agent, depth):
        if self.terminalState(gameState, agent, depth):
            return self.evaluationFunction(gameState), None
        
        if agent == 0:
            return self.maxValue(gameState, agent, depth)
        else:
            return self.minValue(gameState, agent, depth)

    def maxValue(self, gameState: GameState, agent, depth):
        v = (-math.inf, None)

        for action in gameState.getLegalActions(agent):
            successor = gameState.generateSuccessor(agent, action)
            successor_agent = agent+1
            successor_depth = depth

            v = max(v, (self.minimax(successor, successor_agent, successor_depth)[0], action))

        return v    

    def minValue(self, gameState: GameState, agent, depth):    
        v = (math.inf, None)

        for action in gameState.getLegalActions(agent):
            successor = gameState.generateSuccessor(agent, action)
            successor_agent = agent+1
            successor_depth = depth

            if successor_agent == gameState.getNumAgents():
                successor_agent = 0
                successor_depth += 1
            
            v = min(v, (self.minimax(successor, successor_agent, successor_depth)[0], action))

        return v

class AlphaBetaAgent(MultiAgentSearchAgent):
    """
    Your minimax agent with alpha-beta pruning (question 3)
    """

    def getAction(self, gameState: GameState):
        """
        Returns the minimax action using self.depth and self.evaluationFunction
        """
        "*** YOUR CODE HERE ***"
        return self.minimax(gameState, 0, 0, -math.inf, math.inf)[1]
        util.raiseNotDefined()

    def terminalState(self, gameState: GameState, agent, depth):
        if depth == self.depth or gameState.isWin() \
        or gameState.isLose() or len(gameState.getLegalActions(agent)) == 0:
            return True
        return False

    def minimax(self, gameState: GameState, agent, depth, alpha, beta):
        if self.terminalState(gameState, agent, depth):
            return self.evaluationFunction(gameState), None
        
        if agent == 0:
            return self.maxValue(gameState, agent, depth, alpha, beta)
        else:
            return self.minValue(gameState, agent, depth, alpha, beta)

    def maxValue(self, gameState: GameState, agent, depth, alpha, beta):
        v = (-math.inf, None)

        for action in gameState.getLegalActions(agent):
            successor = gameState.generateSuccessor(agent, action)
            successor_agent = agent+1
            successor_depth = depth

            v = max(v, (self.minimax(successor, successor_agent, successor_depth, alpha, beta)[0], action))

            if v[0] > beta: return v

            alpha = max(alpha, v[0])

        return v    

    def minValue(self, gameState: GameState, agent, depth, alpha, beta):    
        v = (math.inf, None)

        for action in gameState.getLegalActions(agent):
            successor = gameState.generateSuccessor(agent, action)
            successor_agent = agent+1
            successor_depth = depth

            if successor_agent == gameState.getNumAgents():
                successor_agent = 0
                successor_depth += 1
            
            v = min(v, (self.minimax(successor, successor_agent, successor_depth, alpha, beta)[0], action))

            if v[0] < alpha: return v

            beta = min(beta, v[0])

        return v

class ExpectimaxAgent(MultiAgentSearchAgent):
    """
      Your expectimax agent (question 4)
    """

    def getAction(self, gameState: GameState):
        """
        Returns the expectimax action using self.depth and self.evaluationFunction

        All ghosts should be modeled as choosing uniformly at random from their
        legal moves.
        """
        "*** YOUR CODE HERE ***"
        actions = []
    
        for action in gameState.getLegalActions():
            successor = gameState.generateSuccessor(0, action)
            actions.append((self.expectimax(successor, 1, 0), action))

        return max(actions)[1]
        util.raiseNotDefined()

    def terminalState(self, gameState: GameState, agent, depth):
        if depth == self.depth or gameState.isWin() \
        or gameState.isLose() or len(gameState.getLegalActions(agent)) == 0:
            return True
        return False 

    def expectimax(self, gameState: GameState, agent, depth):
        if self.terminalState(gameState, agent, depth):
            return self.evaluationFunction(gameState)

        if agent == 0:
            return self.maxValue(gameState, agent, depth)
        else:
            return self.chanceValue(gameState, agent, depth)

    def maxValue(self, gameState: GameState, agent, depth):
        v = -math.inf

        for action in gameState.getLegalActions(agent):
            successor = gameState.generateSuccessor(agent, action)
            successor_agent = agent+1
            successor_depth = depth

            v = max(v, self.expectimax(successor, successor_agent, successor_depth))

        return v

    def chanceValue(self, gameState: GameState, agent, depth):
        v = 0
        legalActions = gameState.getLegalActions(agent)
        probability = 1/len(legalActions)

        for action in legalActions:
            successor = gameState.generateSuccessor(agent, action)
            successor_agent = agent+1
            successor_depth = depth

            if successor_agent == gameState.getNumAgents():
                successor_agent = 0
                successor_depth += 1
            
            v += probability*self.expectimax(successor, successor_agent, successor_depth)

        return v

def betterEvaluationFunction(currentGameState: GameState):
    """
    Your extreme ghost-hunting, pellet-nabbing, food-gobbling, unstoppable
    evaluation function (question 5).

    DESCRIPTION: <write something here so we know what you did>
    """
    "*** YOUR CODE HERE ***"

    newPos = currentGameState.getPacmanPosition()
    newFood = currentGameState.getFood().asList()

    minFood = math.inf
    for foodPos in newFood:
        minFood = min(minFood, manhattanDistance(newPos, foodPos))

    ghostDist = 0
    for ghost in currentGameState.getGhostPositions():
        ghostDist = manhattanDistance(newPos, ghost)
        if (ghostDist < 2):
            return -math.inf

    foodLeft = currentGameState.getNumFood()

    capsLeft = len(currentGameState.getCapsules())

    score = currentGameState.getScore()
    if (currentGameState.isLose()):
        return -math.inf

    if (currentGameState.isWin()):
        return math.inf

    return 10/(minFood+1) + ghostDist - 100*foodLeft + -10*capsLeft + 200*score

    util.raiseNotDefined()

# Abbreviation
better = betterEvaluationFunction
