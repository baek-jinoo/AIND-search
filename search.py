# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called 
by Pacman agents (in searchAgents.py).
"""

import util
import pdb

def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

class Node:
    def __init__(self, state, action, cost, parent=None, heuristicCost=0.):
        self._action = action
        self._cost = cost
        self._parent = parent
        self._state = state
        self._heuristicCost = heuristicCost

    @property
    def action(self):
        return self._action

    @property
    def cost(self):
        return self._cost

    @property
    def c_and_h(self):
        return self._cost + self._heuristicCost

    @property
    def parent(self):
        return self._parent

    @property
    def state(self):
        return self._state

    def actions(self):
        previous_actions = []
        if self._parent != None:
            previous_actions.extend(self._parent.actions())
        if self._action != None:
            previous_actions.append(self._action)
        return previous_actions

    def __eq__(self, other):
        return (isinstance(other, type(self))
                                and self._state == other._state)

    def __hash__(self):
        return hash(self._state)

class SearchAlgorithm:

    def graphSearchGeneric(self, problem, frontier_collection,
                            heuristicFn=nullHeuristic):
        explored_dict = {}

        initial_state = problem.getStartState()
        initial_node = Node(initial_state, None, 0, None,
                            heuristicFn(initial_state, problem))

        frontier_collection.push(initial_node)
        while True:
            if frontier_collection.isEmpty():
                return []

            c_frontier_node = frontier_collection.pop()

            if problem.isGoalState(c_frontier_node.state):
                return c_frontier_node.actions()
            if c_frontier_node not in explored_dict:
                explored_dict[c_frontier_node] = True
                for expanded_triple in problem.getSuccessors(c_frontier_node.state):
                    accumulated_cost = c_frontier_node.cost + expanded_triple[2]

                    expanded_node = Node(expanded_triple[0], expanded_triple[1],
                                         accumulated_cost, c_frontier_node,
                                         heuristicFn(expanded_triple[0], problem))
                    frontier_collection.push(expanded_node)

    def aStarSearch(self, problem, heuristic):
        #TODO add doc
        priorityQueueFunction = lambda item: item.c_and_h
        frontier_q = util.PriorityQueueWithFunction(priorityQueueFunction)
        return self.graphSearchGeneric(problem, frontier_q, heuristic)

    def uniformCostSearch(self, problem):
        #TODO add doc
        priorityQueueFunction = lambda item: item.cost
        frontier_q = util.PriorityQueueWithFunction(priorityQueueFunction)
        return self.graphSearchGeneric(problem, frontier_q)

    def breadthFirstSearch(self, problem):
        #TODO add doc
        return self.graphSearchGeneric(problem, util.Queue())

    def depthFirstSearch(self, problem):
        #TODO add doc
        return self.graphSearchGeneric(problem, util.Stack())

class SearchProblem:
  """
  This class outlines the structure of a search problem, but doesn't implement
  any of the methods (in object-oriented terminology: an abstract class).
  
  You do not need to change anything in this class, ever.
  """
  
  def getStartState(self):
     """
     Returns the start state for the search problem 
     """
     util.raiseNotDefined()
    
  def isGoalState(self, state):
     """
       state: Search state
    
     Returns True if and only if the state is a valid goal state
     """
     util.raiseNotDefined()

  def getSuccessors(self, state):
     """
       state: Search state
     
     For a given state, this should return a list of triples, 
     (successor, action, stepCost), where 'successor' is a 
     successor to the current state, 'action' is the action
     required to get there, and 'stepCost' is the incremental 
     cost of expanding to that successor
     """
     util.raiseNotDefined()

  def getCostOfActions(self, actions):
     """
      actions: A list of actions to take
 
     This method returns the total cost of a particular sequence of actions.  The sequence must
     be composed of legal moves
     """
     util.raiseNotDefined()
           

def tinyMazeSearch(problem):
  """
  Returns a sequence of moves that solves tinyMaze.  For any other
  maze, the sequence of moves will be incorrect, so only use this for tinyMaze
  """
  from game import Directions
  s = Directions.SOUTH
  w = Directions.WEST
  return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
  """
  Search the deepest nodes in the search tree first
  [2nd Edition: p 75, 3rd Edition: p 87]
  
  Your search algorithm needs to return a list of actions that reaches
  the goal.  Make sure to implement a graph search algorithm 
  [2nd Edition: Fig. 3.18, 3rd Edition: Fig 3.7].
  
  To get started, you might want to try some of these simple commands to
  understand the search problem that is being passed in:
  
  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())
  """
  "*** YOUR CODE HERE ***"
  searchAlgorithm = SearchAlgorithm()
  return searchAlgorithm.depthFirstSearch(problem)

def breadthFirstSearch(problem):
  """
  Search the shallowest nodes in the search tree first.
  [2nd Edition: p 73, 3rd Edition: p 82]
  """
  "*** YOUR CODE HERE ***"
  searchAlgorithm = SearchAlgorithm()
  return searchAlgorithm.breadthFirstSearch(problem) 

      
def uniformCostSearch(problem):
  "Search the node of least total cost first. "
  "*** YOUR CODE HERE ***"
  searchAlgorithm = SearchAlgorithm()
  return searchAlgorithm.uniformCostSearch(problem) 

def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  searchAlgorithm = SearchAlgorithm()
  return searchAlgorithm.aStarSearch(problem, heuristic) 
    
  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
