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

class Node:
    def __init__(self, state, action, cost, parent=None):
        self._action = action
        self._cost = cost
        self._parent = parent
        self._state = state

    @property
    def action(self):
        return self._action

    @property
    def cost(self):
        return self._cost

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
                                and (self._action, self._state, self._cost) ==
                                    (other._action, other._state, other._cost))

    def __hash__(self):
        return hash((self._action, self._state, self._cost))

class SearchAlgorithm:

    def depthFirstSearch(self, problem):
        """
        Search problem instance

        Returns
            List
        """
        frontier_queue = util.PriorityQueue()
        frontier_dictionary = {}
        explored_dictionary = {}

        initial_state = problem.getStartState()
        initial_node = Node(initial_state, None, 0)

        frontier_queue.push(initial_node, initial_node.cost)
        frontier_dictionary[initial_node] = initial_node
        while True:
            if frontier_queue.isEmpty():
                return []
            #get the next frontier
            #remove from frontier
            chosen_frontier_node = frontier_queue.pop()
            frontier_dictionary.pop(chosen_frontier_node, 0)
            #goal check
            if problem.isGoalState(chosen_frontier_node.state):
                return chosen_frontier_node.actions()
            #add node to explored
            explored_dictionary[chosen_frontier_node] = chosen_frontier_node
            #expand and add those to frontier if they are not part of frontier or explored
            for expanded_triple in problem.getSuccessors(chosen_frontier_node.state):
                accumulated_cost = chosen_frontier_node.cost + expanded_triple[2]
                expanded_node = Node(expanded_triple[0], expanded_triple[1], accumulated_cost, chosen_frontier_node)

                if expanded_node not in explored_dictionary and expanded_node not in frontier_dictionary:
                    frontier_queue.push(expanded_node, expanded_node.cost)
                    frontier_dictionary[expanded_node] = expanded_node

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
  util.raiseNotDefined()
      
def uniformCostSearch(problem):
  "Search the node of least total cost first. "
  "*** YOUR CODE HERE ***"
  util.raiseNotDefined()

def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  "*** YOUR CODE HERE ***"
  util.raiseNotDefined()
    
  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
