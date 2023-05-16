import heapq as hq
from time import perf_counter
import copy

def getInput():

  n = int(input("Enter the puzzle size: \n").strip())

  initialState = []
  #Initial state input
  print("Let's get input for the initial state! You can represent empty tile with -1.\n")
  for i in range(n):
    row = list(map(int, input(f'Enter n space seperated values for row {i+1}: \n').strip().split()))
    initialState.append(row)

  goalState = []
  goalStateDecision = input("Do you wish to give a custom goal state, press Y - Yes or N - No?\n").strip().lower()

  if goalStateDecision == 'y':

    print("Let's get input for the goal state! You can represent empty tile with -1.\n")
    #Initial state input
    for i in range(n):
      row = list(map(int, input(f'Enter {n} space seperated values for row {i+1} : \n').strip().split()))
      goalState.append(row)

  else:

    #Define default goal State
    goalState = [[(j + 1) + (n * i) for j in range(n)] for i in range(n)]
    goalState[n-1][n-1] = -1

  #Select an algorithm to run
  print("Please enter the choice of your search algorithm:\n" +
        "1. Uniform Cost Search \n" + 
        "2. A* with the Misplaced Tile heuristic \n"
        "3. A* with the Manhattan Distance heuristic \n")
  
  algorithmChoice = int(input().strip())
    
  if algorithmChoice > 3 and algorithmChoice < 1:
    print("Please choose a valid option\n")

  # ask for time limit of the search for the solution
  timeLimitDecision = input("Do you wish to give a time limit for the search, press Y - Yes or N - No?\n").strip().lower()

  if timeLimitDecision == 'y':
    timeLimit = int(input("Please enter the time limit for search in minutes: \n").strip())
  else:
    timeLimit = 0
  
  return initialState, goalState, algorithmChoice, timeLimitDecision, timeLimit

   

#isStateSame function checks is two states are equal or not
def isStateSame(someState, someOtherState):
  for i in range(len(someOtherState)):
    if someState[i] != someOtherState[i]:
      return False
  return True

#calculateMisplacedTiles function calculates the Misplaced tiles for given state
def calculateMisplacedTiles(someState, goalState):
  misplaced = 0
  for i in range(len(goalState)):
    for j in range(len(goalState)):
      if someState[i][j] == -1:
        continue
      if someState[i][j] != goalState[i][j]:
        misplaced += 1
  return misplaced

# calculateManhattanDistance function calculates the Manhattan distance for given state
def calculateManhattanDistance(someState, goalState):
  all_dist = []
  indexDict = {}
  totalDistance = 0

  # Create a dictionary mapping from goal state elements to theit indices
  for i in range(0, len(someState)):
    for j in range(0, len(someState[i])):
      indexDict[goalState[i][j]] = (i, j)
    
  # Calculate manhattan distance for each element except -1 and only if the elements are not equal to goal state elements in same position.
  for i in range(0, len(someState)):
    for j in range(0, len(someState[i])):
      if someState[i][j] == goalState[i][j] or someState[i][j] == -1:
        continue 
      else:
        iGoalState, jGoalState = indexDict[someState[i][j]]     
        totalDistance += abs(i - iGoalState) + abs(j - jGoalState)
  return totalDistance

"""Class SearchTreeNode handles the tree structure for search. Also defines some functions for:
  1. Adding node to tree
  2. Queuing function for heapq
  3. Expansion of nodes using 4 operator functions"""

class SearchTreeNode:
  def __init__(self, state, parent=None, travelCost=0, estimatedGoalCost=0, totalHeuristicCost=0):
        self.parent = parent
        self.state = state
        self.travelCost = travelCost
        self.estimatedGoalCost = estimatedGoalCost
        self.totalHeuristicCost = totalHeuristicCost
        self.children = []

  def __eq__(self, someState):
    return self.state == someState.state
    
  def __lt__(self, someState):
    #this is the custom comparator to pop the state from the heapq with lowest heuristic cost
    return self.totalHeuristicCost < someState.totalHeuristicCost

  def addNode(self, someNode, expansionCost=1):
        #Add a node to search tree, calculate the travel cost for the newly added node, set parent and append to children list
        someNode.travelCost = self.travelCost + expansionCost
        someNode.totalHeuristicCost = someNode.travelCost + someNode.estimatedGoalCost
        someNode.parent = self
        self.children.append(someNode)

  #Expand function will give all the possible states that can be expanded from current states
  def expand(self):
        # First thing we will need is the index of tile = -1.
        #https://www.digitalocean.com/community/tutorials/python-valueerror-exception-handling-examples

        length = len(self.state)
        rowEmptyTile, colEmptyTile = 0, 0

        for i in range(length):
          try:
            j = self.state[i].index(-1)
            rowEmptyTile, colEmptyTile = i, j
          except ValueError as ve:
            continue
        
        newStates = []

        newStateUp = self.moveUp(rowEmptyTile, colEmptyTile, length)
        newStateDown = self.moveDown(rowEmptyTile, colEmptyTile, length)
        newStateLeft = self.moveLeft(rowEmptyTile, colEmptyTile, length)
        newStateRight = self.moveRight(rowEmptyTile, colEmptyTile, length)

        if newStateUp != None:
          newStates.append(newStateUp)

        if newStateDown != None:
          newStates.append(newStateDown)

        if newStateLeft != None:
          newStates.append(newStateLeft)

        if newStateRight != None:
          newStates.append(newStateRight)

        return newStates

  #Operation move up
  def moveUp(self, rowEmptyTile, colEmptyTile, length):
    if (rowEmptyTile != 0):                                         #Check if there is a row above to move up
      copyState = copy.deepcopy(self.state)
      copyState[rowEmptyTile][colEmptyTile], copyState[rowEmptyTile - 1][colEmptyTile] = copyState[rowEmptyTile - 1][colEmptyTile], -1
            
      # Check if the new state is not the same as parent state to avoid repetition
      if self.parent and isStateSame(copyState, self.parent.state):
        return None
      else:
        return copyState
  
  #Operation move down
  def moveDown(self, rowEmptyTile, colEmptyTile, length):
    if (rowEmptyTile != (length - 1)):                               #Check if there is a row below to move down
      copyState = copy.deepcopy(self.state)
      copyState[rowEmptyTile][colEmptyTile], copyState[rowEmptyTile + 1][colEmptyTile]  = copyState[rowEmptyTile + 1][colEmptyTile], -1

      # Check if the new state is not the same as parent state to avoid repetition
      if self.parent and isStateSame(copyState, self.parent.state):
        return None
      else:
        return copyState

  #Operation move left
  def moveLeft(self, rowEmptyTile, colEmptyTile, length):
    if (colEmptyTile != 0):                                         #Check if there is a column on the left to move
      copyState = copy.deepcopy(self.state)
      copyState[rowEmptyTile][colEmptyTile], copyState[rowEmptyTile][colEmptyTile - 1] = copyState[rowEmptyTile][colEmptyTile - 1], -1

      # Check if the new state is not the same as parent state to avoid repetition
      if self.parent and isStateSame(copyState, self.parent.state):
        return None
      else:
        return copyState

  #Operation move right
  def moveRight(self, rowEmptyTile, colEmptyTile, length):
    if (colEmptyTile != (length - 1)):                              #Check if there is a column on the right to move
      copyState = copy.deepcopy(self.state)
      copyState[rowEmptyTile][colEmptyTile], copyState[rowEmptyTile][colEmptyTile + 1] = copyState[rowEmptyTile][colEmptyTile + 1], -1
            
      # Check if the new state is not the same as parent state to avoid repetition
      if self.parent and isStateSame(copyState, self.parent.state):
        return None
      else:
        return copyState


#generalSearch function is the function that initiates the search and tries to find the solution.
def generalSearch(initialState, goalState, algorithmChoice, timeLimit, verbose="y"):

  rootSearchTree = SearchTreeNode(state=initialState)

  #I am using the below heapq to keep track of states that need to be explored.
  q = [] 
  hq.heappush(q, rootSearchTree)

  #Below list tracks the states that are already explored.
  visited = []
    
  maxQueueLength = 1
  visitedCount = 0

  #setting start time, end time and max time limit (if timelimit is not given by user, set default limit to 10 minutes)
  if timeLimit == 0:
    timeLimit = 10
  searchStartTime = perf_counter()
  searchEndMaxTime = searchStartTime + (timeLimit * 60)
  searchEndTime = searchStartTime
  
  #https://stackoverflow.com/questions/24374620/python-loop-to-run-for-certain-amount-of-seconds
  #Search the tree while we have unexplored states in heapq and while time limit not reached
  while q and (perf_counter() < searchEndMaxTime):
    maxQueueLength = max(len(q), maxQueueLength)
    currState = hq.heappop(q)
    searchTime = (perf_counter() - searchStartTime) * 1000
    #Check is current state is the goal state
    if (isStateSame(currState.state, goalState)):
      print("\nSolution found!")
      print("\nNumber of states expanded: ", visitedCount)
      print("\nDepth of the found solution: ", currState.travelCost)
      print("\nNumber of maximum states in the queue for the search: ", maxQueueLength)
      print(f"\nTime to search the solution: {searchTime:.2f} ms")
      
      return
    
    else:
      visited.append(currState)
      
      #newExpandedStates = [non_none_state for non_none_state in currState.expand() if non_none_state]
      newExpandedStates = currState.expand()

      #There are no expanded states for the current state, move to next state in the heapq
      if len(newExpandedStates) == 0:
        continue
  
      #For every expanded states, we calculate the heuristic cost and push the state in heapq
      for newState in newExpandedStates:
        newStateNode = SearchTreeNode(state=newState)

        #If the new expanded state is already in heapq or is already explored, skip that state
        if ((q and newStateNode in q) or (visited and newStateNode in visited)):
          continue

        #If the heuristic choice is Misplaced Tiles
        if (algorithmChoice == 2):
          newStateNode.estimatedGoalCost = calculateMisplacedTiles(newStateNode.state, goalState)

        #If the heuristic choice is Manhattan Distance
        if (algorithmChoice == 3):
          newStateNode.estimatedGoalCost = calculateManhattanDistance(newStateNode.state, goalState)

        #Add the new expanded state in search tree
        currState.addNode(someNode=newStateNode)

        #Push the new expanded state into the heapq to explore in the future
        hq.heappush(q, newStateNode)

      #Increment the visitedCount, it represents number of nodes expanded
      visitedCount += 1
      
  #If heapq is not empty, the time limit of the search exceeded, else search was unsuccessful
  if q:
    print("Time limit exceeded!\n")
  else:
    print("Search unsuccessful!\n") 

  executionTime = (perf_counter() - searchStartTime) * 1000
  print(f"\n Execution Time: {executionTime:.2f} ms")
  
def main():
  initialState, goalState, algorithmChoice, timeLimitDecision, timeLimit  = getInput()
  generalSearch(initialState, goalState, algorithmChoice, timeLimit)
