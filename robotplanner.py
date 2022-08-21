from pyclbr import Function
from pqdict import pqdict
import numpy as np
import math
import time

# functions to time how long planning takes  
def tic():
  return time.time()
def toc(tstart, nm=""):
  print('%s took: %s sec.\n' % (nm,(time.time() - tstart)))

''' A* ALGORITHM '''
def A_star(envmap,start,goal,epsilon):
  # input: environment (array), start node class, goal node class, heuristic epsilon weight
  # output: path to goal/evader

  #print('Creating OPEN and CLOSED sets')
  OPEN = pqdict() # OPEN list (key = node_id: value = f)
  CLOSED =[]      # CLOSED list

  graph = {start.id:start} # dictionary of nodes
  start.g_value = 0
  s = start.id
  graph[s].h_value = np.linalg.norm(graph[s].coordinates-goal.coordinates) # Euclidean norm for heuristic
  f_start = graph[s].g_value + epsilon*graph[s].h_value             # compute priority

  OPEN.additem(start.id,f_start)  # initially populate OPEN list with start node

  while goal.id not in CLOSED:

    '''Consider tie-breaking strategy, for now just picks lowest indexed'''
    smallest_i, _ = OPEN.popitem()     # remove node from OPEN with smallest f

    graph[smallest_i].closed = True     # flag smallest node as CLOSED
    CLOSED.append(smallest_i)           # add removed node to CLOSED list

    '''GET SUCCESSORS'''
    succNodes, succCost, succControl = getSuccessors(graph[smallest_i],envmap) # returns all successor properties from parent i
    
    '''LABEL CORRECTING'''
    for j in succNodes:
      # add successor to graph
      if j not in graph:
        graph[j] = node(j)

      if graph[j].id not in CLOSED:

        '''EXPAND NODE j'''
        gi_value_plus_cost = graph[smallest_i].g_value + succCost[j]
        if graph[j].g_value > gi_value_plus_cost:
          # Label Correction
          graph[j].g_value = gi_value_plus_cost                             # assign new label to j
          graph[j].parent_id = smallest_i                                   # assign parent to j
          graph[j].parent_action_id = succControl[j]                        # assigning control action which takes successor to parent
          
          if j in OPEN:
            # update the priority value of successor
            priority = graph[j].g_value + epsilon*graph[j].h_value

            OPEN.updateitem(j,priority)      # update priority of node 
          else:
            # add the successor to OPEN set
            graph[j].h_value = np.linalg.norm(graph[j].coordinates-goal.coordinates)  # assign heuristic
            priority = graph[j].g_value + epsilon*graph[j].h_value                    # assign priority to new OPEN set member
            OPEN.additem(j,priority)                                   
  
  '''RETRIEVING OPTIMAL PATH'''
  optimal_path, path_cost = recoverPath(graph[goal.id],graph[start.id],graph)

  return optimal_path, path_cost

'''Node Class'''
class node:
  # class representing the node/vertex of graph

  # Constructor
  def __init__(self,coordinates):
    # use coordinates as node identifier
    self.id = coordinates                   # tuple
    self.coordinates = np.asarray(self.id)  # numpy array
  # Class Variables
  g_value = np.inf        # label, all nodes start off as inf g until expanded 
  h_value = 0             # heuristic at node i
  parent_id = 0           # id for parent
  parent_action_id = 0    # control action which takes child to parent
  closed = False          # True if in CLOSED list, False otherwise

def getSuccessors(node,envmap):
    # Input:  node (class), envmap (array)
    # Output: S (list of successor nodes)
    #         S_cost (dictionary of stage cost of each successor)
    #         S_control (dictionary of control ids for each node)
    S = []
    S_cost = {}       # keys are successor id
    S_control = {}    # keys are successor id
    for i in [0,-1,1]:
      for j in [0,-1,1]:
        if i == 0 and j == 0:
          #skip, this is parent coordinates
          pass     
        else:
          #check that the new commanded position is valid
          successor_coords = [node.coordinates[0]+i,node.coordinates[1]+j] # candidate successor position

          if ( successor_coords[0] < 0 or successor_coords[0] >= envmap.shape[0] or \
              successor_coords[1] < 0 or successor_coords[1] >= envmap.shape[1] ):
            pass
          else:
            if envmap[successor_coords[0],successor_coords[1]] == 1:
              # in obstacle, not a node
              pass          
            else:
              successor = (successor_coords[0],successor_coords[1]) # write successor coordinates as tuple
              # add vallid successor to successor list
              S.append(successor)
              # compute cost and control action necessary to reach successor
              control = [i,j]  # control action to get from parent to successor
              if control in [[1,1],[-1,1],[-1,-1],[1,-1]]:
                cost = np.sqrt(2) # diagonal move
              elif control in [[0,1],[1,0],[0,-1],[-1,0]]:
                cost = 1          # cardinal move
              else:
                cost = np.inf
              # create dictionary entry for cost of current successor
              S_cost[successor] = cost
              # create dictionary entry for control of current successor
              S_control[successor] = control

    return S, S_cost, S_control

def recoverPath(goal, start, graph):  
  # Input: goal node, start node, graph as dictionary
  # Output: path - list off nodes to get from start to goal, path_cost - cost of moving along entire path

  path = []
  path_cost = goal.g_value
  curr_node = goal

  while curr_node.id != start.id:   
    pid = curr_node.parent_id # parent id
    path.insert(0,pid)        # add parent to the path
    curr_node = graph[pid]    # new current node is parent

  return path, path_cost  

# Weighted A*
def robotplanner1(envmap, robotpos, targetpos):
  # The Weighted A* Motion Planner
  # Inputs: envmap (array), robotpos (array), targetpos (array)
  # Outputs: newrobotpos (array)
    
  # A* Algorithm
  newrobotpos_asTuple = (robotpos[0],robotpos[1]) # create tuple id
  targetpos_asTuple = (targetpos[0],targetpos[1]) # create tuple id
  start = node(newrobotpos_asTuple)
  goal = node(targetpos_asTuple)
  
  '''EPSILON'''
  ###############################################
  epsilon = 5              # heuristic weight
  ###############################################

  opt_Path, path_cost = A_star(envmap,start,goal,epsilon)

  'WEIGHTED A*'
  newrobotpos = np.asarray(opt_Path[1])

  return newrobotpos

# Delayed Weighted A*
def robotplanner2(envmap, robotpos, targetpos):
  # The Delayed Weighted A* motion planner
  # Inputs: envmap (array), robotpos (array), targetpos (array)
  # Outputs: newrobotpos (array)
    
  # A* Algorithm
  newrobotpos_asTuple = (robotpos[0],robotpos[1]) # create tuple id
  targetpos_asTuple = (targetpos[0],targetpos[1]) # create tuple id
  start = node(newrobotpos_asTuple)
  goal = node(targetpos_asTuple)

  '''EPSILON'''
  ###############################################
  epsilon = 5            # heuristic weight
  ###############################################
  opt_Path, path_cost = A_star(envmap,start,goal,epsilon)
  
  'DELAYED WEIGHTED A*'
  newrobotpos = np.asarray(opt_Path[1:101])

  return newrobotpos