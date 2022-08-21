import numpy as np
import math
from numpy import loadtxt
import matplotlib.pyplot as plt
plt.ion()
import time
from robotplanner import *
from targetplanner import targetplanner

# functions to time how long planning takes  
def tic():
  return time.time()
def toc(tstart, nm=""):
  print('%s took: %s sec.\n' % (nm,(time.time() - tstart)))

# run a planning algorithm on specified map
def runtest(mapfile, robotstart, targetstart):
  # current positions of the target and robot
  robotpos = np.copy(robotstart)
  targetpos = np.copy(targetstart)
  
  # environment
  envmap = loadtxt(mapfile)
    
  # draw the environment
  # transpose because imshow places the first dimension on the y-axis
  f, ax = plt.subplots()
  ax.imshow( envmap.T, interpolation="none", cmap='gray_r', origin='lower', \
             extent=(-0.5, envmap.shape[0]-0.5, -0.5, envmap.shape[1]-0.5) )
  ax.axis([-0.5, envmap.shape[0]-0.5, -0.5, envmap.shape[1]-0.5])
  ax.set_xlabel('x')
  ax.set_ylabel('y')  
  hr = ax.plot(robotpos[0], robotpos[1], 'bs')
  ht = ax.plot(targetpos[0], targetpos[1], 'rs')
  f.canvas.flush_events()
  plt.show()
  
  # to get first frame of plot
  #return 

  '''PLANNING ALGORITHM FLAG'''
  ##############################################################
  algorithm = 1     # 0 = weighted A*; 1 = delayed weighted A*
  ##############################################################

  # print to terminal what algorithm is being run
  if algorithm == 0:
    print('Using Weighted A*')
    print('__________________\n')
  elif algorithm == 1:
    print('Using Delayed Weighted A*')
    print('__________________________\n') 

  # now comes the main loop
  numofmoves = 0
  caught = False
  for i in range(20000):
    # call robot planner
    t0 = tic()
    
    if algorithm == 0:

      'WEIGHTED A*'
      ############################################################
      newrobotpos = robotplanner1(envmap, robotpos, targetpos)
      ############################################################ 
      flag = 1
      # Note: Will take exorbinant amount of time on maps 1, 3bc and 7

    elif algorithm == 1:
      'DELAYED WEIGHTED A*'
      ############################################################
      # Check how far pursuer is from evader
      r = np.linalg.norm(robotpos-targetpos)
      if r > 30:
        # only recompute path every 100 moves
        if i % 100 ==0:
          # if i is divisible by 100 and outside radius, call delayed weighted A*
          print('Outside radius, precomputing path...')
          newrobotpos_list = robotplanner2(envmap, robotpos, targetpos) # returns precomputed path list of 100 moves
          print('Path Computed!')
          flag = 0
      else:
        # pursuer within radius of evader
        # switch to regular A*
        print('Inside radius, swapped to Weighted A*')
        newrobotpos = robotplanner1(envmap,robotpos,targetpos) # returns a coordinate
        flag = 1 
      ###########################################################
      # Note: Scalable to all maps except 7
    else:
      print('ERROR: Invalid Algorithm Chosen')
      break
    
    # Only reassign newrobotpos as entry of list when precomputed path is used
    if flag == 0:
      newrobotpos = newrobotpos_list[i%100]

    'UNCHANGED'
    # compute move time for the target, if it is greater than 2 sec, the target will move multiple steps
    movetime = max(1, math.ceil((tic()-t0)/2.0))
    toc(t0,"Total Robot Planning Time")
    
    #check that the new commanded position is valid
    if ( newrobotpos[0] < 0 or newrobotpos[0] >= envmap.shape[0] or \
         newrobotpos[1] < 0 or newrobotpos[1] >= envmap.shape[1] ):
      print('ERROR: out-of-map robot position commanded\n')
      break
    elif ( envmap[newrobotpos[0], newrobotpos[1]] != 0 ):
      print('ERROR: invalid robot position commanded\n')
      break
    elif (abs(newrobotpos[0]-robotpos[0]) > 1 or abs(newrobotpos[1]-robotpos[1]) > 1):
      print('ERROR: invalid robot move commanded\n')
      break

    # call target planner to see how the target moves within the robot planning time
    newtargetpos = targetplanner(envmap, robotpos, targetpos, targetstart, movetime)
    
    # make the moves
    robotpos = newrobotpos
    hr.append(ax.plot(robotpos[0], robotpos[1], 'go'))
    targetpos = newtargetpos
    numofmoves += 1
    
    # draw positions
    hr[0].set_xdata(robotpos[0])
    hr[0].set_ydata(robotpos[1])
    ht[0].set_xdata(targetpos[0])
    ht[0].set_ydata(targetpos[1])
    f.canvas.flush_events()
    plt.show()    

    # check if target is caught
    if (abs(robotpos[0]-targetpos[0]) <= 1 and abs(robotpos[1]-targetpos[1]) <= 1):
      print('robotpos = (%d,%d)' %(robotpos[0],robotpos[1]))
      print('targetpos = (%d,%d)' %(targetpos[0],targetpos[1]))
      caught = True

      break

  return caught, numofmoves

''' TEST MAPS '''
def test_map0():
  # Small  Map
  robotstart = np.array([0, 2])
  targetstart = np.array([5, 3])
  return runtest('maps/map0.txt', robotstart, targetstart)

def test_map1():
  # Large Map
  robotstart = np.array([699, 799])
  targetstart = np.array([699, 1699])
  return runtest('maps/map1.txt', robotstart, targetstart)

def test_map2():
  # Small Map - Agent Trapped
  robotstart = np.array([0, 2])
  targetstart = np.array([7, 9])
  return runtest('maps/map2.txt', robotstart, targetstart)
  
def test_map3():
  # Med Map - Rooms
  robotstart = np.array([249, 249])
  targetstart = np.array([399, 399])
  return runtest('maps/map3.txt', robotstart, targetstart)

def test_map4():
  # Small Map - Cooridoor
  robotstart = np.array([0, 0])
  targetstart = np.array([5, 6])
  return runtest('maps/map4.txt', robotstart, targetstart)

def test_map5():
  # Small Map - Narrow Passages
  robotstart = np.array([0, 0])
  targetstart = np.array([29, 59])
  return runtest('maps/map5.txt', robotstart, targetstart)

def test_map6():
  # Medium Map - Narrow Wall at Top
  robotstart = np.array([0, 0])
  targetstart = np.array([29, 36])
  return runtest('maps/map6.txt', robotstart, targetstart)

def test_map7():
  # HUGE Map
  robotstart = np.array([0, 0])
  targetstart = np.array([4998, 4998])
  return runtest('maps/map7.txt', robotstart, targetstart)

def test_map1b():
  robotstart = np.array([249, 1199])
  targetstart = np.array([1649, 1899])
  return runtest('maps/map1.txt', robotstart, targetstart)

def test_map3b():
  robotstart = np.array([74, 249])
  targetstart = np.array([399, 399])
  return runtest('maps/map3.txt', robotstart, targetstart)

def test_map3c():
  robotstart = np.array([4, 399])
  targetstart = np.array([399, 399])
  return runtest('maps/map3.txt', robotstart, targetstart)
  
if __name__ == "__main__":
  # you should change the following line to test different maps  
  caught, numofmoves = test_map1()  # change # in test_map# to try different maps 
  #test_map3c()
  print('Number of moves made: {}; Target caught: {}.\n'.format(numofmoves, caught))
  plt.ioff()
  plt.show()