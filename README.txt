Required Installs:
pyclbr
pqdict
numpy
time
matplotlib

Directions and Function Descriptions:
_____________________________________
_____________________________________

Directions:
___________

To run project, call main.py in terminal with python.

In order to vary maps and choose which algorithm to use (Weighted A*, or Delayed Weighted A*),
user must alter certain flags.

*Choosing Map*
In main.py alter line 205 by changing the # of the test_map#() function.

*Choosing Algorithm*
In main.py line 40, alter the algorithm flag.
	0 -> weighted A*
	1 -> delayed weighted A*

*Changing Epsilon (A* Heuristic Weight)*
In robotplanner.py line 161 (or line 185 if using Delayed algorithm) change
value of epsilon variable to any value >= 1.

Function Descriptions:
______________________

Only custom and altered functions will be described here.

From main.py:

1.) runtest(mapfile, robotstart, targetstart):

	Alterations:

	Added flag check called algorithm which determines which algorithm
	will be used to motion plan.
	algorithm = 0 -> Weighted A*
	algorithm = 1 -> Delayed Weighted A*

	Depending on which algorithm is chosen, calls robotplanner1 (Weighted A*)
	or robotplanner2 (Delayed Weighted A*) and returns either:
	newrobotpos (array)	 <- Weighted A*
	newrobotpos_list (list)  <- Delayed Weighted A*

	If Delayed Weighted A* is chosen, the Euclidean norm between the target robot
	and pursuer robot is queried and if it exceeds 30 units the Delayed Weighted A*
	algorithm is employed. Otherwise, the normal Weighted A* algorithm is used. If
	the distance between the target and pursuer is below 30 units the precomputed path
	is discarded and Weighted A* is used.

From robotplanner.py:

1.) A_star(envmap,start,goal,epsilon)

	Input: environment (array), start node class, goal node class, heuristic epsilon weight
  	Output: path to goal/evader

	The Main A* algorithm. Returns the best found path to the evader based on provided
	epsilon value.

2.) getSuccessors(node,envmap)

	Input:  node (class), envmap (array)
     	Output: S (list of successor nodes)
             	S_cost (dictionary of stage cost of each successor)
             	S_control (dictionary of control ids for each node)

	Takes a node and the environment to return it's children, cost to the children, and
	control necessary to reach the parent.

3.) recoverPath(goal,start,graph)

	Input: goal node, start node, graph as dictionary
  	Output: path - list off nodes to get from start to goal, path_cost - cost of moving along entire path
	
    	Takes the start, goal, and graph nodes and generates the best path through the graph from the
	goal using the parents of the goal.

4.) robotplanner1(envmap,robotpos,targetpos)

	Inputs: envmap (array), robotpos (array), targetpos (array)
  	Outputs: newrobotpos (array)

	Weighted A*: Generates the next robot position using the weighted A* algorithm. Allows
	user to change the heuristic weight epsilon for the A* function.


5.) robotplanner2(envmap,robotpos,targetpos)

	Inputs: envmap (array), robotpos (array), targetpos (array)
  	Outputs: newrobotpos (array)

	Delayed Weighted A*: Generates a path of the next 100 robot positions using the weighted A* algorithm. Allows
	user to change the heuristic weight epsilon for the A* function.