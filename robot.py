import numpy as np
from _overlapped import NULL
from Queue import PriorityQueue

class Robot(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''

        self.location = [0, 0]
        self.heading = 'up'
        self.maze_dim = maze_dim
        self.path = []
        self.root = Node()
        goal_bounds = [maze_dim/2 - 1, maze_dim/2]
        

    def next_move(self, sensors):
        '''
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.

        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.

        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returing the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        '''

        rotation = 0
        movement = 0
        
        if len(self.path) == 0:
            pass
        else:
            best_first_graph_search(self, self.root, sensors)
            pass

        return rotation, movement
    
    def leastNumberSteps(self):
        return 0
    
    def goal_test(self, state):
        if state[0] in goal_bounds and state[1] in goal_bounds:
            return True
        else:
            return False
        
        
class Node(object):
    def __init__(self, parent = NULL, distance = 0, state=(0,0), heading='up'):
        self.parent = parent
        self.distance = distance #distance = path cost + neuristic cost to be coded
        self.state = state
        self.heading = heading
    
    def expand(self, sensors):
        #return (self.child_node(problem, action) for action in problem.actions(self.state))
        return [(i,j) for i in [-90,0,90] for j in sensors if j!=0]
    
    def child_node(self, problem, action):
        "[Figure 3.10]"
        next_state = problem.result(self.state, action)
        return Node(next_state, self, action,
                    problem.path_cost(self.path_cost, self.state,
                                      action, next_state))
    
        
        
def best_first_graph_search(robot, node, sensors):
    """Search the nodes with the lowest f scores first.
    You specify the function f(node) that you want to minimize; for example,
    if f is a heuristic estimate to the goal, then we have greedy best
    first search; if f is node.depth then we have breadth-first search.
    There is a subtlety: the line "f = memoize(f, 'f')" means that the f
    values will be cached on the nodes as they are computed. So after doing
    a best first search you can examine the f values of the path returned."""
    #f = memoize(f, 'f')
    #node = Node(problem.initial)
    if robot.goal_test(node.state):
        return node
    frontier = PriorityQueue()
    frontier.put((-node.distance, node))
    explored = set()
    while frontier:
        node = frontier.get()
        if robot.goal_test(node.state):
            return node
        explored.add(node.state)
        for child in node.expand(sensors):
            if child.state not in explored and child not in frontier:
                frontier.put((child.distance, child))
            elif child in frontier:
                incumbent = frontier[child]
                if f(child) < f(incumbent):
                    # del frontier[incumbent]
                    frontier.append(child)
    return None