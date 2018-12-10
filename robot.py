import numpy as np
from collections import namedtuple, deque, Counter, defaultdict
import heapq
from enum import Enum

class RobotPath(Enum):
    forward = 0
    backTrace = 1
    fastFwd = 2

dir_move = {'up': [0, 1], 'right': [1, 0], 'down': [0, -1], 'left': [-1, 0]}
dir_sensors = {'up': ['left', 'up', 'right'], 'right': ['up', 'right', 'down'],
               'down': ['right', 'down', 'left'], 'left': ['down', 'left', 'up']}

#goal_bounds  = []
firstTime = True
coord = (0,0,'up')
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
        self.goal_bounds = [maze_dim/2 - 1, maze_dim/2]
        print('goal bounds', self.goal_bounds)
        self.currentNode = self.root
        self.currentPath = RobotPath.forward
        self.backTracePath = []
        self.fwdNode = []
        self.frontier = PriorityQueue(min, self.cost)
        #self.frontier.append(self.root)
        self.explored = set()
        self.explored.add(self.root.state)

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
        '''
        if len(self.path) == 0:
            pass
        else:
            node = best_first_graph_search(self, self.currentNode, sensors, cost)
            self.currentNode = node
        '''
        print('nextmove', self.currentPath)
        node = []
        global coord
        if self.currentPath == RobotPath.forward:
            rotation, movement = self.best_first_graph_search(self.currentNode, sensors, self.cost)
            self.nextState(rotation, movement)
            print(rotation, movement, coord)
            return rotation, movement
        elif self.currentPath == RobotPath.backTrace:
            rotation, movement = self.processBackTrace()
            self.nextState(rotation, movement)
            print(rotation, movement, coord)
            return rotation, movement
        else:
            print(self.currentNode,self.fwdNode)
            if self.currentNode.state == self.fwdNode.state:
                self.currentPath = RobotPath.forward
                self.fwdNode = []
                print('0,0,', coord)
                return 0,0
            else:
                newNode = []
                for nd in self.fwdNode.childNodes:
                    if self.checkChildRecusively1(nd):
                        newNode = nd
                        break
                self.currentNode = newNode
                print('FFWDNM ', newNode.direction, newNode.distance, sensors, newNode.state, newNode.heading)
                self.nextState(newNode.direction, newNode.distance)
                print(newNode.direction, newNode.distance, coord)
                return newNode.direction, newNode.distance
        
        
    def nextState(self, rotation, movement):
        dr = 0
        global coord
        if rotation==-90:
            dr = dir_sensors[coord[2]][0]
        elif rotation == 0:
            dr = dir_sensors[coord[2]][1]
        else:
            dr = dir_sensors[coord[2]][2]
        coord = (movement*dir_move[dr][0] + coord[0],  movement*dir_move[dr][1] + coord[1], dr)
    
    def leastNumberSteps(self):
        return 0
    
    def goal_test(self, state):
        if state[0] in goal_bounds and state[1] in goal_bounds:
            return True
        else:
            return False
        
    def cost(self, node): #path cost + heuristic
        return node.mazeDistance + self.heuristic(node, self.manhattanHeuristic)

    def heuristic(self, node, f):
        min0 = min(f((self.goal_bounds[0], self.goal_bounds[0]),node.state), 100000)
        min1 = min(f((self.goal_bounds[0], self.goal_bounds[1]),node.state), min0)
        min2 = min(f((self.goal_bounds[1], self.goal_bounds[0]),node.state), min1)
        min3 = min(f((self.goal_bounds[1], self.goal_bounds[1]),node.state), min2)
        return min3
    
    def manhattanHeuristic(self, xy1, xy2):
        return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])
    
    def euclideanHeuristic(self, xy1, xy2):
        return ( (xy1[0] - xy2[0]) ** 2 + (xy1[1] - xy2[1]) ** 2 ) ** 0.5
        
    
    def best_first_graph_search(self, node, sensors, f):
        '''
        if robot.goal_test(node.state):
            return node
        frontier = PriorityQueue(cost)
        frontier.append(node)
        explored = set()
        while frontier:
            node = frontier.pop()
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
        '''
        #frontier.append(node)
        junction = [1 for i in sensors if i>0]
        newNode = []
        if all(junction) == False:
            #pass# it is a dead end
            print('dead end')
            node.mazeDistance = 100000
            self.currentPath == RobotPath.backTrace
            self.backTracePath = [node, node.parent]
            global firstTime
            firstTime = True
            return 0,-1*node.value
            #set mazedistance to infinity so that this node will never be reached
        else:
            #pass # it is not a junction
            childrenNodes = node.expand(sensors)
            self.explored.add(node.state)
            for child in childrenNodes:
                if child.state not in self.explored and child not in self.frontier:
                    self.frontier.append(child)
                    #node = child
                elif child in self.frontier:
                    incumbent = self.frontier[child]
                    if f(child) < f(incumbent):
                        # del frontier[incumbent]
                        self.frontier.append(child)
        
            newNode = self.frontier.pop()
            print('b4newNode ', newNode.direction, newNode.distance, sensors, newNode.state, newNode.heading)
            if newNode in childrenNodes:
                self.currentPath == RobotPath.forward
                self.backTracePath = []
                self.currentNode = newNode
                print('newNode ', newNode.direction, newNode.distance, sensors, newNode.state, newNode.heading)
                return newNode.direction, newNode.distance
            elif self.checkChildRecusively1(node, newNode):
                self.currentPath = RobotPath.fastFwd
                self.backTracePath = []
                self.fwdNode = newNode
                for nd in node.childNodes:
                    if self.checkChildRecusively1(nd, self.fwdNode):
                        newNode = nd
                        break
                self.currentNode = newNode
                print('FFWD ', newNode.direction, newNode.distance, sensors, newNode.state, newNode.heading)
                return newNode.direction, newNode.distance
            else:
                self.currentPath = RobotPath.backTrace
                firstTime = True
                self.backTracePath = [node, newNode]
                print('BK', newNode.direction, newNode.distance, sensors, newNode.state, newNode.heading)
                return 0,-1*node.distance
                
                #rotation,movement = self.processBackTrace()
                #return rotation, movement
         
        
        
        
    def processBackTrace(self):
        
        node = self.backTracePath[0]
        #Process Code
        checkPath = True
        if node.parent:
            checkPath = self.checkChildRecusively(node.parent) 
        print('processBackTrace', checkPath, self.backTracePath)
        if checkPath:
            #if node == self.backTracePath[1]:
            if node.parent.state == self.backTracePath[1].state:
                #handle direction
                if node.heading != self.backTracePath[1].heading:
                    if (node.heading == 'up' and selfbackTracePath[1].heading == 'down') or (node.heading == 'down' and selfbackTracePath[1].heading == 'or') or (node.heading == 'left' and self.backTracePath[1].heading == 'right') or (node.heading == 'right' and self.backTracePath[1].heading == 'left'):
                        return -90,0
                    else:
                        self.currentNode = self.backTracePath[1]
                        self.currentPath = RobotPath.forward
                        self.backTracePath = []
                        direction = 0
                        if (node.heading == 'up' and self.backTracePath[1].heading == 'left'):
                            direction = -90
                        if (node.heading == 'up' and self.backTracePath[1].heading == 'right'):
                            direction = 90
                        if (node.heading == 'down' and self.backTracePath[1].heading == 'left'):
                            direction = 90
                        if (node.heading == 'down' and self.backTracePath[1].heading == 'right'):
                            direction = -90
                        return direction, 0
                else:
                    self.currentNode = self.backTracePath[1]
                    self.currentPath = RobotPath.forward
                    self.backTracePath = []
                    print('eq', self.currentNode)
                    return 0,0 # can start the forward A* process
            else:
                #Semi forward code
                global firstTime
                if firstTime:
                    '''
                    firstTime = False
                    direction = -1*(node.direction) + node.parent.direction
                    value = 0
                    self.backTracePath[0] = node.parent
                    print('firstTIme', direction, value)
                    return direction, value
                    '''
                    if node.heading != node.parent.heading:
                        if (node.heading == 'up' and node.parent.heading == 'down') or (node.heading == 'down' and node.parent.heading == 'or') or (node.heading == 'left' and node.parent.heading == 'right') or (node.heading == 'right' and node.parent.heading == 'left'):
                            firstTime = True
                            print('firstTIme if -90, 0')
                            return -90,0
                        else:
                            firstTime = False
                            direction = 0
                            if (node.heading == 'up' and node.parent.heading == 'left'):
                                direction = -90
                            if (node.heading == 'up' and node.parent.heading == 'right'):
                                direction = 90
                            if (node.heading == 'down' and node.parent.heading == 'left'):
                                direction = 90
                            if (node.heading == 'down' and node.parent.heading == 'right'):
                                direction = -90
                            if (node.heading == 'left' and node.parent.heading == 'up'):
                                direction = 90
                            if (node.heading == 'left' and node.parent.heading == 'down'):
                                direction = -90
                            if (node.heading == 'right' and node.parent.heading == 'up'):
                                direction = -90
                            if (node.heading == 'right' and node.parent.heading == 'down'):
                                direction = 90
                            self.backTracePath[0] = node.parent
                            print('firstTIme else', direction, ' 0')
                            return direction, 0
                    else:
                        firstTime = False
                        direction = 0
                        value = 0
                        self.backTracePath[0] = node.parent
                        print('firstTIme', direction, value)
                        return direction, value
                    
                if node.state == self.backTracePath[1].state:
                    if node.heading != self.backTracePath[1].heading:
                        if (node.heading == 'up' and self.backTracePath[1].heading == 'down') or (node.heading == 'down' and self.backTracePath[1].heading == 'or') or (node.heading == 'left' and self.backTracePath[1].heading == 'right') or (node.heading == 'right' and self.backTracePath[1].heading == 'left'):
                            return -90,0
                        else:
                            self.currentNode = self.backTracePath[1]
                            self.currentPath = RobotPath.forward
                            self.backTracePath = []
                            direction = 0
                            if (node.heading == 'up' and self.backTracePath[1].heading == 'left'):
                                direction = -90
                            if (node.heading == 'up' and self.backTracePath[1].heading == 'right'):
                                direction = 90
                            if (node.heading == 'down' and self.backTracePath[1].heading == 'left'):
                                direction = 90
                            if (node.heading == 'down' and self.backTracePath[1].heading == 'right'):
                                direction = -90
                            return direction, 0
                    else:
                        self.currentNode = node
                        self.currentPath = RobotPath.forward
                        self.backTracePath = []
                        print('node ', node.direction, node.distance, node.state, node.heading)
                        return 0,0 # can start the forward A* process
                else:
                    for nd in self.backTracePath[0].childNodes:
                        if self.checkChildRecusively(nd):
                            self.backTracePath[0] = nd
                            return nd.direction, nd.distance
                    print('Not suppose to reach here')
        else:
            #if len(node.childNodes.values()[0].chilNodes.values()) == 0: #the very first one in backtrace path for not considering direction
            if False:
                #node = node.parent
                self.backTracePath[0] = node
                print('inside false')
                return 0,-1*node.value
            else:
                reverseDirection = -1*(node.direction) # how to find which child node
                node = node.parent
                self.backTracePath[0] = node
                print('revere', reverseDirection)
                return reverseDirection,-1*node.distance
        return node
        
    def checkChildRecusively(self, node):
        if node.state == self.backTracePath[1].state:
            return True
        try:
            for nd in node.childNodes:
                if nd.state == self.backTracePath[1].state:
                    return True
                if self.checkChildRecusively(nd):
                    return True
        except:
            print('An error occured checkChildRecusively.')
        return False
    
    def checkChildRecusively1(self, start, end):
        #print('checkChildRecusively1 ', start.state, end.state)
        if start.state == end.state:
            return True
        try:
            for nd in start.childNodes:
                #print('nd ', nd.state )
                if nd.state == end.state:
                    return True
                if self.checkChildRecusively(nd, end):
                    return True
        except:
            print('An error occured checkChildRecusively1.')
        return False
        
class Node(object):
    def __init__(self, parent = [], distance = 0, mazeDistance = 0, state=(0,0), heading= 'up', direction = 0):
        self.parent = parent
        self.depth = 0
        if parent:
            self.depth = parent.depth + 1
        self.distance = distance #distance = path cost
        self.mazeDistance = mazeDistance
        self.state = state
        self.heading = heading
        self.direction = direction
        self.childNodes = []
    
    def expand(self, sensors):
        #return (self.child_node(problem, action) for action in problem.actions(self.state))        
        nodesInfo = [(j,k) for i,j in enumerate([-90,0,90]) for k in range(1,min(sensors[i],3)+1) if sensors[i]!=0]
        #nodesInfo = [(j,1) for i,j in enumerate([-90,0,90]) if sensors[i]!=0]
        #print(nodesInfo, sensors)
        #childNodes = dict()
        for key,value in nodesInfo:
            state = (0,0)
            if key==-90:
                heading = dir_sensors[self.heading][0]
                '''
                if value > 0:
                    state = (self.state[0]-j, self.state[1])
                else:
                    state = (self.state[0]-j, self.state[1])
                '''
                state = (self.state[0]+(value * dir_move[heading][0])), (self.state[1]+(value * dir_move[heading][1]))
            elif key==0:
                heading = dir_sensors[self.heading][1]
                state = (self.state[0]+(value * dir_move[heading][0])), (self.state[1]+(value * dir_move[heading][1]))
            else:
                heading = dir_sensors[self.heading][2]
                state = (self.state[0]+(value * dir_move[heading][0])), (self.state[1]+(value * dir_move[heading][1]))
            #self.childNodes[(key,value)] = Node(self, value, value + self.distance, state, heading, key)
            self.childNodes.append(Node(self, value, value + self.distance, state, heading, key))
        return self.childNodes
    '''
    def __repr__(self):
        return "<Node %s>" % (self.state,)
    
    
    def __eq__(self, other):
        return isinstance(other, Node) and self.state == other.state
        #return self.__dict__ == other.__dict__
    
    def __hash__(self):
        return hash(self.state)
    '''
    '''
    def child_node(self, problem, action):
        "[Figure 3.10]"
        next_state = problem.result(self.state, action)
        return Node(next_state, self, action,
                    problem.path_cost(self.path_cost, self.state,
                                      action, next_state))
    '''
    
class Queue:
    """Queue is an abstract class/interface. There are three types:
        Stack(): A Last In First Out Queue.
        FIFOQueue(): A First In First Out Queue.
        PriorityQueue(order, f): Queue in sorted order (default min-first).
    Each type supports the following methods and functions:
        q.append(item)  -- add an item to the queue
        q.extend(items) -- equivalent to: for item in items: q.append(item)
        q.pop()         -- return the top item from the queue
        len(q)          -- number of items in q (also q.__len())
        item in q       -- does q contain item?
    Note that isinstance(Stack(), Queue) is false, because we implement stacks
    as lists.  If Python ever gets interfaces, Queue will be an interface."""

    def __init__(self):
        raise NotImplementedError

    def extend(self, items):
        for item in items:
            self.append(item)

class PriorityQueue(Queue):
    """A queue in which the minimum element (as determined by f and
    order) is returned first.  Also supports dict-like lookup.

    MODIFIED FROM AIMA VERSION
        - Use heapq
        - Use an additional dict to track membership
    """

    def __init__(self, order=None, f=lambda x: x):
        self.A = []
        self._A = Counter()
        self.f = f

    def append(self, item):
        heapq.heappush(self.A, (self.f(item), item))
        self._A[item] += 1

    def __len__(self):
        return len(self.A)

    def pop(self):
        #print(self.A)
        _, item = heapq.heappop(self.A)
        #print('PQ Popped ', self._A[item])
        self._A[item] -= 1
        return item

    def __contains__(self, item):
        return self._A[item] > 0

    def __getitem__(self, key):
        if self._A[key] > 0:
            return key
        

