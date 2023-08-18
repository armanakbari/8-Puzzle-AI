from operator import mod
from queue import LifoQueue 
from tracemalloc import start
import copy
from time import time
import tracemalloc
from queue import PriorityQueue

initial_state = []
goal_state = [1, 2, 3, 4, 5, 6, 7, 8, 0]
total_expanded_nodes = 0

def change_the_input_output(arr):
    out = []
    i = 0
    temp = []
    while i < len(arr):
        temp.append(arr[i])
        i+=1
        temp.append(arr[i])
        i+=1
        temp.append(arr[i])
        i+=1
        out.append(temp)
        temp = []
    return out  

def printpuzzle(arr, prevarr):
    for i in range(3):
        for j in range(3):
            if arr[i][j] == 0:
                a, b = i, j
            if prevarr[i][j] == 0:
                c, d = i, j
    if c + 1 == a:
        print("\'Right\',", end=" ")
    elif c == a + 1:
        print("\'Left\',", end=" ")
    elif d + 1 == b:
        print("\'Down\',", end=" ")
    else:
        print("\'Up\',", end=" ")

class Node:
    def __init__(self, state, parent, operator, depth, cost):
        global total_expanded_nodes
        self.state = state
        self.parent = parent
        self.operator = operator
        self.depth = depth
        self.cost = cost
        self.heuristic = None

def checkgoal(start_d, goal_d):  
    for i in range(3):
        for j in range(3):
            if start_d[i][j] != goal_d[i][j]:
                return False
    return True

def move_dfs(current):  
    x, y = 0, 0
    for i in range(3):
        for j in range(3):
            if current[i][j] == 0:
                x, y = i, j
    movelist = [[x, y, x + 1, y], [x, y, x - 1, y], [x, y, x, y + 1], [x, y, x, y - 1]]
    return movelist

def move_up(state):
    new_state = state[:]
    index = new_state.index(0)
    if index not in [0, 1, 2]:
        new_state[index] , new_state[index - 3] = new_state[index - 3] , new_state[index]
        return new_state
    else:
        return None

def move_down(state):
    new_state = state[:]
    index = new_state.index(0)
    if index not in [6, 7, 8]:
        new_state[index] , new_state[index + 3] = new_state[index + 3] , new_state[index]
        return new_state
    else:
        return None

def move_left(state):
    new_state = state[:]
    index = new_state.index(0)
    if index not in [0, 3, 6]:
        new_state[index] , new_state[index - 1] = new_state[index - 1] , new_state[index]
        return new_state
    else:
        return None

def move_right(state):
    new_state = state[:]
    index = new_state.index(0)
    if index not in [2, 5, 8]:
        new_state[index] , new_state[index + 1] = new_state[index + 1] , new_state[index]
        return new_state
    else:
        return None

def expand_node(node):
    global total_expanded_nodes
    expanded_nodes = []
    expanded_nodes.append(Node(move_up(node.state), node, "up", node.depth + 1, 0))
    expanded_nodes.append(Node(move_down(node.state), node, "down", node.depth + 1, 0))
    expanded_nodes.append(Node(move_left(node.state), node, "left", node.depth + 1, 0))
    expanded_nodes.append(Node(move_right(node.state), node, "right", node.depth + 1, 0))
    expanded_nodes = [node for node in expanded_nodes if node.state != None]  
    total_expanded_nodes = total_expanded_nodes + 1
    return expanded_nodes

def bfs(start, goal):
    goal = goal
    start_node = Node(start, None, None, 0, 0)
    fringe = []
    fringe.append(start_node)
    current = fringe.pop(0)
    path = []
    while (current.state != goal):
        fringe.extend(expand_node(current))
        current = fringe.pop(0)
    while (current.parent != None):
        path.insert(0, current.operator)
        current = current.parent
    return path

def dfs(current, goalarr, level=20):  
    if level <= 0:
        return False
    visited.append(current)
    global states_dfs
    states_dfs += 1
    if checkgoal(current, goalarr):
        resultarr.append(current)
        return True
    movelist = move_dfs(current)
    for i in movelist:
        temp = copy.deepcopy(current)
        if 0 <= i[2] < 3 and 0 <= i[3] < 3:
            temp[i[2]][i[3]] = current[i[0]][i[1]]
            temp[i[0]][i[1]] = current[i[2]][i[3]]
        if temp in visited:
            continue
        if dfs(temp, goalarr,level - 1):
            resultarr.append(current)
            return True
    visited.remove(current)
    return False

def ids(current, goalarr):  
    i = 1
    while i <= 35:
        visited.clear()
        if dfs(current, goalarr, i):
            return True
        i += 1
    return False

def uniform_cost(start, goal):
    start_node = Node(start, None, None, 0, 0)
    fringe = []
    path = []
    fringe.append(start_node)
    current = fringe.pop(0)
    while (current.state != goal):
        temp = expand_node(current)
        for item in temp:
            item.depth += current.depth
            fringe.append(item)
        fringe.sort(key=lambda x: x.depth)
        current = fringe.pop(0)
    while (current.parent != None):
        path.insert(0, current.operator)
        current = current.parent
    return path

class State_Node_for_astar:
    goal = [1, 2, 3, 4, 5, 6, 7, 8, 0] 
    greedy_evaluation = None
    AStar_evaluation = None
    heuristic = None
    def __init__(self, state, parent, direction, depth, cost):
        self.state = state
        self.parent = parent
        self.direction = direction
        self.depth = depth
        if parent:
            self.cost = parent.cost + cost
        else:
            self.cost = cost
    def test(self): 
        if self.state == self.goal:
            return True
        return False
    def Manhattan_Distance(self ,n): 
        self.heuristic = 0
        for i in range(1 , n*n):
            distance = abs(self.state.index(i) - self.goal.index(i))
            self.heuristic = self.heuristic + distance/n + distance%n
        self.greedy_evaluation = self.heuristic    
        self.AStar_evaluation = self.heuristic + self.cost
        return( self.greedy_evaluation, self.AStar_evaluation)
    @staticmethod
    def available_moves(x,n): 
        moves = ['Left', 'Right', 'Up', 'Down']
        if x % n == 0:
            moves.remove('Left')
        if x % n == n-1:
            moves.remove('Right')
        if x - n < 0:
            moves.remove('Up')
        if x + n > n*n - 1:
            moves.remove('Down')
        return moves
    def expand(self , n): 
        x = self.state.index(0)
        moves = self.available_moves(x,n)
        children = []
        for direction in moves:
            temp = self.state.copy()
            if direction == 'Left':
                temp[x], temp[x - 1] = temp[x - 1], temp[x]
            elif direction == 'Right':
                temp[x], temp[x + 1] = temp[x + 1], temp[x]
            elif direction == 'Up':
                temp[x], temp[x - n] = temp[x - n], temp[x]
            elif direction == 'Down':
                temp[x], temp[x + n] = temp[x + n], temp[x]
            children.append(State_Node_for_astar(temp, self, direction, self.depth + 1, 1)) 
        return children
    def solution(self):
        solution = []
        solution.append(self.direction)
        path = self
        while path.parent != None:
            path = path.parent
            solution.append(path.direction)
        solution = solution[:-1]
        solution.reverse()
        return solution

def AStar_search(given_state):
    frontier = PriorityQueue()
    explored = []
    counter = 0
    root = State_Node_for_astar(given_state, None, None, 0, 0)
    evaluation = root.Manhattan_Distance(3) 
    frontier.put((evaluation[1], counter, root)) 

    while not frontier.empty():
        current_node = frontier.get()
        current_node = current_node[2]
        explored.append(current_node.state)
        
        if current_node.test():
            return current_node.solution(), len(explored)

        children = current_node.expand(3)
        for child in children:
            if child.state not in explored:
                counter += 1
                evaluation = child.Manhattan_Distance(3) 
                frontier.put((evaluation[1], counter, child)) #based on A* evaluation
    return

def dfs_helper():
    global starting_state, goal_state, visited, states_dfs
    tracemalloc.start()
    time1 = time()
    if dfs(change_the_input_output(starting_state), change_the_input_output(goal_state)):
        print("\nDFS Output: ", end=" ")
        previous_state = change_the_input_output(starting_state)
        for i in reversed(resultarr):
            printpuzzle(i , previous_state)
            previous_state = i
    total_time= time() - time1
    print('\nDFS time: ', total_time)
    print("DFS Memory usage ", tracemalloc.get_traced_memory())
    tracemalloc.stop()

def bfs_helper():
    tracemalloc.start()
    time1 = time()
    result = bfs(starting_state, goal_state)
    if result == None:
        print("No solution found")
    elif result == []:
        print("Start node was the goal!")
    else:
        print("\nBFS output: ", result)
    total_time= time() - time1
    print('BFS time: ', total_time)
    print("BFS Memory usage ", tracemalloc.get_traced_memory())
    tracemalloc.stop()

def ids_helper():
    global starting_state, goal_state, visited, states_dfs
    tracemalloc.start()
    time1 = time()
    if ids(change_the_input_output(starting_state), change_the_input_output(goal_state)):
        print("\nIDS Output: ", end=" ")
        previous_state = change_the_input_output(starting_state)
        for i in reversed(resultarr):
            printpuzzle(i , previous_state)
            previous_state = i
    total_time= time() - time1
    print('\nIDS time: ', total_time)
    print("IDS Memory usage ", tracemalloc.get_traced_memory())
    tracemalloc.stop()

def ucs_helper():
    tracemalloc.start()
    time1 = time()
    result = uniform_cost(starting_state, goal_state)
    if result == None:
        print("No solution found")
    elif result == []:
        print("Start node was the goal!")
    else:
        print("\nUCS output: ",result)
    total_time= time() - time1
    print('UCS time: ', total_time)
    print("UCS Memory usage ", tracemalloc.get_traced_memory())
    tracemalloc.stop()

def a_star_helper():
    print("AStar")
    tracemalloc.start()
    time4 = time()
    AS = AStar_search(starting_state)
    print('A* Solution is ', AS[0])
    astar_time = time() - time4
    print('Astar Time:', astar_time)
    print("A* Memory usage ", tracemalloc.get_traced_memory())
    tracemalloc.stop()

temp =  [ [1,2,3,0,7,6,5,4,8],
 [0,4,1,2,5,3,7,8,6] ,
 [4,1,3,0,2,6,7,5,8] ,
 [1,2,3,0,4,8,7,6,5] ,
 [1,2,0,4,8,3,7,6,5] ,
 [1,0,2,4,6,3,7,5,8] ,
 [0,1,2,4,5,3,7,8,6] ,
 [1,2,3,0,4,5,7,8,6] ,
 [1,2,3,4,0,5,7,8,6] ,
 [1,2,3,4,5,0,7,8,6] ,
 [0,1,3,4,2,5,7,8,6] ,
 [2,3,5,1,0,4,7,8,6] ,
 [1,6,2,5,3,0,4,7,8] ,
 [1,8,2,0,4,3,7,6,5] ,
 [2,5,3,4,1,6,0,7,8],
 [1,2,3,4,6,8,7,5,0] ,
 [1,6,2,5,7,3,0,4,8] ,
 [0,4,1,5,3,2,7,8,6] ,
 [0,5,2,1,8,3,4,7,6] ,
 [1,2,3,0,4,6,7,5,8] ]

for i in temp:
    starting_state = i
    #print("Enter 9 number of the initial grid in seperate line: ")
    #for i in range(9):
    #    temp_input = int(input())
    #    starting_state.append(temp_input)
    print("---------------------------------------------")
    print("Input: ", starting_state)
    print("---------------------------------------------")
    initial_state = starting_state
    visited, states_dfs = [], 0
    resultarr = []
    dfs_helper()
    visited, states_dfs = [], 0
    resultarr = []
    ids_helper()
    bfs_helper()
    ucs_helper()
    visited, states_dfs = [], 0
    resultarr = []
    a_star_helper()






