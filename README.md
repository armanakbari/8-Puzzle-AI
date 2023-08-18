# 8-Puzzle

This was an assignment from my Artificial Intelligence course at university.

The 8-puzzle game is a sliding puzzle that consists of a 3x3 grid with 8 numbered tiles and
one blank space. The goal is to rearrange the tiles from a start state to a goal state by
sliding the tiles into the blank space.

The following are the components for solving the 8-puzzle game with search algorithms:
  1. States: The state of the puzzle at a given moment, represented by an arrangement of the tiles on the grid.
  2. Actions: The possible moves of the blank space, which are either up, down, left, or right.
  3. Goal Test: A function that determines if the current state is the goal state.
  4. Path Cost: A value assigned to each step in the search, representing the cost of getting from one state to another. In the 8-puzzle game, the path cost is usually
  assigned a value of 1 for each move.
  5. Transition Model: A function that takes the current state and an action, and returns the next state after that action is executed

## The Manhattan distance heuristic:
  The Manhattan distance is the sum of the distances of each tile from its goal position, measured along the rows and columns. It is an admissible and consistent heuristic, which means it never overestimates the true distance and satisfies the triangle inequality. This makes it well-suited for guiding the search towards the goal.

  1. Breadth-First Search (BFS): BFS is guaranteed to find the optimal solution in the least number of moves, but its time and space complexity can be very high, as it needs to keep track of all possible states at every depth level.
  2. Depth-First Search (DFS): DFS has low time and space complexity, but it may not find the optimal solution and can get stuck in an infinite loop if the depth limit is not set properly.
  3. Iterative Deepening Search (IDS): IDS combines the best of DFS and BFS by gradually increasing the depth limit to ensure optimality while avoiding the infinite loop
  problem. However, it still has a high time complexity, especially for deep states.
  4. Uniform Cost Search (UCS): UCS is similar to BFS in terms of finding the optimal solution, but it prioritizes states based on their cost rather than their depth level. It is useful for solving problems with non-uniform step costs.
  5. A* Search: A* search uses a heuristic function to guide the search and find the optimal solution in a more efficient manner than UCS. The heuristic should be
  admissible and consistent, meaning it never overestimates the true distance to the goal and satisfies the triangle inequality. The best heuristic function for the 8-puzzle problem is the Manhattan distance.

In conclusion, A* search is generally considered the best algorithm for the 8-puzzle problem, as it finds the optimal solution efficiently while ensuring optimality. The average time for A* was approximately 0.0011 which was way better than other search algorithms. More results containing the time and space of these algorithms are shown below. 

I used Tracemalloc to report the max memory usage for each of the search algorithms. Tracemalloc is a library module that traces every memory block in Python. The tracing starts by using the start() during runtime. This library module can also give information about the total size, number, and average size of allocated memory blocks. The output is given in the form of (current, peak), i.e., current memory is the memory the code is currently using, and peak memory is the maximum space the program used while executing.


## ⭐Bonus part: 
  The best algorithm for solving this problem is A*. I also took a look at IDA* which is Iterative Deepening A*. It is a variant of the A* algorithm that addresses the
  the problem of the high memory usage associated with A*. IDA* uses a depth-first search approach and gradually increases the depth limit while using a heuristic function to guide the search. In general, A* is considered to be a more efficient and flexible algorithm than IDA*. However, IDA* may be preferred in situations where memory usage is a concern, or where the heuristic function satisfies the conditions for monotonicity. Ultimately, the choice between the two algorithms depends on the specific problem and the requirements of the application.
