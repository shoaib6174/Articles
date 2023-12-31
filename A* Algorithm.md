# A* algorithm in short for revision

### Introduction
The A* algorithm is a widely used and efficient graph traversal algorithm that finds the shortest path between two nodes in a weighted graph. It is commonly used in pathfinding and graph search applications, such as route planning in GPS systems and video games.

The A* algorithm combines the benefits of two other popular algorithms, Dijkstra's algorithm and Greedy Best-First Search, by using a heuristic function to guide the search. The algorithm is guaranteed to find the shortest path if certain conditions are met. It intelligently searches through the possible paths, prioritizing the most promising ones first. 


In A* algorithm, the F score will be calculated with help of G score and H score -

F(node) = G(node) + H(node)

G score is the distance from the starting node to that node.
H score is the distance from the end node to that node. 
In this implementation, Ecludian distance will be used.


### **Steps of the Algorithm**

- Initialize open_set (minHeap) and matrices to store G score (gScore), F score (fScore) and predecessor node (cameFrom)

- Calculate G, H and F score of the staring node and add it to open_set 

- While the open_set is not empty: 
  - Select the node with the lowest f-score from open_set.
  - If it's the end node then Reconstruct the path from the goal node to the start node using parent pointers.
  - Remove it from the open_set
  - For each neighbor: 
    - Skip if it is blocked. 
    - Calculate tentative g-score. 
    - If a better g-score is found, update g-score and set parent. 
    - Calculate f-score and add to open_seted if not already there.
    - If it's not in the open_set list add it
- Return None or []


### **Psudocode:**
```
function AStar(start, goal):
    open_set := priority queue with start node
    cameFrom := empty map
    gScore := map with default value of infinity for all nodes
    gScore[start] := 0
    fScore := map with default value of infinity for all nodes
    fScore[start] := heuristic_estimate(start, goal)

    while open_set is not empty:
        current := node in open_set with lowest fScore
        if current == goal:
            path = []
            path.insert(0, current)
            while current in came_from:
                current = came_from[current]
                path.insert(0, current)
            return path
        
        open_set.remove(current)
        for each neighbor of current:
            tentative_gScore := gScore[current] + 1
            
            if tentative_gScore < gScore[neighbor]:
                cameFrom[neighbor] := current
                gScore[neighbor] := tentative_gScore
                fScore[neighbor] := gScore[neighbor] + distance_from_goal(neighbor, goal)
                
                if neighbor not in open_set:
                    open_set.add(neighbor)
    
    return failure (no path found)

function distance_from_goal(node, goal):
    // This function provides an estimated cost from the node to the goal
    // It should be admissible (never overestimate the true cost)
```

**Implementation**
```python
import heapq

def aStarAlgorithm(startRow, startCol, goalRow, goalCol, grid):
    # start and end node
    start = (startRow, startCol)
    goal = (goalRow, goalCol)
    # Define possible movement directions (up, down, left, right, diagonal)
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    # Helper function to calculate the distance_from_goal (Manhattan distance)
    def distance_from_goal(node,goal):
        return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

    open_set = [(0, start)]  # Priority queue (heap) with (f_score, node) tuples
    came_from = {}  # Dictionary to reconstruct the path
    g_score = {(row,col): float('inf') for row in range(len(grid)) for col in range(len(grid[0]))}  # Initialize with infinity
    g_score[start] = 0

    while open_set:
        _, current = heapq.heappop(open_set)
        
        if current == goal:
            # Reconstruct the path from the goal to the start
            path = []
            path.insert(0, current)
            while current in came_from:
                current = came_from[current]
                path.insert(0, current)
            return path

        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)

            if 0 <= neighbor[0] < len(grid) and 0 <= neighbor[1] < len(grid[0]) and grid[neighbor[0]][neighbor[1]] == 0:
                tentative_g_score = g_score[current] + 1
                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + distance_from_goal(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))
    
    return []  # No path found
```
