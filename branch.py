//code
waypoints = [] 
with open("C:\\Users\\moham\\Downloads\\hack\\waypoints.txt") as f: 
    for line in f: 
        id, x, y, z = map(float, line.strip().split()) 
        waypoints.append((int(id), x, y, z))
import math 
N = len(waypoints) 
dist = [[0.0] * N for _ in range(N)] 
for i in range(N): 
    for j in range(N): 
        _, x1, y1, z1 = waypoints[i] 
        _, x2, y2, z2 = waypoints[j] 
        dist[i][j] = math.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)
import math
import heapq

# Step 1: Read waypoints.txt and parse data
def read_waypoints(filename):
    waypoints = []
    with open(filename, 'r') as f:
        for line in f:
            id, x, y, z = map(float, line.strip().split())
            waypoints.append((int(id), x, y, z))
    return waypoints

# Step 2: Compute Euclidean distance matrix
def compute_distance_matrix(waypoints):
    N = len(waypoints)
    dist = [[0.0] * N for _ in range(N)]
    for i in range(N):
        _, x1, y1, z1 = waypoints[i]
        for j in range(N):
            _, x2, y2, z2 = waypoints[j]
            dist[i][j] = math.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)
    return dist

# Node class for priority queue
class Node:
    def __init__(self, path, level, bound, visited):
        self.path = path
        self.level = level
        self.bound = bound
        self.visited = visited

    def __lt__(self, other):
        return self.bound < other.bound

# Calculate lower bound (approximate minimum additional cost)
def calculate_bound(path, visited, dist, N):
    bound = 0
    for i in range(N):
        if not visited[i]:
            min_edge = float('inf')
            for j in range(N):
                if i != j and not visited[j]:
                    min_edge = min(min_edge, dist[i][j])
            bound += 0 if min_edge == float('inf') else min_edge
    return bound

# Branch and Bound TSP solver
def branch_and_bound_tsp(waypoints, dist):
    N = len(waypoints)
    best_cost = float('inf')
    best_path = []

    pq = []
    visited = [False] * N
    visited[0] = True
    initial_path = [0]
    initial_bound = calculate_bound(initial_path, visited, dist, N)
    heapq.heappush(pq, Node(initial_path, 1, initial_bound, visited[:]))

    while pq:
        current = heapq.heappop(pq)

        if current.level == N:
            cost = sum(dist[current.path[i]][current.path[i+1]] for i in range(N - 1))
            cost += dist[current.path[-1]][current.path[0]]  # Return to start
            if cost < best_cost:
                best_cost = cost
                best_path = current.path[:]
        else:
            for i in range(N):
                if not current.visited[i]:
                    new_path = current.path + [i]
                    new_visited = current.visited[:]
                    new_visited[i] = True
                    bound = calculate_bound(new_path, new_visited, dist, N)
                    if bound < best_cost:
                        heapq.heappush(pq, Node(new_path, current.level + 1, bound, new_visited))

    best_path.append(best_path[0])  # Return to start
    return best_path, best_cost

# Step 5: Write result to path.txt
def write_output(filename, path, cost, waypoints):
    with open(filename, 'w') as f:
        ids = [waypoints[i][0] for i in path]
        f.write(" ".join(map(str, ids)) + f" {cost:.2f}\n")

# Main entry point
def main():
    waypoints = read_waypoints('waypoints.txt')
    dist = compute_distance_matrix(waypoints)
    path, cost = branch_and_bound_tsp(waypoints, dist)
    write_output('path.txt', path, cost, waypoints)

if __name__ == '__main__':
    main()
        