import heapq, time, copy

food_map = [
    ['S', '.', '.', '#', '.'],
    ['.', '#', '.', '.', '.'],
    ['.', '.', '.', '#', 'C']
]
start = (0, 0)
goal = (2, 4)

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def print_grid(grid, path):
    visual = copy.deepcopy(grid)
    for x, y in path:
        if visual[x][y] not in ['S', 'C']:
            visual[x][y] = 'P'
    for row in visual:
        print(' '.join(str(cell) for cell in row))

# Fungsi untuk menjalankan algoritma GBFS (Greedy Best-First Search)
def gbfs_fd(start, goal, grid):
    queue = [(heuristic(start, goal), start)]
    came_from = {start: None}
    visited = set()
    node_count = 0

    while queue:
        _, current = heapq.heappop(queue)
        node_count += 1
        if current == goal:
            break
        visited.add(current)
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx, ny = current[0]+dx, current[1]+dy
            next_node = (nx, ny)
            if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]) and \
               grid[nx][ny] != '#' and next_node not in visited:
                visited.add(next_node)
                came_from[next_node] = current
                heapq.heappush(queue, (heuristic(next_node, goal), next_node))

    path = []
    node = goal
    while node:
        path.append(node)
        node = came_from.get(node)
    path.reverse()
    return path, node_count

# Fungsi untuk menjalankan A* (A-star)
def a_star_fd(start, goal, grid):
    queue = [(0, start)]
    came_from = {start: None}
    cost_so_far = {start: 0}
    node_count = 0

    while queue:
        _, current = heapq.heappop(queue)
        node_count += 1
        if current == goal:
            break
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx, ny = current[0]+dx, current[1]+dy
            next_node = (nx, ny)
            if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]) and grid[nx][ny] != '#':
                new_cost = cost_so_far[current] + 1
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + heuristic(next_node, goal)
                    heapq.heappush(queue, (priority, next_node))
                    came_from[next_node] = current

    path = []
    node = goal
    while node:
        path.append(node)
        node = came_from.get(node)
    path.reverse()
    return path, node_count

# Perbandingan antara GBFS dan A*
print("[Assignment 3] Food Delivery")
print("Initial Grid:")
print_grid(food_map, [])

start_time = time.time()
gbfs_path, gbfs_nodes = gbfs_fd(start, goal, food_map)
gbfs_time = (time.time() - start_time) * 1000

print("\nGBFS Path:", gbfs_path, f"Time: {gbfs_time:.3f}ms, Nodes: {gbfs_nodes}")
print("GBFS Grid:")
print_grid(food_map, gbfs_path)

start_time = time.time()
astar_path, astar_nodes = a_star_fd(start, goal, food_map)
astar_time = (time.time() - start_time) * 1000

print("\nA* Path:", astar_path, f"Time: {astar_time:.3f}ms, Nodes: {astar_nodes}")
print("A* Grid:")
print_grid(food_map, astar_path)
