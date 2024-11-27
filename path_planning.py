import heapq
grid_size = 8 # feet
obstacles = [
       [(3,3), (5,3)],
       [(5,3), (5,5)],
       [(5,5), (3,5)],
       [(3,5), (3,3)],
       [(0,0), (8,0)],
       [(8,0), (8,8)],
       [(8,8), (0,8)],
       [(0,0), (0,8)]
 ]

resolution = 0.5
start = (6.5,1.5)
end = (1.5, 6.5)
robot_width = 0.2

min_heap = []
visited = {}
ancestors = {}
import math


def distance_from_point_to_line(x1, y1, x2, y2, x3, y3):
    A = y2 - y1
    B = x1 - x2
    C = x2 * y1 - x1 * y2
    
    distance = abs(A * x3 + B * y3 + C) / math.sqrt(A**2 + B**2)
    return distance

def is_point_on_line(line, point, radius):
    x1, y1 = line[0]
    x2, y2 = line[1]
    x3, y3 = point

    line_length_sq = (x2 - x1) ** 2 + (y2 - y1) ** 2

    if line_length_sq == 0:
        return math.sqrt((x3 - x1) ** 2 + (y3 - y1) ** 2) <= radius

    t = max(0, min(1, ((x3 - x1) * (x2 - x1) + (y3 - y1) * (y2 - y1)) / line_length_sq))
    px = x1 + t * (x2 - x1)
    py = y1 + t * (y2 - y1)

    distance = math.sqrt((px - x3) ** 2 + (py - y3) ** 2)

    return distance <= radius

def check_collision(obstacles, point):
    for line in obstacles:
        if is_point_on_line(line, point, robot_width):
            return True
        
    return False

def snap_to_grid(point, resolution):
    x, y = point
    return round(x / resolution) * resolution, round(y / resolution) * resolution

def get_neighbors(node, resolution):
    x, y = node
    directions = [
        (0, resolution),  
        (0, -resolution),
        (resolution, 0),  
        (-resolution, 0),  
        (resolution, -resolution),
        (-resolution, resolution),
        (resolution, resolution),
        (-resolution, -resolution)
    ]
    return [(x + dx, y + dy) for dx, dy in directions]

heapq.heappush(min_heap, (0.0, start))
visited[start] = 1

while(True):
    curr_cost, curr_node = heapq.heappop(min_heap)

    if curr_node == end:
        break

    neighbors = [snap_to_grid(neighbor, resolution) for neighbor in get_neighbors(curr_node, resolution)]

    for neighbor in neighbors:
        collision = check_collision(obstacles, neighbor)

        if collision or neighbor in visited:
            continue

        visited[neighbor] = 1
        
        new_cost = curr_cost + math.sqrt( math.pow((curr_node[0] - neighbor[0]),2) + math.pow((curr_node[1] - neighbor[1]),2))
        heapq.heappush(min_heap, (new_cost, neighbor))
        ancestors[neighbor] = curr_node


path = []
curr = end
while(curr != start):
    parent = ancestors[curr]
    old_parent = parent
    curr = tuple(round(num, 4) for num in curr)
    parent = tuple(round(num, 4) for num in parent)

    if round(parent[1] + resolution,4) == curr[1] and parent[0] == curr[0]:
        path.append((curr[0], curr[1], math.pi/2))
    elif parent[1] == curr[1] and round(parent[0] - resolution,4) == curr[0]:
        path.append((curr[0], curr[1], math.pi))
    elif round(parent[1] + resolution,4) == curr[1] and round(parent[0] - resolution,4) == curr[0]:
        path.append((curr[0], curr[1], 3 * math.pi/4))
    else:
        print(curr)
        print(f"{parent[0] - resolution} {parent[1] + resolution}")
        print(parent)

    
    curr = old_parent
feet_to_meters = 0.3048


data_in_meters = [(x * feet_to_meters, y * feet_to_meters, theta) for x, y, theta in path]
# print(data_in_meters)

import matplotlib.pyplot as plt
import numpy as np


fig, ax = plt.subplots()

for x, y, theta in path:
    ax.plot(x, y, 'bo')  
    
    dx = np.cos(theta)  
    dy = np.sin(theta)  
    ax.arrow(x, y, dx * 0.5, dy * 0.5, head_width=0.2, head_length=0.3, fc='r', ec='r') 

square_x = [3, 5, 5, 3, 3]
square_y = [3, 3, 5, 5, 3]
ax.plot(square_x, square_y, linestyle='--', color='g', label='Square (3,3 to 5,5)')

ax.set_xlim(0, 8)
ax.set_ylim(0, 8)
ax.set_aspect('equal')
ax.grid(True)


plt.show()

