import numpy as np
import matplotlib.pyplot as plt
import cv2
import heapq
from matplotlib.animation import FuncAnimation

# Generating map
map = np.zeros((int(3000), int(6000), 3))
red = np.array([255, 0, 0])
black = np.array([0, 0, 0])

# Generating circular obstacles 
center_circle = (int(1120), int(2420))   
radius_circle = int(400)
cv2.circle(map, center_circle, radius_circle, (255, 0, 0), -1)

center_circle = (int(2630), int(900))   
radius_circle = int(700)
cv2.circle(map, center_circle, radius_circle, (255, 0, 0), -1)

center_circle = (int(4450), int(2200))   
radius_circle = int(375)
cv2.circle(map, center_circle, radius_circle, (255, 0, 0), -1)

# Display the image
plt.imshow(map.astype(int))
plt.gca().invert_yaxis()
plt.title("Initial Map 'red = obstacle'\n\n" r"$\bf{(close\ this\ window\ to\ continue)}$")
plt.pause(2) 
plt.close()  

# Define the actions set
actions_set = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)]
cost_straight = 1.0
cost_diagonal = 1.4

# Define the function to calculate the cost of moving from one node to another
def calculate_cost(current_cost, action, step_size=1):
    if action in [(1, 1), (-1, 1), (1, -1), (-1, -1)]:
        return current_cost + cost_diagonal
    else:
        return current_cost + cost_straight

# Define the heuristic function (Euclidean distance)
def heuristic(node, goal):
    return np.sqrt((goal[0] - node[0]) ** 2 + (goal[1] - node[1]) ** 2)

# Define colors
red = np.array([255, 0, 0])

# Actual Bidirectional A* algorithm implementation
def bidirectional_astar(start, goal, map, step_size=10):
    # Initialize the open and closed sets
    open_set_start = [Node(start, None, 0, heuristic(start, goal))]
    open_set_goal = [Node(goal, None, 0, heuristic(goal, start))]
    closed_set_start = set()
    closed_set_goal = set()

    # Initialize the path and cost dictionaries
    path_start = {}
    path_goal = {}
    cost_start = {start: 0}
    cost_goal = {goal: 0}

    path_iteration = 0

    # Run the bidirectional A* algorithm
    while open_set_start and open_set_goal:
        # Get the node with the lowest cost from the start open set
        current_node_start = heapq.heappop(open_set_start)
        closed_set_start.add(current_node_start.position)
        
        # Get the node with the lowest cost from the goal open set
        current_node_goal = heapq.heappop(open_set_goal)
        closed_set_goal.add(current_node_goal.position)

        # Plot the current nodes being explored in green
        print("current start node: ", current_node_start.position, "current goal node: ", current_node_goal.position)

        if 0 <= current_node_start.position[0] < map.shape[0] and 0 <= current_node_start.position[1] < map.shape[1]:
            map[current_node_start.position[0], current_node_start.position[1]] = [112, 79, 0]
        if 0 <= current_node_goal.position[0] < map.shape[0] and 0 <= current_node_goal.position[1] < map.shape[1]:
            map[current_node_goal.position[0], current_node_goal.position[1]] = [45, 22, 22]
        path_iteration += 1
        if path_iteration % 3000 == 0:
            plt.imshow(map.astype(int))
            plt.gca().invert_yaxis()
            plt.title(f'Scanned map after {path_iteration} iterations')
            plt.pause(0.001)

        # Check if the current nodes are in each other's closed sets
        if current_node_start.position in closed_set_goal:
            # Path found, set the intersecting node and break the loop
            intersecting_node = current_node_start.position
            break
        if current_node_goal.position in closed_set_start:
            # Path found, set the intersecting node and break the loop
            intersecting_node = current_node_goal.position
            break

        # Expand the neighbors of the current nodes
        neighbors_start = get_neighbors(current_node_start.position, map, step_size)
        neighbors_goal = get_neighbors(current_node_goal.position, map, step_size)


        for neighbor in neighbors_start:
            # Calculate the cost of moving to the neighbor
            cost = calculate_cost(current_node_start.cost, neighbor)

            # Check if the neighbor is already in the closed set
            if neighbor in closed_set_start:
                continue

            # Check if the neighbor is in the open set
            neighbor_node = None
            for node in open_set_start:
                if node.position == neighbor:
                    neighbor_node = node
                    break

            if neighbor_node is None:
                # Add the neighbor to the open set
                neighbor_node = Node(neighbor, current_node_start, cost, heuristic(neighbor, goal))
                heapq.heappush(open_set_start, neighbor_node)
            elif cost < neighbor_node.cost:
                # Update the cost of the neighbor
                neighbor_node.cost = cost
                neighbor_node.parent = current_node_start

            # Update the path and cost variables
            path_start[neighbor] = current_node_start
            cost_start[neighbor] = cost

        for neighbor in neighbors_goal:
            # Calculate the cost of moving to the neighbor
            cost = calculate_cost(current_node_goal.cost, neighbor)

            # Check if the neighbor is already in the closed set
            if neighbor in closed_set_goal:
                continue

            # Check if the neighbor is in the open set
            neighbor_node = None
            for node in open_set_goal:
                if node.position == neighbor:
                    neighbor_node = node
                    break

            if neighbor_node is None:
                # Add the neighbor to the open set
                neighbor_node = Node(neighbor, current_node_goal, cost, heuristic(neighbor, start))
                heapq.heappush(open_set_goal, neighbor_node)
            elif cost < neighbor_node.cost:
                # Update the cost of the neighbor
                neighbor_node.cost = cost
                neighbor_node.parent = current_node_goal

            # Update the path and cost variables
            path_goal[neighbor] = current_node_goal
            cost_goal[neighbor] = cost

    # If no path found, return None
    if 'intersecting_node' not in locals():
        return None

    # Reconstruct the two paths
    start_path = []
    goal_path = []

    # Reconstruct the start path
    node = path_start.get(intersecting_node)
    while node:
        start_path.insert(0, node.position)
        node = path_start.get(node.position)

    # Reconstruct the goal path
    node = path_goal.get(intersecting_node)
    while node:
        goal_path.append(node.position)
        node = path_goal.get(node.position)

    goal_path.reverse()

    # Return the two paths
    return start_path, goal_path

# Define the Node class
class Node:
    def __init__(self, position, parent, cost, heuristic):
        self.position = position
        self.parent = parent
        self.cost = cost
        self.heuristic = heuristic
        self.total_cost = cost + heuristic

    def __lt__(self, other):
        return self.total_cost < other.total_cost

# Define the function to get the valid neighbors of a node
def get_neighbors(position, map, step_size):
    neighbors = []
    for action in actions_set:
        neighbor = (position[0] + action[0]* step_size, position[1] + action[1]* step_size)
        if is_valid(neighbor, map):
            neighbors.append(neighbor)
    return neighbors

# Define the function to check if a position is valid
def is_valid(position, map):
    if position[0] < 0 or position[0] >= map.shape[0] or position[1] < 0 or position[1] >= map.shape[1]:
        return False
    if np.array_equal(map[position[0], position[1]], red):
        return False
    
    return True

# Define the heuristic function (Euclidean distance)
def heuristic(position, goal):
    return np.sqrt((position[0] - goal[0])**2 + (position[1] - goal[1])**2)

# Define the cost function (Manhattan distance)
def calculate_cost(current_cost, neighbor):
    return current_cost + 1

# Define the start and goal positions
start_x = int(0)
start_y = int(1500)
goal_x = int(6000)
goal_y = int(1500)

# Run Bidirectional A_star's algorithm
start_node = (start_y, start_x) 
goal_node = (goal_y, goal_x)
start_path, goal_path = bidirectional_astar(start_node, goal_node, map)

# Print the path
path_iteration = 0
if start_path:
    print("Path found:", start_path)

    # Draw circles for initial and goal positions
    cv2.circle(map, (start_x, start_y), 7, (255, 255, 0), -1)  # Green circle for initial position
    cv2.circle(map, (goal_x, goal_y), 7, (255, 0, 0), -1)   # Red circle for goal position

    # Create arrays of start and goal points
    start_points = np.array(start_path)
    goal_points = np.array(goal_path)

    # Define update function for animation
    def update_plot(frame):
        plt.clf()
        plt.imshow(map.astype(int))
        plt.gca().invert_yaxis()
        plt.title(f'Generating Path - Frame {frame}')
        current_position = frame * 100  # Adjust this value based on animation speed
        plt.plot(start_points[:current_position, 1], start_points[:current_position, 0], color='yellow', label='Path from Start')
        plt.plot(goal_points[:current_position, 1], goal_points[:current_position, 0], color='green', label='Path from Goal')
        plt.legend()

    # Create animation
    fig, ax = plt.subplots()
    animation = FuncAnimation(fig, update_plot, frames=len(start_points) // 10, interval=200)  # Adjust interval for animation speed
    plt.show()

else:
    print("No path found.")

# Display the updated image after scanning
plt.imshow(map.astype(int))
plt.gca().invert_yaxis()
plt.title(f'Map after scanning')
plt.show()
cv2.destroyAllWindows()