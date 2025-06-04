"""
EPUCK controller script with Wi-Fi communication, path planning, and PID line following.
"""
from controller import Robot
import serial
import json
import socket
import time
import sys

# Wi-Fi communication setup
ESP32_IP = 'XXXX'  # Replace with IP displayed in Thonny after the connection
ESP32_PORT = 'XXXX' # Replace with Port USed

# Makes the TCP connection
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5)
    sock.connect((ESP32_IP, ESP32_PORT))
    sock.settimeout(0.05) 
    print("ESP32 connected via Wi-Fi")
except Exception as e:
    print("Wi-Fi connection failed:", e)
    sys.exit(1)

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Ground/line tracking sensor initialisation
gs = [robot.getDevice(f'gs{i}') for i in range(3)]
for g in gs:
    g.enable(timestep)

# Motor (wheel) initialisation
rightMotor = robot.getDevice('right wheel motor')
leftMotor = robot.getDevice('left wheel motor')
rightMotor.setPosition(float('inf'))
leftMotor.setPosition(float('inf'))
rightMotor.setVelocity(0.0)
leftMotor.setVelocity(0.0)

# Node graph: all nodes have neighbouring nodes with (Node name, Distance, Direction)
# Directions: 0 = North, 1 = East, 2 = South, 3 = West
graph = {
    '1A': [('2C', 21, 0)], 
    '1B': [('2D', 21, 0)], 
    '1C': [('2E', 21, 0)], 
    '1D': [('2F', 21, 0)],
    '2A': [('2B', 49, 1), ('3A', 19, 0)],
    '2B': [('2A', 49, 3), ('3B', 19, 0), ('2C', 50, 1)],
    '2C': [('1A', 21, 2), ('2D', 12, 1), ('2B', 50, 3)],
    '2D': [('2C', 12, 3), ('1B', 21, 2), ('2E', 12, 1)],
    '2E': [('2D', 12, 3), ('1C', 21, 2), ('2F', 12, 1)],
    '2F': [('2E', 12, 3), ('1D', 21, 2), ('4C', 34, 1)],
    '3A': [('2A', 19, 2), ('4A', 13, 0), ('3B', 72, 1)],
    '3B': [('3A', 72, 3), ('2B', 19, 2), ('4B', 13, 0)],
    '4A': [('3A', 13, 2), ('4B', 49, 1), ('6A', 34, 0)],
    '4B': [('3B', 13, 2), ('4A', 49, 3), ('5A', 12, 0), ('4C', 49, 1)],
    '4C': [('2F', 34, 2), ('4B', 49, 3), ('5B', 13, 0)],
    '5A': [('4B', 12, 2), ('5B', 49, 1), ('6E', 13, 0)],
    '5B': [('5A', 49, 3), ('4C', 8, 2), ('6F', 19, 0)],
    '6A': [('4A', 34, 2), ('6B', 12, 1), ('7A', 12, 0)],
    '6B': [('6C', 12, 1), ('7B', 12, 0), ('6A', 12, 3)],
    '6C': [('6D', 12, 1), ('7C', 21, 0), ('6B', 12, 3)],
    '6D': [('6E', 12, 1), ('7D', 12, 0), ('6C', 12, 3)],
    '6E': [('5A', 19, 2), ('6F', 49, 1), ('6D', 13, 3)],
    '6F': [('5B', 19, 2), ('6E', 49, 3)],
    '7A': [('6A', 12, 2)],
    '7B': [('6B', 12, 2)],
    '7C': [('6C', 12, 2)],
    '7D': [('6D', 12, 2)]
}

# Keeps track of the EPUCK's current position as a node from the graph
class TrackNode:
    """
    Tracks the current node and direction of the e-puck within the node graph,
    providing methods to determine turn directions and update position.
    """

    def __init__(self, graph, start_node, start_direction):
        """
        Initialize the TrackNode with a given graph, starting node, and direction.

        Args:
            graph (dict): Adjacency list of nodes with neighbour information.
            start_node (str): The node where the robot begins.
            start_direction (int): The robot's initial direction (0 = North, 1 = East, 2 = South, 3 = West).
        """
        self.graph = graph
        self.current_node = start_node
        self.direction = start_direction
        
    def get_turn_to(self, next_node):
        """
        Determine the relative turn needed and the absolute direction to reach the next node.

        Args:
            next_node (str): The target neighbouring node.

        Returns:
            tuple:
                relative_direction (int): Relative turn direction (0 = straight, 1 = right, 2 = about-face, 3 = left).
                absolute_direction (int): Absolute direction from current node to next node (0-3).

        Raises:
            Exception: If there is no direct edge from the current node to next_node.
        """
        for neighbour, _, direction in self.graph[self.current_node]:
            if neighbour == next_node:
                relative_direction = (direction - self.direction) % 4
                print(f"Current node: {self.current_node}, Next planned node: {next_node}, direction: {self.direction}")
                return relative_direction, direction
        raise Exception(f"No path found from {self.current_node} to {next_node}")
    
    def advance(self, next_node):
        """
        Update the robot's current node and direction after moving to the next node.

        Args:
            next_node (str): The node to which the robot is advancing.

        Returns:
            int: The relative turn direction used to get to the next node.
        """
        relative_direction, absolute_direction = self.get_turn_to(next_node)
        print(f"Moving from {self.current_node} to {next_node}. New direction: {absolute_direction}")
        self.current_node = next_node
        self.direction = absolute_direction
        return relative_direction

def get_path(start, goal):
    """
    Request a path from the ESP32 over a TCP socket, sending start and goal node names.

    Args:
        start (str): Starting node name.
        goal (str): Goal node name.

    Returns:
        list: A list of node names representing the route, or an empty list if failed.
    """
    try:
        sock.send((json.dumps({'start': start, 'goal': goal}) + '\n').encode())
    except Exception as e:
        print("Failed to request path:", e)
        return []

    # Waits for valid response as JSON.
    while True:
        try:
            line = sock.recv(1024).decode('utf-8').strip()
            if line.startswith('{') and line.endswith('}'):
                return json.loads(line).get('path', [])
        except (socket.timeout, UnicodeDecodeError, json.JSONDecodeError):
            continue
        except Exception as e:
            print("Error getting path:", e)
            break
    return []

def pid_calc(error):
    """
    Compute the PID correction for line following based on sensor error.

    Uses global variables integral and prev_err to maintain state.

    Args:
        error (float): The error signal derived from line sensor readings.

    Returns:
        float: The correction value to apply to motor velocities.
    """
    global integral, prev_err
    integral += error
    derivative = error - prev_err
    prev_err = error
    return Kp * error + Ki * integral + Kd * derivative

# Parameters for movement
TURN_90 = 44  # Step count for a 90° turn
TURN_180 = 88  # Step count for a 180° turn
TURN_INTERSECTION = 10  # Steps to exit a intersection
MAX_SPEED = 6.28  # Max wheel velocity (2π radians)
TURN_SPEED = 0.25 * MAX_SPEED  # Angular velocity while turning
BASE_SPEED = 0.3 * MAX_SPEED  # Angular velocity while following a line

# PID constants
Kp = 0.75
Ki = 0.001
Kd = 0.08
integral = 0
prev_err = 0

# Override turn distances for tricky intersections/sensor misalignment
ADJUSTED_TURNS = {
    ('4B', '5A'): 39,
    ('2B', '3B'): 49,
    ('5A', '5B'): 50,
    ('6A', '6B'): 49,
    ('6D', '7D'): 49,
    ('6E', '5A'): 44,
    ('4A', '4B'): 44,
    ('4B', '3B'): 54,
}

# Robot state machine states
FOLLOW_LINE = 0
TURN_PREP = 1
IN_TURN = 2
SEARCH = 3
INTERSECTION = 4
COMPLETED = 5

# Set starting and goal nodes
START_NODE = '7A'  # Start node/initial position
GOAL_NODE = '3B'  # Robot's goal node

# Get path
PATH = get_path(START_NODE, GOAL_NODE)
if PATH:
    print("Following the path: " + " -> ".join(PATH))
else:
    print("No path found. Quitting.")
    sys.exit(1)
if not PATH:
    print("No path found. Quitting.")
    sys.exit(1)

# Start at initial path node
path_index = 0
node_tracking = TrackNode(graph, PATH[0], 2)  # Change direction to where EPUCK is facing by default

# State machine variables
state = FOLLOW_LINE
turn_counter = 0
turn_direction = 0
turn_steps_target = 0
next_node = None
turn_lost_line = False
intersection_cooldown = 0
intersection_cooldown_steps = 32
past_intersection = False

# Main Loop
while robot.step(timestep) != -1:
    # Get ground sensor values
    gsValues = [g.getValue() for g in gs]
    line_left = gsValues[0] > 350
    line_center = gsValues[1] > 350
    line_right = gsValues[2] > 350
    intersection_now = sum([line_left, line_center, line_right]) > 1

    # Update intersection marker to prevent re-triggering
    if not intersection_now:
        past_intersection = True

    # State: line Following
    if state == FOLLOW_LINE:
        if line_left and not line_center and not line_right:
            error = -1
        elif line_right and not line_center and not line_left:
            error = 1
        elif line_center and not line_left and not line_right:
            error = 0
        elif line_left and line_center and not line_right:
            error = -0.5
        elif line_right and line_center and not line_left:
            error = 0.5
        else:
            error = 0  
        correction = pid_calc(error)
        leftMotor.setVelocity(BASE_SPEED - correction)
        rightMotor.setVelocity(BASE_SPEED + correction)

        # Intersection detection and path deciding
        if intersection_cooldown == 0 and intersection_now and past_intersection and path_index + 1 < len(PATH):
            next_node = PATH[path_index + 1]
            turn_direction, _ = node_tracking.get_turn_to(next_node)
            past_intersection = False
            if turn_direction != 0:
                state = TURN_PREP
                turn_counter = 0
                turn_lost_line = False
            else:
                state = INTERSECTION
                turn_counter = 0
    
    # State: prep for making a turn (drives forward slightly)
    elif state == TURN_PREP:
        leftMotor.setVelocity(BASE_SPEED)
        rightMotor.setVelocity(BASE_SPEED)
        turn_counter += 1
        if turn_counter >= 4:
           state = IN_TURN
           turn_counter = 0
           # Use turn correction if required
           turn_steps_target = ADJUSTED_TURNS.get(
               (node_tracking.current_node, next_node),
                TURN_90 if turn_direction in (1, 3) else TURN_180
            )
    
    # State: 90 or 180 degree turn
    elif state == IN_TURN:
        if turn_direction == 1:
            leftMotor.setVelocity(TURN_SPEED)
            rightMotor.setVelocity(-TURN_SPEED)
        elif turn_direction == 3:
            leftMotor.setVelocity(-TURN_SPEED)
            rightMotor.setVelocity(TURN_SPEED)
        elif turn_direction == 2:
            leftMotor.setVelocity(TURN_SPEED)
            rightMotor.setVelocity(-TURN_SPEED)

        turn_counter += 1
        if not (line_left or line_center or line_right):
            turn_lost_line = True
        if (turn_lost_line and line_center) or (turn_counter >= turn_steps_target):
            state = SEARCH
            turn_counter = 0
            turn_lost_line = False

    # State: searching for line if lost
    elif state == SEARCH:
        leftMotor.setVelocity(0.5 * MAX_SPEED)
        rightMotor.setVelocity(0.5 * MAX_SPEED)
        if line_center:
            state = INTERSECTION
            turn_counter = 0

    # State: short drive after intersection
    elif state == INTERSECTION:
        leftMotor.setVelocity(BASE_SPEED)
        rightMotor.setVelocity(BASE_SPEED)
        turn_counter += 1
        if turn_counter >= TURN_INTERSECTION and not intersection_now:  # Confirm node has changed and update the path
            node_tracking.advance(next_node)
            path_index += 1
            intersection_cooldown = intersection_cooldown_steps
            past_intersection = False
            state = COMPLETED if path_index == len(PATH) - 1 else FOLLOW_LINE
            turn_counter = 0

    # State: stop EPUCK if final node is reached
    elif state == COMPLETED:
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        break

    if intersection_cooldown > 0:
        intersection_cooldown -= 1

sock.close()
