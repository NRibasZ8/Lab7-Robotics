from controller import Robot
import serial
import json
import time
import sys
import socket

# Setup Wi-Fi socket communication with ESP32
ESP32_IP = 'PRIVATE '# Replace with the IP shown by your ESP32
ESP32_PORT = 'PRIVATE'

try:
    # Establishes TCP socket connection
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5)
    sock.connect((ESP32_IP, ESP32_PORT))
    sock.settimeout(0.05)  # Set short timeout for non-blocking reads
    print("Connected to ESP32 over Wi-Fi.")
except Exception as e:
    print("Wi-Fi socket connection failed:", e)
    sys.exit(1)

# Gets World timestep
robot = Robot()
timestep = int(robot.getBasicTimeStep())
# Initialize ground sensors
gs = [robot.getDevice(f'gs{i}') for i in range(3)]
for g in gs:
    g.enable(timestep)

# Initialize motors and set to velocity control mode
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Navigation graph: each node has neighbors with (target, distance, heading)
# Headings: 0=up, 1=right, 2=down, 3=left
graph = {
    '1A': [('2C', 21, 0)], 
    '1B': [('2D', 21, 0)], 
    '1C': [('2E', 21, 0)], 
    '1D': [('2F', 21, 0)],
    '2A': [('2B', 49, 1), ('3A', 19, 0)],
    '2B': [('2A', 49, 3), ('3B', 19, 0), ('2C', 13, 1)],
    '2C': [('1A', 21, 2), ('2D', 12, 1), ('2B', 13, 3)],
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
#
class NodeTracker:
    """ Tracks the robot's current node and heading within the navigation graph.

    Attributes:
        graph (dict): The navigation graph with neighbors and heading info.
        current_node (str): The current node where the robot is located.
        heading (int): The current heading (0=up, 1=right, 2=down, 3=left).
    """
    # Tracks the robot's current node and heading within the graph.
    def _init_(self, graph, start_node, start_heading):
        """ Initialize the tracker with a starting node and heading.

        Args:
            graph (dict): The navigation graph.
            start_node (str): The initial node of the robot.
            start_heading (int): The initial heading direction.
        """
        self.graph = graph
        self.current_node = start_node
        self.heading = start_heading
        
    # Given the next node, compute the relative turn needed based on heading.
    def get_turn_to(self, next_node):
        """ Compute the relative turn direction needed to move to the next node.

        Args:
            next_node (str): The destination node.

        Returns:
            tuple: (relative_turn (int), new_heading (int))

        Raises:
            Exception: If there is no path to the next node.
        """
        for neighbor, _, direction in self.graph[self.current_node]:
            if neighbor == next_node:
                rel_dir = (direction - self.heading) % 4
                print(f"Current node: {self.current_node}, Next node: {next_node}, Heading: {self.heading}, "
                      f"Next edge direction: {direction}, Turn needed: {rel_dir}")
                return rel_dir, direction
        raise Exception(f"No path from {self.current_node} to {next_node}")
    
    # Advance to the next node and update heading.
    def advance(self, next_node):
        """ Compute the relative turn direction needed to move to the next node.

        Args:
            next_node (str): The destination node.

        Returns:
            tuple: (relative_turn (int), new_heading (int))

        Raises:
            Exception: If there is no path to the next node.
        """
        rel_dir, abs_dir = self.get_turn_to(next_node)
        print(f"[DEBUG] Advancing from {self.current_node} to {next_node}. Previous heading: {self.heading}, New heading: {abs_dir}")
        self.current_node = next_node
        self.heading = abs_dir
        return rel_dir

# Request a path from the ESP32 server over socket.
def request_path(start, goal):
    """ Request a path from the ESP32 server using TCP socket.

    Args:
        start (str): The starting node.
        goal (str): The goal node.

    Returns:
        list: A list of node names representing the path.

    Side Effects:
        Sends a request over socket, reads response from socket.
    """
    try:
        sock.send((json.dumps({'start': start, 'goal': goal}) + '\n').encode())
    except Exception as e:
        print("Failed to send path request:", e)
        return []

    # Waits for a valid JSON response to path follow
    while True:
        try:
            line = sock.recv(1024).decode('utf-8').strip()
            if line.startswith('{') and line.endswith('}'):
                return json.loads(line).get('path', [])
        except (socket.timeout, UnicodeDecodeError, json.JSONDecodeError):
            continue
        except Exception as e:
            print("Error receiving path:", e)
            break
    return []

# Simple PID controller for line-following adjustments.  
def compute_pid(error):
    """ Compute the PID (Proportional-Integral-Derivative) correction value for a given sensor error.

    This function calculates the PID control output using the current error between the desired
    and actual sensor values. The output is used to adjust motor speeds in order to maintain
    accurate line-following behavior.

    Parameters:
    - error (float): The current error from the center of the line (negative = left, positive = right).

    Returns:
    - float: The correction value to apply to the motors. Positive values steer right, negative steer left.

    Notes:
    - Uses global variables: last_error (for derivative), integral (for accumulated error).
    - This function assumes that the robot is in line-following mode where PID is applicable.
    """
    global pid_integral, pid_last_error
    pid_integral += error
    derivative = error - pid_last_error
    pid_last_error = error
    return Kp * error + Ki * pid_integral + Kd * derivative


# Tuning parameters for movement
TURN_STEPS_90 = 44 # Approximate step count for 90° turn
TURN_STEPS_180 = 2 * TURN_STEPS_90
TURN_DRIVE_OFF = 10 # Drive forward steps after turn to clear junction
TURN_SPEED = 0.25 * 6.28 # Turning wheel velocity
BASE_SPEED = 0.3 * 6.28 # Base line-following velocity
MAX_SPEED = 6.28 # Max wheel velocity

# PID controller constants
Kp = 0.75
Ki = 0.001
Kd = 0.08
pid_integral = 0
pid_last_error = 0

# Override turn distances for tricky junctions/sensor misalignment
TURN_CORRECTIONS = {
    ('N', 'P'): 39,
    ('J', 'L'): 49,
    ('P', 'Q'): 50,
    ('W', 'V'): 49, 
    ('T', 'AA'): 49,
    ('S', 'P'): 44,
    ('M', 'N'): 44,
    ('N', 'L'): 54,
}

# Define robot FSM states
STATE_FOLLOW = 0
STATE_PREPARE_TURN = 1
STATE_TURNING = 2
STATE_REACQUIRE_LINE = 3
STATE_DRIVE_OFF = 4
STATE_FINISHED = 5

# Initialize path planning
START_NODE = 'X' # Input Starting Node
GOAL_NODE = 'O' # Input Final Node
PATH = request_path(START_NODE, GOAL_NODE)
if PATH:
    print("I am going to take this path: " + " -> ".join(PATH))
else:
    print("No path found. Exiting.")
    sys.exit(1)
if not PATH:
    print("No path found. Exiting.")
    sys.exit(1)

# Start at initial path node
path_index = 0
node_tracker = NodeTracker(graph, PATH[0], 2) # Change heading to where robot is facing
# 0=up, 1=right, 2=down, 3=left

# FSM state variables
state = STATE_FOLLOW
turn_counter = 0
turn_direction = 0
turn_steps_target = 0
next_node = None
turn_lost_line = False
junction_cooldown = 0
JUNCTION_COOLDOWN_STEPS = 32
past_junction = False

# -- Main Loop  --
while robot.step(timestep) != -1:
    # Read sensor values (IR ground sensors)
    gsValues = [g.getValue() for g in gs]
    line_left = gsValues[0] > 350
    line_center = gsValues[1] > 350
    line_right = gsValues[2] > 350
    junction_now = sum([line_left, line_center, line_right]) > 1

    # Update junction flag to prevent re-triggering
    if not junction_now:
        past_junction = True

    # -- State: Line Following --
    if state == STATE_FOLLOW:
        # Calculate error from sensor data
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

        correction = compute_pid(error)
        leftMotor.setVelocity(BASE_SPEED - correction)
        rightMotor.setVelocity(BASE_SPEED + correction)

        # Junction detection and path decision
        if junction_cooldown == 0 and junction_now and past_junction and path_index + 1 < len(PATH):
            next_node = PATH[path_index + 1]
            turn_direction, _ = node_tracker.get_turn_to(next_node)
            past_junction = False

            if turn_direction != 0:
                state = STATE_PREPARE_TURN
                turn_counter = 0
                turn_lost_line = False
            else:
                state = STATE_DRIVE_OFF
                turn_counter = 0
    
    # -- State: Short drive before turning --
    elif state == STATE_PREPARE_TURN:
        leftMotor.setVelocity(BASE_SPEED)
        rightMotor.setVelocity(BASE_SPEED)
        turn_counter += 1
        if turn_counter >= 4:
           state = STATE_TURNING
           turn_counter = 0
           # Use special correction if available, else default turn steps
           turn_steps_target = TURN_CORRECTIONS.get(
               (node_tracker.current_node, next_node),
                TURN_STEPS_90 if turn_direction in (1, 3) else TURN_STEPS_180
            )
    
    # -- State: Turning; left, right or 180
    elif state == STATE_TURNING:
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
            state = STATE_REACQUIRE_LINE
            turn_counter = 0
            turn_lost_line = False

    # -- State: Drive until line is redetected --
    elif state == STATE_REACQUIRE_LINE:
        leftMotor.setVelocity(0.5 * MAX_SPEED)
        rightMotor.setVelocity(0.5 * MAX_SPEED)
        if line_center:
            state = STATE_DRIVE_OFF
            turn_counter = 0

    # -- State: Short Drive after turn to clear junction --
    elif state == STATE_DRIVE_OFF:
        leftMotor.setVelocity(BASE_SPEED)
        rightMotor.setVelocity(BASE_SPEED)
        turn_counter += 1
        if turn_counter >= TURN_DRIVE_OFF and not junction_now:
            # Confirm node change and update path
            node_tracker.advance(next_node)
            path_index += 1
            junction_cooldown = JUNCTION_COOLDOWN_STEPS
            past_junction = False
            state = STATE_FINISHED if path_index == len(PATH) - 1 else STATE_FOLLOW
            turn_counter = 0

    elif state == STATE_FINISHED:
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        break

    if junction_cooldown > 0:
        junction_cooldown -= 1

sock.close()
