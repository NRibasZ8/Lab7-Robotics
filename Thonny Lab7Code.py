import network
import socket
from machine import Pin
from time import sleep
import ujson

# Wi-Fi communication setup
SSID = "Matasiphone" # Replace with own SSID
PASSWORD = "lol12344" # Replace with own Wi-Fi password
print("Connecting to Wi-Fi")
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
if wlan.isconnected() == False:
    wlan.connect(SSID, PASSWORD)
while not wlan.isconnected():
    sleep(0.5)
print("Wi-Fi connection succesful. IP:", wlan.ifconfig()[0])

# Socket setup
HOST = ''  # All interfaces
PORT = 5555 # Can be anything, but must match Webots
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((HOST, PORT))
server.listen(1)
server.settimeout(None)

print("Waiting for Webots connection")
client, addr = server.accept()
print("Connected to", addr)
client.settimeout(0.1)

# Initial state
current_state = 'forward'
counter = 0
COUNTER_MAX = 5
state_updated = True

# Debounce ground sensors
def debounce_line(prev, new, count, threshold=3):
    if new == prev:
        count += 1
    else:
        count = 0
    return new if count >= threshold else prev, count

line_left = False
line_center = False
line_right = False
dl, dc, dr = 0, 0, 0 # dl - left; dc - center; dr - right

# Graph for Dijkstra's algorythm calculations
graph = {
    '1A': {'2C': 21},
    '1B': {'2D': 21},
    '1C': {'2E': 21},
    '1D': {'2F': 21},
    '2A': {'3A': 19, '2B': 49},
    '2B': {'2A': 49, '3B': 19, '2C': 26},
    '2C': {'1A': 21, '2B': 26, '2D': 12},
    '2D': {'2C': 12, '1B': 21, '2E': 12},
    '2E': {'2D': 12, '1C': 21, '2F': 12},
    '2F': {'2E': 12, '1D': 21, '4C': 34},
    '3A': {'4A': 13, '3B': 72, '2A': 19},
    '3B': {'4B': 13, '2B': 19, '3A': 49},
    '4A': {'6A': 29, '4B': 49, '3A': 13},
    '4B': {'5A': 12, '4A': 49, '3B': 13, '4C': 49},
    '4C': {'5B': 13, '4B': 49, '2F': 34},
    '5A': {'6E': 13, '5B': 49, '4B': 12},
    '5B': {'6F': 19, '4C': 8, '5A': 49},
    '6A': {'7A': 12, '6B': 12, '4A': 3},
    '6B': {'6A': 12, '7B': 12, '6C': 12},
    '6C': {'6B': 12, '7C': 21, '6D': 12},
    '6D': {'6C': 12, '7D': 12, '6E': 12},
    '6E': {'6D': 13, '5A': 13, '6F': 49},
    '6F': {'6E': 49, '5B': 19},
    '7A': {'6A': 12},
    '7B': {'6B': 12},
    '7C': {'6C': 21},
    '7D': {'6D': 12}
}

# Dijkstras
def dijkstra(graph, start, goal):
    # Initialise a dict holding the shortest distance from the start to every node
    shortest_distance = {node: float('inf') for node in graph}
    shortest_distance[start] = 0
    predecessor = {} # Will map each node to the node from which it was reached, later used to reconstruct the path
    unseen_nodes = dict(graph) # Copy of the grap's keys to represent nodes that haven't been finalized
    # Main loop, continuing as long as there are unfinalized nodes
    while unseen_nodes:
        min_node = min(unseen_nodes, key=shortest_distance.get)  # Select the unseen node with the smallest known distance from 'start'
        if shortest_distance[min_node] == float('inf'): 
            break
        # Compute the distance from 'start' to 'neighbour' via 'min_node'
        for neighbour, weight in graph[min_node].items():
            if weight + shortest_distance[min_node] < shortest_distance[neighbour]:
                shortest_distance[neighbour] = weight + shortest_distance[min_node]
                predecessor[neighbour] = min_node
        unseen_nodes.pop(min_node) # Remove 'min_node' from unseen_nodes, since its shortest distance is now finalized.
    
    # Reconstruct the path from 'start' to 'goal'
    path = []
    node = goal
    while node != start:
        if node not in predecessor:
            return []
        path.insert(0, node)
        node = predecessor[node]
    path.insert(0, start)
    return path

# Main Loop
buffer = b"" # Accumulate raw bytes coming in over the socket

while True:
    try:
        data = client.recv(1024) # Read up to 1024 bytos from Webots socket
        if data:
            buffer += data # Append new data to the buffer
            while b'\n' in buffer:
                # Split out the first line (up to the newline)
                line, buffer = buffer.split(b'\n', 1)
                msg_str = line.decode('utf-8').strip()

                # Handle path request if it's a JSON object
                if msg_str.startswith('{') and msg_str.endswith('}'):
                    try:
                        command = ujson.loads(msg_str)
                        start = command['start']
                        goal = command['goal']
                        path = dijkstra(graph, start, goal)
                        print("Dijkstra from", start, "to", goal, ":", path)
                        # Send back the computed path as a JSON string
                        client.send((ujson.dumps({'path': path}) + '\n').encode())
                    except Exception as e:
                        client.send((ujson.dumps({'error': str(e)}) + '\n').encode())

                # If the message is exactly three characters long (e.g., "010"), interpret that as raw line-sensor readings:
                elif len(msg_str) == 3:
                    # 0 reads as black, 1 reads as white (no line)
                    raw_left = (msg_str[0] == '0')
                    raw_center = (msg_str[1] == '0')
                    raw_right = (msg_str[2] == '0')
                    line_left, dl = debounce_line(line_left, raw_left, dl)
                    line_center, dc = debounce_line(line_center, raw_center, dc)
                    line_right, dr = debounce_line(line_right, raw_right, dr)

    except Exception:
        pass  # Socket timeout or no data

    # State machine logic
    if current_state == 'forward':
        counter = 0
        if line_right and not line_left:
            current_state = 'turn_right'
            state_updated = True
        elif line_left and not line_right:
            current_state = 'turn_left'
            state_updated = True
        elif not line_left and not line_center and not line_right:
            current_state = 'search'
            state_updated = True

    elif current_state == 'turn_right':
        if counter >= COUNTER_MAX:
            current_state = 'forward'
            state_updated = True

    elif current_state == 'turn_left':
        if counter >= COUNTER_MAX:
            current_state = 'forward'
            state_updated = True

    elif current_state == 'search':
        if line_center or line_left or line_right:
            current_state = 'forward'
            state_updated = True

    elif current_state == 'stop':
        current_state = 'forward'
        state_updated = True

    if state_updated:
        try:
            client.send((current_state + '\n').encode())
        except:
            pass
        state_updated = False

    counter += 1
    sleep(0.02)
