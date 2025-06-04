import network
import socket
from machine import Pin
from time import sleep
import ujson

# --- Wi-Fi Setup ---
SSID = "Matasiphone" # Change with your own Wi-fi ID
PASSWORD = "lol12344" # Change with your own Wi-fi password

print("Connecting to Wi-Fi...")
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(SSID, PASSWORD)

# Wait until connected
while not wlan.isconnected():
    sleep(0.5)
print("Connected to Wi-Fi. IP:", wlan.ifconfig()[0])

# --- Socket Server Setup ---
HOST = ''  # All interfaces
PORT = 8888

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((HOST, PORT))
server.listen(1)
server.settimeout(None)

print("Waiting for PC connection...")
client, addr = server.accept()
print("Connected by", addr)

client.settimeout(0.1)  # Non-blocking read

# --- Init State ---
current_state = 'forward'
counter = 0
COUNTER_MAX = 5
state_updated = True

# Debounce for line sensors
def debounce_line(prev, new, count, threshold=3):
    if new == prev:
        count += 1
    else:
        count = 0
    return new if count >= threshold else prev, count

line_left = False
line_center = False
line_right = False
dl, dc, dr = 0, 0, 0

# Graph for Dijkstra
graph = {
    'A': {'E': 21},
    'B': {'F': 21},
    'C': {'G': 21},
    'D': {'H': 21},
    'E': {'A': 21, 'J': 26, 'F': 12},
    'F': {'E': 12, 'B': 21, 'G': 12},
    'G': {'F': 12, 'C': 21, 'H': 12},
    'H': {'G': 12, 'D': 21, 'O': 34},
    'I': {'K': 19, 'J': 49},
    'J': {'I': 49, 'L': 19, 'E': 34},
    'K': {'M': 13, 'L': 72, 'I': 19},
    'L': {'N': 13, 'J': 19, 'K': 49},
    'M': {'W': 29, 'N': 49, 'K': 13},
    'N': {'P': 12, 'M': 49, 'L': 13, 'O': 49},
    'O': {'Q': 13, 'N': 49, 'H': 34},
    'P': {'S': 13, 'Q': 49, 'N': 12},
    'Q': {'R': 19, 'O': 8, 'P': 49},
    'R': {'S': 49, 'Q': 19},
    'S': {'T': 13, 'P': 13, 'R': 49},
    'T': {'U': 12, 'AA': 12, 'S': 12},
    'U': {'V': 12, 'Z': 21, 'T': 12},
    'V': {'W': 12, 'Y': 12, 'U': 12},
    'W': {'X': 12, 'V': 12, 'M': 3},
    'X': {'W': 12},
    'Y': {'V': 12},
    'Z': {'U': 21},
    'AA': {'T': 12},
}

def dijkstra(graph, start, goal):
    shortest_distance = {node: float('inf') for node in graph}
    shortest_distance[start] = 0
    predecessor = {}
    unseen_nodes = dict(graph)

    while unseen_nodes:
        min_node = min(unseen_nodes, key=shortest_distance.get)
        if shortest_distance[min_node] == float('inf'):
            break

        for neighbour, weight in graph[min_node].items():
            if weight + shortest_distance[min_node] < shortest_distance[neighbour]:
                shortest_distance[neighbour] = weight + shortest_distance[min_node]
                predecessor[neighbour] = min_node
        unseen_nodes.pop(min_node)

    path = []
    node = goal
    while node != start:
        if node not in predecessor:
            return []
        path.insert(0, node)
        node = predecessor[node]
    path.insert(0, start)
    return path

# --- Main Loop ---
buffer = b""

while True:
    try:
        data = client.recv(1024)
        if data:
            buffer += data
            while b'\n' in buffer:
                line, buffer = buffer.split(b'\n', 1)
                msg_str = line.decode('utf-8').strip()

                # Handle path request
                if msg_str.startswith('{') and msg_str.endswith('}'):
                    try:
                        command = ujson.loads(msg_str)
                        start = command['start']
                        goal = command['goal']
                        path = dijkstra(graph, start, goal)
                        print("Dijkstra from", start, "to", goal, ":", path)
                        client.send((ujson.dumps({'path': path}) + '\n').encode())
                    except Exception as e:
                        client.send((ujson.dumps({'error': str(e)}) + '\n').encode())

                # Handle sensor input (e.g., "010")
                elif len(msg_str) == 3:
                    raw_left = (msg_str[0] == '0')
                    raw_center = (msg_str[1] == '0')
                    raw_right = (msg_str[2] == '0')
                    line_left, dl = debounce_line(line_left, raw_left, dl)
                    line_center, dc = debounce_line(line_center, raw_center, dc)
                    line_right, dr = debounce_line(line_right, raw_right, dr)

    except Exception:
        pass  # Socket timeout or no data

    # FSM Logic
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