# Autonomous Line-Following Robot with Dijkstra's Shortest Path Planning

![image](https://github.com/user-attachments/assets/22d16e16-aae8-4662-8503-2299405d7497)

This code is based on the tutorials for Hardware In the Loop Simulations ![link](https://felipenmartins.github.io/Robotics-Simulation-Labs/Lab7/) 

## Overview
This project implements a complete autonomous navigation system featuring:
- Dijkstra's algorithm for optimal path planning
- Webots simulation environment
- ESP32 microcontroller for real-time control
- WiFi communication between components
- Robust line-following with PID control
- Junction detection and handling

This GitHub Repository includes the code that is used in both Thonny IDE (flashed to the ESP32) and on Webots. 
Both codes depend on eachother, as one code will be ran by the ESP which controls the actions of the other code. 

The Jupyter Notebook displays a first attempt at implementing Dijkstra's algorithm using the grid approach, rather than nodes. It allows
for visualization of a custom grid and how a path is planned.


The Relationship can be pictured as the following (image produced by DeepSeek AI):

![deepseek_mermaid_20250604_9c5955](https://github.com/user-attachments/assets/eb3ce27f-02f2-473d-8f8f-be954de869c6)

![deepseek_mermaid_20250604_57aece](https://github.com/user-attachments/assets/4ee71566-1e74-46c1-965e-f2d9ad8fbeb3)



## Mapping
The nodes used by the robot to set the start and end goal, are mapped in this graphic
![image](https://github.com/user-attachments/assets/5fd49e28-e9a7-4522-8e47-a8f4337e9d40)

## To replicate this program, you will need:
  * ESP32
  * MicroPython firmware (≥ 1.19.1) for ESP32
  * Thonny IDE
  * Webots R2024b or later

### ESP Setup Thony IDE

![image](https://github.com/user-attachments/assets/59ec68b6-a462-4416-a4b3-2149837392d4)

### Webots Version Used

![image](https://github.com/user-attachments/assets/1a2ed2b7-3268-4439-bbf9-27dbef28b19a)



## Limitations and Future Improvements:
  * Some parts of the map are not localized properly
  * A* or D* Algorithms would make a faster, more optimized path finding.
  * Obstacle Avoidance is not present
  * Live graphing of the path is to be implemented
