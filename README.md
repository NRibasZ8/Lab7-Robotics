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

It uses a non-numpy version of Dijkstra's Algorithm (as shown in the jupyter Notebook), as micropython (used for running the code in the ESP32) does not support NumPy.

## Mapping
The nodes used by the robot to set the start and end goal, are mapped in this graphic
![image](https://github.com/user-attachments/assets/5fd49e28-e9a7-4522-8e47-a8f4337e9d40)

## To replicate this program, you will need:
  * ESP32
  * MicroPython firmware (≥ 1.19.1) for ESP32
  * Thonny IDE
  * Webots R2024b or later

## Limitations and Future Improvements:
  Some parts of the map are not localized properly
  A* or D* Algorithms would make a faster, more optimized path finding. 
