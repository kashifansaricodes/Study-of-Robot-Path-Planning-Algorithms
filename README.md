# Robot Path Planning Algorithms

<p align="center">
  <img src="https://github.com/user-attachments/assets/f7094637-3b46-4d4f-a040-403a173a9703" width="45%">
  <img src="https://github.com/user-attachments/assets/6c38987b-a759-40cc-95f8-a1450a06747d" width="45%">
</p>



## Overview
This repository combines multiple path planning algorithms implemented in Python. The algorithms include:

- **Dijkstra's Algorithm**
- **A* Algorithm**
- **Bi-Directional A* Algorithm**
- **Potential Field Path Planning**

Each algorithm is designed to navigate a robot from a start point to a goal point while avoiding obstacles in a 2D environment. The implementations use visualization tools to display the path planning process.

---
## Repository Structure

### **Files and Directories**

```
|-- Djikstra.py           # Implementation of Dijkstra's Algorithm
|-- a_star.py             # Implementation of A* Algorithm
|-- potential_field.py    # Implementation of Potential Field Path Planning Algorithm
|-- Initial_map.png       # Initial environment visualization
|-- Scanned_map.png       # Final scanned map with planned path
|-- Video.mp4             # Demonstration of the algorithms in action
|-- README.md             # Documentation for running the project
```

---
## **Requirements**

The following Python libraries are required to run the scripts:

- numpy
- matplotlib
- opencv-python (cv2)
- heapq
- time (for A* algorithm)

To install the required dependencies, run:

```sh
pip install numpy matplotlib opencv-python
```

---
## **Usage Instructions**

### **Dijkstra's Algorithm**
1. Run `Djikstra.py` using any Python IDE (e.g., VS Code, Spyder, or Colab).
2. Close the initial map window when prompted.
3. Enter the start coordinates.
4. Enter the goal coordinates.
5. The algorithm computes the shortest path, which is displayed on the scanned map.

---
### **A* Algorithm**
1. Run `a_star.py` using a Python IDE.
2. Close the initial map window to proceed.
3. Input the step size.
4. Input the clearance (displayed in blue on the map).
5. Input the robot radius (this value is added to the clearance).
6. Enter the start coordinates.
7. Enter the goal coordinates.
8. Wait for the computed path to be backtracked.
9. The final scanned map displays the generated path.

---
### **Potential Field Path Planning**

This algorithm uses attractive and repulsive forces to navigate a robot in a 2D environment.

#### **Setup**
1. Run `potential_field.py`.
2. Adjust the environment parameters as needed.

#### **Algorithm Steps:**
- Defines map dimensions and obstacles.
- Computes repulsive forces from obstacles and attractive forces towards the goal.
- Updates the robot’s position iteratively until it reaches the goal.
- Visualizes the trajectory using Matplotlib’s `FuncAnimation`.

---
## **Customization**
You can modify the following parameters in the scripts:
- **Map dimensions** (height, width)
- **Obstacle positions and sizes**
- **Path planning parameters** (step size, robot radius, clearance, attraction/repulsion strengths)

---
## **Visualization and Results**
Each algorithm generates a visualization of the planned path:
- `Initial_map.png` shows the obstacle space before planning.
- `Scanned_map.png` shows the computed path.
- `Video.mp4` contains a demonstration of the algorithms in action.


---
## **License**
This project is open-source and free to use for educational and research purposes.

