# Bidirectional A* Algorithm in Python

This repository contains an implementation of the Bidirectional A* algorithm in Python. Bidirectional A* is a pathfinding algorithm that simultaneously searches from the start and goal nodes, meeting in the middle to find the optimal path more efficiently than unidirectional A*.

## Features
- **Bidirectional Search:** Searches from both the start and goal nodes simultaneously.
- **Efficient Pathfinding:** Optimizes search by reducing the number of nodes explored.
- **Separate Path Capture:** Captures and returns paths from both start and goal nodes when they meet.
- **Visualization:** Includes visualization of the search process and the final path using Matplotlib and OpenCV.

## Requirements
- Python 3.8 or higher
- NumPy
- Matplotlib
- OpenCV

## Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/kashifansaricodes/Bi-Directional-A-Star.git
   ```

2. **Install the required packages:**
   ```bash
   pip install -r requirements.txt
   ```

## Usage

1. **Run the main script:**
   ```bash
   python bidirectional_astar.py
   ```

## How It Works

1. **Initialization:**
   - The algorithm initializes two priority queues, one for the forward search from the start node and one for the backward search from the goal node.
   
2. **Search Process:**
   - The algorithm alternates between expanding nodes from the forward and backward searches.
   - Nodes expanded from one search are checked against the nodes expanded from the other search to find a meeting point.

3. **Path Reconstruction:**
   - Once the searches meet, the paths from the start to the meeting point and from the goal to the meeting point are combined to form the final path.
   - The paths from both searches are captured separately and can be visualized.

4. **Visualization:**
   - The search process and final path can be visualized using Matplotlib and OpenCV for better understanding and debugging.

--------------------------------------------------------------------------------