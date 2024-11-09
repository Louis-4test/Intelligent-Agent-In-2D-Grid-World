# Intelligent Agent in 2D Grid World

This project implements an intelligent agent navigating a 2D grid world using the A* search algorithm. The agent starts from a given position and finds the shortest path to a target position, avoiding obstacles placed in the grid.

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Requirements](#requirements)
- [Usage](#usage)
- [Code Explanation](#code-explanation)

## Overview

The script simulates a grid world where an agent navigates from a starting position to a target position while avoiding obstacles. The agent uses the A* search algorithm to find the shortest path to the target based on a heuristic (Manhattan distance).

## Features

- Generates a 2D grid with obstacles and a target.
- Implements an `Agent` class to represent the moving agent.
- Uses the A* algorithm for pathfinding, ensuring optimal pathfinding based on Manhattan distance.
- Outputs each move of the agent and the complete path to the target if found.

## Requirements

The script requires Python 3 with the following libraries:
- `numpy`: For handling the grid structure
- `heapq`: For managing the priority queue in the A* algorithm (part of Python standard library)

To install `numpy`, use:
```bash
pip install numpy
