# Advanced-implementation-of-the-A-search-Algorithm-with-Heuristic-Evaluation
This project implements the A* search algorithm *from scratch* and evaluates its performance under different:
- Graph structures (grid, irregular graphs, weighted road networks)
- Heuristic functions (Manhattan, Euclidean, and custom domain-specific)
- Heuristic properties (admissible vs. inadmissible, consistent vs. inconsistent)

The design explicitly separates:

1. *Graph representation* (graph.py)
2. *Search logic (A)** (astar.py)
3. *Heuristic calculation and evaluation* (heuristics.py, profiles.py)

This repository is intended as an advanced academic / learning project, not as a production library.

---

## Features

- Generic weighted graph abstraction
- A* search with:
  - Open/closed sets
  - Configurable heuristic
  - Optional tie-breaking strategies
- Multiple heuristic functions:
  - Manhattan distance
  - Euclidean distance
  - Zero heuristic (Dijkstra)
  - Custom heuristics for irregular grids / road networks
- Performance profiling:
  - Compare runtime and node expansions across heuristics
  - Scaling with graph size and obstacle density
  - Inspect effects of admissible vs. inadmissible heuristic choices
- Simple example environments:
  - 2D grid with obstacles
  - Toy road network

---

## Project structure

```text
src/advanced_astar/
  graph.py        # Weighted graph and grid graph implementations
  astar.py        # Core A* implementation
  heuristics.py   # Heuristic functions and interfaces
  profiles.py     # Performance profiling utilities
  experiments.py  # Scripts to run and log experiments

examples/
  grid_demo.py        # Run A* on a grid and print the path
  road_network_demo.py# Run A* on a toy road network

tests/
  test_astar_basic.py
  test_heuristics.py
  test_consistency.py 
