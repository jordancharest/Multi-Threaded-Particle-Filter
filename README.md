# Multi-threaded Particle Filter
Particle Filter Localization in serial and parallel on a simulated robot with performance analysis. The project was largely developed in a separate, private repo.

[//]: # (Image References)
[initial]: ./images/initial_state.png "Initial State"

## Implementation
The algorithm implementation is described in detail [here](./multi-threaded-particle-filter.pdf). 

In short, the simulation begins by initializing a square world (e.g. WORLD_SIZE = 512), and randomly initializing WORLD_SIZE * 2 particles (e.g. 1024), a robot, and 12 landmarks. The world appears planar in the visualization, but is toroidal in the simulation (i.e. if a particle/robot drives of the right boundary, it reappears at the left boundary; if it drives off the top boundary, it reappears at the bottom boundary and vice versa). Visually, the initial state may appear like this:

![alt_text][initial]


## Compile and Run
```
g++ -O3 -std=c++14 -pthread -o main.exe main.cpp robot.cpp
./main.exe <num_threads> <random-seed>
```

## Execution
