# Multi-threaded Particle Filter
Particle Filter Localization in serial and parallel on a simulated robot with performance analysis. Initial tests demonstrate a ~2x-5x speedup, dependent on the number of threads and the number of particles used in the simulation.

[//]: # (Image References)
[initial]: ./images/initial_state.png "Initial State"
[during]: ./images/during.png "Partially Converged"
[convergence]: ./images/convergence.png "Converged"
[performance]: ./images/performance.png "Performance"
[speedup]: ./images/parallelization.png "Speedup"

## Background & Implementation
The algorithm implementation is described in detail [here](./multi-threaded-particle-filter.pdf). 

In short, the simulation begins by initializing a square world (e.g. WORLD_SIZE = 512), and randomly initializing WORLD_SIZE * 2 particles (e.g. 1024), a robot, and 12 landmarks. The world appears planar in the visualization, but is toroidal in the simulation (i.e. if a particle/robot drives of the right boundary, it reappears at the left boundary; if it drives off the top boundary, it reappears at the bottom boundary and vice versa). Visually, the initial state may appear as it does below:

![alt_text][initial]

In this visualization, the black triangles represent known landmarks, the large red circle near the top right corner represents the robot’s actual location, and the small blue dots scattered throughout are the particles, or the robot’s guesses as to its true location.
Note that the map size, particle quantity, and landmark quantity are reduced in this visualization to reduce visual clutter.

The robot then begins to localize itself, performing a motion update, sensor update, importance weight calculation, resampling, and mean error evaluation, in a loop until the solution converges. The particles begin to converge to the robot's relative location fairly quickly. Below is a visualization of the level of convergence of the particles after two updates. Note that this visualization is entirely independent of the initial state visualization above. The robot was randomly initialized to the top left corner this time and is shown taking two steps to the right (the previous robot locations remain in the visualization to show the robot path).

![alt_text][during]

After two motion updates, the particles have converged to the general area of the robot. Upon close inspection, it becomes clear that each particle location is actually a very small distribution. This is due to the motion noise that is necessary for convergence. The number of particles remains the same as the initial quantity, but their poses are close enough to visually overlap.

The particle filter continues to run until the mean error drops below an abritrary allowable limit and the solution is declared as converged. A full run may look like the visualization below. Note that this visualization is a continuation of the above image. In this example, the robot starts around (20, 75) and starts moving to the right, passes through the top boundary, reappearing on the bottom, then passes through the right boundary, reappearing on the left. The solution finally converges when the robot is slightly above where it originally started. Note that the size of the robot steps are exaggerated for visual clarity. In reality, the solution would be able to converge in significantly less distance.

![alt_text][convergence]


## Compile and Run
```
g++ -O3 -std=c++14 -pthread -o main.exe main.cpp robot.cpp
./main.exe <num-threads> <random-seed>
```

## Execution
The visualizations shown above are stills from an animation generated with another program I wrote, [particle_filter.py](https://github.com/jordancharest/AI-for-Robotics/blob/master/Particle-Filters), not the high performance program in this repo. This program simply displays runtime statistics to STDOUT in the following format:
```
Parallel Particle Filter - 8 threads
  time  |   Mean Error     |  Loop Time
     0  |       7.277264   |  0.016956
     1  |       5.226717   |  0.001158
     2  |       4.684458   |  0.001185
     3  |       5.583659   |  0.005599
     4  |       5.918658   |  0.008603
     5  |       6.446126   |  0.001407
     6  |       5.206860   |  0.001337
     7  |       4.829508   |  0.001116
     8  |       4.351076   |  0.001100
     9  |       4.267200   |  0.001198
    10  |       3.993746   |  0.001123
    11  |       4.128606   |  0.001130
    12  |       4.201962   |  0.001101
    13  |       4.685445   |  0.001145
    14  |       4.741052   |  0.001259
    15  |       4.697053   |  0.001166
    16  |       4.569939   |  0.001218
    17  |       4.569769   |  0.001145
    18  |       4.560523   |  0.001401
    19  |       4.774743   |  0.001172
    20  |       4.348195   |  0.001161
    21  |       4.477184   |  0.005474
    22  |       4.463998   |  0.001132
    23  |       4.327607   |  0.001130
    24  |       4.106002   |  0.001143
    25  |       3.943834   |  0.001162
    26  |       4.036755   |  0.005327
    27  |       3.955813   |  0.001106
    28  |       3.826012   |  0.001160
    29  |       3.782999   |  0.001120
    30  |       4.785907   |  0.001206
    31  |       2.146478   |  0.001792
Solution Converged. Average update time: 0.002326

```
with similar results shown for a serial run. Due to the stochastic nature of the algorithm, the number of steps to convergence is variable.

## Results
Initial results were promising, with parallel runs showing significant improvements over the serial run, with speedups of ~2x-5x, dependent on the number of threads, number of particles, and of course the machine. My personal laptop, built for single-thread performance, shows little to no improvement while purpose-built parallel machines demonstrate significant speedups.

Execution times of different thread configurations as a function of number of particles is shown below. Note that the execution time denotes the time for a single update step, not time to convergence, which is variable.

![alt_text][performance]


The speedups relative to the serial run, for each thread configuration, across different particle quantities is shown below.

![alt_text][speedup]
