# Multi-threaded Particle Filter
Particle Filter Localization in serial and parallel on a simulated robot with performance analysis
## Compile and Run
g++ -O3 -std=c++14 -pthread -o main.exe main.cpp robot.cpp

./main.exe <num_threads> <random-seed>
