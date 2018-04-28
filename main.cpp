#include <iostream>
#include <iomanip>
#include <vector>
#include <thread>
#include <atomic>
#include <ctime>
#include <chrono>

#include "robot.hpp"

// GLOBAL - for access by all threads
static int N = 2048;           // number of particles
static int world_size = N/2;   // toroidal world
static int L = 12;              // number of landmarks
static const double SENSOR_NOISE = 3.0;
static const double MOVE_NOISE = 0.08;
static const double ALLOWABLE = 0.75 * SENSOR_NOISE;
std::atomic<double> max_weight(-1.0);
std::atomic<double> p_mean_error(100.0);

static double forward_cmd;
static double turn_cmd;
static Robot parallel_robot(world_size);

std::vector<Point> landmarks(L);
static std::vector<Robot> particles(N);
static std::vector<Robot> resampled_particles(N);
static std::vector<double> measurements(L);

// random generator for particle initialization
std::default_random_engine generator;
std::uniform_real_distribution<double> uniform(0.0, 1.0);
std::normal_distribution<double> gaussian(0.0, SENSOR_NOISE);
std::normal_distribution<double> gaussian_move(0.0, MOVE_NOISE);

// thread barrier for synchronization (C construct)
pthread_barrier_t barrier;


// EVALUATE ======================================================================================
/* Evaluate the mean error between all the particles and the robot using Euclidean distance     */
void parallel_evaluate(const auto &robot, int particle_index, int particles_per_thread) {
    double sum = 0.0;
    double dx, dy;

    for (int i = particle_index; i < particle_index + particles_per_thread; i++) {
        dx = modulo((particles[i].x() - robot.x() + (world_size/2.0)), world_size) - (world_size/2.0);
        dy = modulo((particles[i].y() - robot.y() + (world_size/2.0)), world_size) - (world_size/2.0);
        sum += sqrt(dx*dx + dy*dy);
    }

    p_mean_error = p_mean_error + sum / N;
}


// EVALUATE ======================================================================================
/* Evaluate the mean error between all the particles and the robot using Euclidean distance     */
void evaluate(double &mean_error, const auto &robot) {
    double sum = 0.0;
    double dx, dy;

    for (auto &particle : particles) {
        dx = modulo((particle.x() - robot.x() + (world_size/2.0)), world_size) - (world_size/2.0);
        dy = modulo((particle.y() - robot.y() + (world_size/2.0)), world_size) - (world_size/2.0);
        sum += sqrt(dx*dx + dy*dy);
    }

    mean_error = sum / N;
}



// PARALLEL PARTICLE FILTER ======================================================================
/* Implements the particle filter in parallel                                                   */
void parallel_particle_filter(int num_threads, int t_index) {

    int particles_per_thread = N / num_threads;
    int particle_index = N / num_threads * t_index;

    using namespace std::chrono;
    high_resolution_clock::time_point start;
    double total_time = 0.0;

    int t = 0;

    // simulate the robot moving about its environment until the solution converges
    while (p_mean_error > ALLOWABLE) {

        // have the main thread generate a movement command
        if (t_index == 0) {
            max_weight = -1.0;
            start = high_resolution_clock::now();

            // define a random movement
            forward_cmd = 1 + uniform(generator) * 5;
            if (uniform(generator) > 0.5)
                turn_cmd = uniform(generator) * 0.3;    // radians
            else
                turn_cmd = - uniform(generator) * 0.3;  // radians
        }

        // BARRIER: don't attempt to move the robot until a new command is generated
        pthread_barrier_wait(&barrier);

        // move the robot, perform landmark measurements, and reset the mean error
        if (t_index == 0) {
            p_mean_error = 0.0;
            parallel_robot.move(forward_cmd, turn_cmd);
            parallel_robot.sense(measurements);
        }

        // simulate the same motion update for all particles
        for (int i = particle_index; i < particle_index + particles_per_thread; i++)
            particles[i].move(forward_cmd, turn_cmd);

        // calculate importance weights for all particles based on their locations, calculate the max along the way
        double p_max_weight = -1.0;
        for (int i = particle_index; i < particle_index + particles_per_thread; i++) {
            particles[i].measurement_prob(measurements);
            p_max_weight = std::max(p_max_weight, particles[i].weight());
        }

        // update global max
        if (p_max_weight > max_weight)
            max_weight = p_max_weight;


        // BARRIER: importance weights must be calculated before starting resampling
        pthread_barrier_wait(&barrier);

        // resample using the Resampling Wheel Algorithm
        double beta = 0.0;
        int index = uniform(generator) * N;

        for (int i = particle_index; i < particle_index + particles_per_thread; i++) {
            beta += uniform(generator) * 2.0 * max_weight;

            while (beta > particles[index].weight()) {
                beta -= particles[index].weight();
                index = (index + 1) % N;
            }

            resampled_particles[i] = particles[index];
        }

        // BARRIER: reassign particles vector to the resampled particles
        pthread_barrier_wait(&barrier);

        if (t_index == 0)
            particles.swap(resampled_particles);

        // BARRIER: don't evaluate mean error until the pointers are swapped
        pthread_barrier_wait(&barrier);

        parallel_evaluate(parallel_robot, particle_index, particles_per_thread);

        // BARRIER: ensure every thread gets the same mean error so they all exit together
        pthread_barrier_wait(&barrier);

        // log update statistics
        if (t_index == 0) {
            high_resolution_clock::time_point finish = high_resolution_clock::now();
            duration<double> span = duration_cast<duration<double> >(finish-start);
            total_time += span.count();
            std::cout << std::setw(6) << t << "  |     " << std::setw(10) << std::fixed << p_mean_error << "   |  " << span.count() << "\n";
        }

        t++;
    }

    if (t_index == 0)
        std::cout << "Solution Converged. Average update time: " << total_time/t << std::endl;
}


// PARALLEL INIT =================================================================================
void parallel(int num_threads) {

    using namespace std::chrono;

    std::cout << "\nParallel Particle Filter - " << num_threads << " threads\n";
    std::cout << std::setw(6) << "time" << "  |  " << std::setw(12) << " Mean Error " << "  |  "
              << std::setw(10) << "Loop Time\n";


    std::vector<std::thread> threads(num_threads);
    for (int i = 1; i < num_threads; i++) {
        threads[i] = std::thread(parallel_particle_filter, num_threads, i);
    }

    parallel_particle_filter(num_threads, 0);


    for (int i = 1; i < num_threads; i++)
        threads[i].join();
}


// SERIAL PARTICLE FILTER ========================================================================
double serial_particle_filter(Robot &robot) {

    using namespace std::chrono;
    double total_time = 0.0;

    std::cout << "\nSerial Particle Filter\n";
    std::cout << std::setw(6) << "time" << "  |  " << std::setw(14) << " Mean Error " << "  |  "
              << std::setw(10) << "Loop Time\n";

    int t = 0;
    std::vector<double> loc;
    double mean_error = 100.0;

    // simulate the robot moving about its environment until the solution converges
    while (mean_error > ALLOWABLE) {
        high_resolution_clock::time_point start = high_resolution_clock::now();

        // generate a random movement
        forward_cmd = 1 + uniform(generator) * 5;
        if (uniform(generator) > 0.5)
            turn_cmd = uniform(generator) * 0.3;    // radians
        else
            turn_cmd = - uniform(generator) * 0.3;  // radians


        // move the robot
        robot.move(forward_cmd, turn_cmd);

        // simulate the same motion update for all particles
        for (auto &particle : particles)
            particle.move(forward_cmd, turn_cmd);

        // take sensor measurements to all the landmarks
        robot.sense(measurements);

        // calculate importance weights for all particles based on their locations, calculate the max on the way
        double max_weight = -1.0;
        for (auto &particle : particles) {
            particle.measurement_prob(measurements);
            max_weight = std::max(max_weight, particle.weight());
        }

        // resample using the Resampling Wheel Algorithm
        double beta = 0.0;
        int index = uniform(generator) * N;
        std::vector<Robot> new_particles(particles.size());

        for (int i = 0; i < N; i++) {
            beta += uniform(generator) * 2.0 * max_weight;

            while (beta > particles[index].weight()) {
                beta -= particles[index].weight();
                index = (index + 1) % N;
            }

            new_particles[i] = particles[index];
        }

        particles.swap(new_particles);

        // evaluate the mean error
        evaluate(mean_error, robot);

        // log the update statistics
        high_resolution_clock::time_point finish = high_resolution_clock::now();
        duration<double> span = duration_cast<duration<double> >(finish-start);
        total_time += span.count();
        std::cout << std::setw(6) << t << "  |     " << std::setw(10) << std::fixed << mean_error << "   |  " << span.count() << "\n";

        t++;
    }

    std::cout << "Solution Converged. Average update time: " << total_time/t << std::endl;

    return total_time/t;
}



// INIT ==========================================================================================
void init(Robot &robot) {

    // Initialize L random landmarks
    Point landmark;
    for (int i = 0; i < L; i++) {
        landmark.x = uniform(generator) * world_size;
        landmark.y = uniform(generator) * world_size;
        landmarks[i] = landmark;
    }

    // Define noise levels on robot
    robot.setNoise(MOVE_NOISE, MOVE_NOISE, SENSOR_NOISE);

    // Initialize N random particles - particle locations are randomly generated in the constructor
    for (int i = 0; i < N; i++) {
        Robot particle(world_size);
        particle.setNoise(MOVE_NOISE, MOVE_NOISE, SENSOR_NOISE);
        particles[i] = particle;
    }
}


// MAIN ==========================================================================================
int main(int argc, char** argv) {

    if (argc != 4) {
        std::cerr << "ERROR: invalid argument(s)\n";
        std::cerr << "USAGE: " << argv[0] << " <num-threads> <random-seed> <num-particles>\n";
        exit(EXIT_FAILURE);
    }

    int num_threads = atoi(argv[1]);
    if (num_threads < 2 || num_threads > 64) {
        std::cerr << "ERROR: invalid argument(s)\n";
        std::cerr << "Number of threads must be between 2 and 32 inclusive\n";
        exit(EXIT_FAILURE);

    }

	pthread_barrier_init(&barrier, NULL, num_threads);

    // seed for random particle generation
    int seed = atoi(argv[2]);

    // number of particles
    N = atoi(argv[3]);
    if (N % 64 != 0) {
        std::cerr << "ERROR: invalid particle count\n";
        std::cerr << "Make divisible by 64 for best results\n";
        exit(EXIT_FAILURE);
    }


    // Initialize the robot to a random location and define noise levels; start the serial simulation
    generator.seed(seed);
    Robot serial_robot(world_size);
    init(serial_robot);
    serial_particle_filter(serial_robot);

    // reset the world to the exact same configuration and rerun in parallel
    generator.seed(seed);
    Robot temp(world_size);
    parallel_robot = temp;
    init(parallel_robot);
    parallel(num_threads);


    return EXIT_SUCCESS;
}
