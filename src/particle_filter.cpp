#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <string>
#include <cmath>

struct Particle {
    double x, y, theta;
    double weight;
};

class ParticleFilter {
public:
    ParticleFilter(int num_particles) : gen{rd()}, num_particles(num_particles), parameters{} {
        particles.resize(num_particles);
    }

    // 1. Initialization    
    void init(double mean_x, double mean_y, double mean_theta, const std::vector<double>& std_vec) {
        // Gaussian distribution with mean and std
        std::normal_distribution<double> x_dist{mean_x, std_vec[0]};
        std::normal_distribution<double> y_dist{mean_y, std_vec[1]};
        std::normal_distribution<double> theta_dist{mean_theta, std_vec[2]};

        for (auto& p : particles) {
            p.x = x_dist(gen);
            p.y = y_dist(gen);
            p.theta = theta_dist(gen);
            p.weight = 1.0;
        }
    }

    // Helper function to generate motion based noise
    double motionNoise(double v, double w, const std::vector<double> p) {
        return sqrt(p[0] * v * v + p[1] * w * w);
    } 

    // 2. Motion update (predict)
    void motionUpdate(double delta_t, double velocity, double yaw) {
        std::normal_distribution<double> x_noise_dist{0, motionNoise(velocity, yaw, parameters)};
        std::normal_distribution<double> y_noise_dist{0, motionNoise(velocity, yaw, parameters)};
        std::normal_distribution<double> theta_noise_dist{0, motionNoise(velocity, yaw, parameters)};

        for (auto& p : particles) {
            p.x += (velocity /yaw) * (-1 * sin(p.theta) + sin(p.theta + yaw * delta_t));
            p.y += (velocity /yaw) * (cos(p.theta) - cos(p.theta + yaw * delta_t));
            p.theta += yaw * delta_t;
            
            // Add random noise
            p.x += x_noise_dist(gen);
            p.y += y_noise_dist(gen);
            p.theta += theta_noise_dist(gen);
        }
    }

private:
    std::vector<Particle> particles;
    int num_particles;
    // Random number generator with random seed
    std::random_device rd{};
    std::mt19937 gen;
    std::vector<double> parameters;
};