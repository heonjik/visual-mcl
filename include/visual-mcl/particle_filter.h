#ifndef VISUAL_MCL_PARTICLE_FILTER_HPP
#define VISUAL_MCL_PARTICLE_FILTER_HPP

#include <vector>
#include <random>

struct Particle {
    double x, y, theta;
    double weight;
};

struct Landmark {
    double x, y;
};

class ParticleFilter {
public:
    explicit ParticleFilter(int num_particles);

    void init(double mean_x, double mean_y, double mean_theta, const std::vector<double>& std_vec);
    void motionUpdate(double delta_t, double velocity, double yaw);
    void updateWeights(const std::vector<Landmark>& landmarks, const std::vector<double>& observations, double std_range);
    void resample();
    Particle getBestEstimate() const;

private:
    std::vector<Particle> particles;
    int num_particles;
    // Random number generator with random seed
    std::random_device rd{};
    std::mt19937 gen;
    std::vector<double> parameters;

    double motionNoise(double v, double w, const std::vector<double> p);
    double gaussianPdf(double mean, double x, double std_dev);
    void normalizeWeights();
};

#endif // VISUAL_MCL_PARTICLE_FILTER_HPP