#pragma once
#include <Eigen/Dense>
#include <vector>
#include <random>
#include <cmath>

struct Wall {
    Eigen::Vector2f start;
    Eigen::Vector2f end;
};

struct Particle{
    float x, y, theta, weight;
};

struct UpdateResult {
    Eigen::Vector3f estimated_pose;
    float Neff;
    float min_weight;
    float max_weight;
    float mean_weight;
};

class Map {
public:
    Map();

    std::vector<float> simulateRays(const Particle& p, float max_range) const;
    std::vector<float> senseRays(const Particle& p, float max_range);

    const std::vector<Wall>& getWalls() const;
    const std::vector<Eigen::Vector2f>& getLandmarks() const;

private:
    // 2D scalar cross product helper
    float cross(const Eigen::Vector2f& a, const Eigen::Vector2f& b) const {
        return a.x() * b.y() - a.y() * b.x();
    }

    bool intersectRayWithSegment(const Eigen::Vector2f& origin, const Eigen::Vector2f& direction,
                                 const Wall& wall, float& out_distance) const;

    void castRay(const Eigen::Vector2f& origin, const Eigen::Vector2f& direction,
                 const float max_range, float& distance_hit) const;

    std::vector<Wall> walls_;
    std::vector<Eigen::Vector2f> landmarks_;
};

class Robot {
public:
    Robot(float sigma_pos, float sigma_rot, float sigma_sense, float x, float y, float theta);

    void moveForward(float distance);
    void rotate(float angle);
    std::vector<float> senseAllLandmarks(const std::vector<Eigen::Vector2f>& landmarks);

    // Getter functions
    float getSigmaPos() const { return sigma_pos_; }
    float getSigmaRot() const { return sigma_rot_; }
    float getSigmaSense() const { return sigma_sense_; }

    float getX() const { return x_; }
    float getY() const { return y_; }
    float getTheta() const { return theta_; }

    // Print state (for testing)
    void printState() const;

private:
    float x_, y_, theta_;
    float sigma_pos_; // standard deviation for position
    float sigma_rot_; // standard deviation for rotation
    float sigma_sense_; // standard deviation for sensing

    std::default_random_engine gen_; // Random number generation
    std::normal_distribution<float> dist_pos_; // Noise for position
    std::normal_distribution<float> dist_rot_; // Noise for rotation
    std::normal_distribution<float> dist_sense_; // Noise for sensing
};

class ParticleFilter{
public:
    ParticleFilter(float sigma_pos, float sigma_rot, float sigma_sense, float sigma_rot1, float sigma_trans, float sigma_rot2);

    std::vector<Particle> getParticles() const { return particles_; }
    void initializeParticles(float robot_x, float robot_y, float robot_theta, int num_particles);
    UpdateResult updateAndEstimate(float delta_rot1, float delta_trans, float delta_rot2, const std::vector<float>& measurements, const Map& map, float max_range);
    float getSenseNoise();

private:
    std::vector<Particle> particles_;
    float sigma_pos_, sigma_rot_, sigma_sense_;
    std::default_random_engine gen_;
    std::normal_distribution<float> dist_pos_;
    std::normal_distribution<float> dist_rot_;
    std::normal_distribution<float> dist_sense_;

    std::normal_distribution<float> dist_rot1_;
    std::normal_distribution<float> dist_trans_;
    std::normal_distribution<float> dist_rot2_;

    void moveParticle(Particle& p, float delta_rot1, float delta_trans, float delta_rot2);
    void updateMotion(float delta_rot1, float delta_trans, float delta_rot2);
    void calculateWeights(const std::vector<float>& measurements, const Map& map, float max_range);
    void resampleParticles();
    Eigen::Vector3f estimateState() const;
    float gaussian(float mean, float stddev, float x);
};