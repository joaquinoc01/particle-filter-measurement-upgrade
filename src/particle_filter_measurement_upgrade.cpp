#include <iostream>
#include <fstream>

#include "particle_filter_measurement_upgrade.hpp"

constexpr float TWO_PI = 2.0f * M_PI;

Map::Map() {
    walls_ = {
        {{0.0f, 0.0f}, {10.0f, 0.0f}},   // Bottom wall
        {{10.0f, 0.0f}, {10.0f, 10.0f}}, // Right wall
        {{10.0f, 10.0f}, {0.0f, 10.0f}}, // Top wall
        {{0.0f, 10.0f}, {0.0f, 0.0f}}    // Left wall
    };
    landmarks_ = {
        {0.0f, 0.0f},
        {0.0f, 10.0f},
        {5.0f, 7.0f},
        {10.0f, 10.0f},
        {10.0f, 0.0f}
    };
}

bool Map::intersectRayWithSegment(const Eigen::Vector2f& origin, const Eigen::Vector2f& direction, const Wall& wall, float& out_distance) const
{
    Eigen::Vector2f q { wall.start };
    Eigen::Vector2f s { wall.end };
    Eigen::Vector2f v { s - q };

    float D { direction.x() * v.y() - direction.y() * v.x() };

    Eigen::Vector2f w { q - origin};
    float t { cross(q - origin, v) / D };
    float u { cross(q - origin, direction) / D };

    if(std::abs(D) < 1e-6) // Wall is paralell to raycasting
        return false;
    else if(t >= 0 && u <= 1 && u >= 0)
    {
        out_distance = t;
        return true;
    }
    return false;
}

void Map::castRay(const Eigen::Vector2f& origin, const Eigen::Vector2f& direction,
                  const float max_range, float& distance_hit) const
{
    distance_hit = max_range;
    float new_distance { distance_hit };
    for (const auto& wall : walls_)
    {
        if (intersectRayWithSegment(origin, direction, wall, new_distance)) // If the ray is not paralel to the wall
        {
            if (new_distance < distance_hit)
                distance_hit = new_distance;
        }
    }
}

std::vector<float> Map::simulateRays(const Particle& p, float max_range) const
{
    std::vector<float> angles;
    int num_rays = 19;
    float fov = 180.0f; // Field of view in degrees
    float angle_increment = fov / (num_rays - 1);

    for (int i = 0; i < num_rays; ++i) {
        angles.push_back(-fov / 2.0f + i * angle_increment);
    }

    std::vector<float> results(angles.size());
    Eigen::Vector2f origin { p.x, p.y };
    for (size_t i{0}; i < angles.size(); ++i)
    {
        float angle { (p.theta + angles[i]) * static_cast<float>(M_PI) / 180.0f };
        Eigen::Vector2f d { cos(angle), sin(angle) };
        castRay(origin, d, max_range, results[i]);
    }
    return results;
}

const std::vector<Wall>& Map::getWalls() const {
    return walls_;
}

const std::vector<Eigen::Vector2f>& Map::getLandmarks() const{
    return landmarks_;
}

Robot::Robot(float sigma_pos, float sigma_rot, float sigma_sense, float x, float y, float theta) : x_(x), y_(y), theta_(theta), sigma_pos_(sigma_pos), sigma_rot_(sigma_rot), sigma_sense_(sigma_sense), dist_pos_(0.0f, sigma_pos_), dist_rot_(0.0f, sigma_rot_), dist_sense_(0.0f, sigma_sense_), gen_(std::random_device{}()) {}

void Robot::moveForward(float distance)
{
    float noisy_distance {distance + dist_pos_(gen_)};
    x_ += noisy_distance * cos(theta_);
    y_ += noisy_distance * sin(theta_);
}

void Robot::rotate(float rotation)
{
    float noisy_rotation {rotation + dist_rot_(gen_)};
    theta_ = fmod(theta_ + noisy_rotation, TWO_PI);
    if(theta_ < 0) theta_ += TWO_PI; // Keep theta in range (0 - 2π)
}

std::vector<float> Robot::senseAllLandmarks(const std::vector<Eigen::Vector2f>& landmarks)
{
    std::vector<float> measurements;
    for (const auto& lm : landmarks)
    {
        float dx = lm[0] - x_;
        float dy = lm[1] - y_;
        float dist = std::sqrt(dx * dx + dy * dy);
        measurements.push_back(dist + dist_sense_(gen_));
    }
    return measurements;
}

void Robot::printState() const
{
    std::cout << "Robot state: x = " << x_
              << ", y = " << y_
              << ", theta = " << theta_ << std::endl;
}

ParticleFilter::ParticleFilter(float sigma_pos, float sigma_rot, float sigma_sense, float sigma_rot1, float sigma_trans, float sigma_rot2) : sigma_pos_(sigma_pos), sigma_rot_(sigma_rot), sigma_sense_(sigma_sense), dist_pos_(0.0f, sigma_pos_), dist_rot_(0.0f, sigma_rot), dist_sense_(0.0f, sigma_sense_), dist_rot1_(0.0f, sigma_rot1), dist_trans_(0.0f, sigma_trans), dist_rot2_(0.0f, sigma_rot2), gen_(std::random_device{}()) {}

void ParticleFilter::initializeParticles(float robot_x, float robot_y, float robot_theta, int num_particles)
{
    particles_.clear();
    std::normal_distribution<float> normal_x(robot_x, sigma_pos_);
    std::normal_distribution<float> normal_y(robot_y, sigma_pos_);
    std::normal_distribution<float> normal_theta(robot_theta, sigma_rot_);

    for(int i{0}; i < num_particles; ++i)
    {
        Particle p{normal_x(gen_), normal_y(gen_), normal_theta(gen_), 1.0f/num_particles};
        particles_.push_back(p);
    }
}

void ParticleFilter::moveParticle(Particle& p, float delta_rot1, float delta_trans, float delta_rot2)
{
    float noisy_rot1 { delta_rot1 + dist_rot1_(gen_) };
    float noisy_trans { delta_trans + dist_trans_(gen_) };
    float noisy_rot2 { delta_rot2 + dist_rot2_(gen_) };

    p.theta += noisy_rot1;
    p.theta = fmod(p.theta, TWO_PI);
    if(p.theta < 0) p.theta += TWO_PI;

    p.x += noisy_trans * cos(p.theta);
    p.y += noisy_trans * sin(p.theta);

    p.theta += noisy_rot2;
    p.theta = fmod(p.theta, TWO_PI);
    if(p.theta < 0) p.theta += TWO_PI;
}

void ParticleFilter::updateMotion(float delta_rot1, float delta_trans, float delta_rot2)
{
    for(auto& p : particles_)
    {
        moveParticle(p, delta_rot1, delta_trans, delta_rot2);
    }
}

float ParticleFilter::gaussian(float mean, float stddev, float x) {
    float exponent = -((x - mean) * (x - mean)) / (2.0f * stddev * stddev);
    float normalization = 1.0f / (std::sqrt(2.0f * M_PI) * stddev);
    return normalization * std::exp(exponent);
}

void ParticleFilter::calculateWeights(const std::vector<float>& measurements,
                                      const Map& map, float max_range)
{
    float total_weight{0.0f};
    const float gauss_norm {1.0f / std::sqrt(2.0f * static_cast<float>(M_PI * sigma_sense_ * sigma_sense_))};
    const float denom {2.0f * sigma_sense_ * sigma_sense_};

    for (auto& p : particles_) {
        auto expected = map.simulateRays(p, max_range);

        float log_weight = 0.0f;
        for (size_t i = 0; i < measurements.size(); ++i) {
            float error = measurements[i] - expected[i];
            float exponent = -(error * error) / (2.0f * sigma_sense_ * sigma_sense_);
            log_weight += std::log(gauss_norm) + exponent;
        }
        p.weight = std::exp(log_weight);
        total_weight += p.weight;
    }

    if (total_weight == 0.0f) {
        std::cerr << "[WARNING] All particle weights are zero. Resetting weights uniformly." << std::endl;
        for (auto& p : particles_) {
            p.weight = 1.0f / particles_.size();
        }
        return;
    }

    for (auto& p : particles_) {
        p.weight /= total_weight;
    }
}

void ParticleFilter::resampleParticles()
{
    std::vector<Particle> new_particles;
    std::vector<float> weights;
    for (const auto& p : particles_)
        weights.push_back(p.weight);

    std::discrete_distribution<int> distribution(weights.begin(), weights.end());
    
    for (size_t i{0}; i < particles_.size(); ++i)
    {
        int index {distribution(gen_)};
        new_particles.push_back(particles_[index]);
    }
    particles_ = std::move(new_particles);
}

Eigen::Vector3f ParticleFilter::estimateState() const
{
    Eigen::Vector3f estimated_pose;
    float sum_x{0}, sum_y{0}, sum_sin{0}, sum_cos{0};
    for (const auto& p : particles_) {
        sum_x += p.x;
        sum_y += p.y;
        sum_sin += sin(p.theta);
        sum_cos += cos(p.theta);
    }
    estimated_pose[0] = sum_x/particles_.size();
    estimated_pose[1] = sum_y/particles_.size();
    estimated_pose[2] = atan2(sum_sin/particles_.size(), sum_cos/particles_.size());

    return estimated_pose;
}

UpdateResult ParticleFilter::updateAndEstimate(float delta_rot1, float delta_trans, float delta_rot2,
                                              const std::vector<float>& measurements,
                                              const Map& map, float max_range)
{
    updateMotion(delta_rot1, delta_trans, delta_rot2);
    calculateWeights(measurements, map, max_range);

    float total_weight_squared = 0.0f;
    float min_w = std::numeric_limits<float>::max();
    float max_w = std::numeric_limits<float>::lowest();
    float sum_w = 0.0f;

    for (const auto& p : particles_) {
        total_weight_squared += p.weight * p.weight;
        if (p.weight < min_w) min_w = p.weight;
        if (p.weight > max_w) max_w = p.weight;
        sum_w += p.weight;
    }
    float Neff = 1.0f / (total_weight_squared + 1e-6f);

    if (Neff < particles_.size() / 2) {
        resampleParticles();
    }

    Eigen::Vector3f estimated_pose = estimateState();

    return {estimated_pose, Neff, min_w, max_w, sum_w / particles_.size()};
}

float ParticleFilter::getSenseNoise()
{
    return dist_sense_(gen_);
}

int main()
{
    float sigma_pos = 0.1f, sigma_rot = 0.07f, sigma_sense = 0.3f;
    float sigma_rot1 = 0.07f, sigma_trans = 0.15f, sigma_rot2 = 0.07f;
    float x = 1.0f, y = 1.0f, theta = 0.0f;
    int num_particles = 500;
    float max_range = 10.0f;

    Map map;
    Robot robot(sigma_pos, sigma_rot, sigma_sense, x, y, theta);
    ParticleFilter pf(sigma_pos, sigma_rot, sigma_sense, sigma_rot1, sigma_trans, sigma_rot2);
    pf.initializeParticles(x, y, theta, num_particles);

    std::vector<int> side_lengths = {8, 8, 8, 8};
    float forward_distance = 1.0f;
    float turn_angle = -M_PI / 2.0f;

    std::ofstream log_file("trajectory_log.csv");
    log_file << "robot_x,robot_y,estimate_x,estimate_y\n";

    for (int side = 0; side < 4; ++side)
    {
        for (int step = 0; step < side_lengths[side]; ++step)
        {
            float delta_rot1 = 0.0f, delta_trans = forward_distance, delta_rot2 = 0.0f;
            robot.moveForward(delta_trans);
            Particle robot_pose { robot.getX(), robot.getY(), robot.getTheta(), 1.0f };
            auto measurements = map.simulateRays(robot_pose, max_range);
            for (auto& m : measurements)
                m += pf.getSenseNoise();

            auto result = pf.updateAndEstimate(delta_rot1, delta_trans, delta_rot2,
                                            measurements, map, max_range);

            log_file << robot.getX() << "," << robot.getY() << "," 
                    << result.estimated_pose[0] << "," << result.estimated_pose[1] << "\n";

            std::cout << "Step " << step << ", Neff: " << result.Neff
                    << ", Min W: " << result.min_weight
                    << ", Max W: " << result.max_weight
                    << ", Mean W: " << result.mean_weight << std::endl;
            
            float mean_x = 0.0f, mean_y = 0.0f;
            for (const auto& p : pf.getParticles()) {
                mean_x += p.x;
                mean_y += p.y;
            }
            mean_x /= pf.getParticles().size();
            mean_y /= pf.getParticles().size();

            float var_x = 0.0f, var_y = 0.0f;
            for (const auto& p : pf.getParticles()) {
                var_x += (p.x - mean_x) * (p.x - mean_x);
                var_y += (p.y - mean_y) * (p.y - mean_y);
            }
            var_x /= pf.getParticles().size();
            var_y /= pf.getParticles().size();

            std::cout << "Step " << step << " - Spread: std_x = "
                    << std::sqrt(var_x) << ", std_y = " << std::sqrt(var_y) << std::endl;

        }

        float delta_rot1 = turn_angle, delta_trans = 0.0f, delta_rot2 = 0.0f;
        robot.rotate(delta_rot1);
        Particle robot_pose { robot.getX(), robot.getY(), robot.getTheta(), 1.0f };
        auto measurements = map.simulateRays(robot_pose, max_range);
        for (auto& m : measurements)
            m += pf.getSenseNoise();

        auto result = pf.updateAndEstimate(delta_rot1, delta_trans, delta_rot2,
                                        measurements, map, max_range);

        log_file << robot.getX() << "," << robot.getY() << "," 
                << result.estimated_pose[0] << "," << result.estimated_pose[1] << "\n";

        std::cout << "Turn step, Neff: " << result.Neff
                << ", Min W: " << result.min_weight
                << ", Max W: " << result.max_weight
                << ", Mean W: " << result.mean_weight << std::endl;
    }
    log_file.close();
    return 0;
}
