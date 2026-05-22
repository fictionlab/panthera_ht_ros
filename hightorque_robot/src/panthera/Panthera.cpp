#include "panthera/Panthera.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>

namespace panthera
{

// ==================== Constructors and destructors ====================

Panthera::Panthera(const std::string& config_path)
    : hightorque_robot::robot(config_path), motor_count_(0), gripper_id_(0)
{
    initialize(config_path);
}

Panthera::~Panthera()
{
}

// ==================== Initialization methods ====================

void Panthera::initialize(const std::string& config_path)
{
    // Load configuration file
    loadConfig(config_path);

    // Save configuration file directory
    size_t last_slash = config_path.find_last_of("/\\");
    if (last_slash != std::string::npos) {
        config_dir_ = config_path.substr(0, last_slash);
    } else {
        config_dir_ = ".";
    }

    // Note: the base class constructor is called in the initializer list and will invoke init_robot

    // Get motor count (excluding gripper)
    motor_count_ = Motors.size() - 1;
    gripper_id_ = Motors.size();

    std::cout << "Initializing Panthera arm..." << std::endl;
    std::cout << "Found " << motor_count_ << " motors" << std::endl;

    if (motor_count_ == 0) {
        std::cerr << "No motors found. Please check your configuration and connections." << std::endl;
        return;
    }

    // Print motor information
    for (size_t i = 0; i < Motors.size(); ++i) {
        std::cout << "Motor " << i << ": "
                  << "ID=" << Motors[i]->get_motor_id() << ", "
                  << "Type=" << static_cast<int>(Motors[i]->get_motor_enum_type()) << ", "
                  << "Name=" << Motors[i]->get_motor_name() << std::endl;
    }
}

void Panthera::loadConfig(const std::string& config_path)
{
    try {
        config_ = YAML::LoadFile(config_path);
        std::cout << "Configuration file loaded: " << config_path << std::endl;

        // Read joint limits
        if (config_["robot"] && config_["robot"]["joint_limits"]) {
            auto limits = config_["robot"]["joint_limits"];
            joint_limits_lower_ = limits["lower"].as<std::vector<double>>();
            joint_limits_upper_ = limits["upper"].as<std::vector<double>>();

            std::cout << "Joint limits loaded: lower=[";
            for (size_t i = 0; i < joint_limits_lower_.size(); ++i) {
                std::cout << joint_limits_lower_[i];
                if (i < joint_limits_lower_.size() - 1) std::cout << ", ";
            }
            std::cout << "], upper=[";
            for (size_t i = 0; i < joint_limits_upper_.size(); ++i) {
                std::cout << joint_limits_upper_[i];
                if (i < joint_limits_upper_.size() - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
        } else {
            std::cerr << "Warning: 'joint_limits' not found in configuration file" << std::endl;
        }

        // Read joint names
        if (config_["kinematics"] && config_["kinematics"]["joint_names"]) {
            joint_names_ = config_["kinematics"]["joint_names"].as<std::vector<std::string>>();
        }

    } catch (const YAML::Exception& e) {
        std::cerr << "Failed to load configuration file: " << e.what() << std::endl;
        throw;
    }
}

bool Panthera::checkJointLimits(const std::vector<double>& pos)
{
    if (joint_limits_lower_.empty() || joint_limits_upper_.empty()) {
        return true; // No joint limits configured -> pass
    }

    if (pos.size() > joint_limits_lower_.size()) {
        std::cerr << "Error: position array size exceeds joint limits configuration" << std::endl;
        return false;
    }

    bool all_in_range = true;
    std::vector<int> out_indices;

    for (size_t i = 0; i < pos.size(); ++i) {
        if (pos[i] < joint_limits_lower_[i] || pos[i] > joint_limits_upper_[i]) {
            all_in_range = false;
            out_indices.push_back(i);
        }
    }

    if (!all_in_range) {
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "Warning: target position exceeds joint limits!" << std::endl;
        std::cout << "Target positions: [";
        for (size_t i = 0; i < pos.size(); ++i) {
            std::cout << pos[i];
            if (i < pos.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        std::cout << "Limits lower: [";
        for (size_t i = 0; i < joint_limits_lower_.size(); ++i) {
            std::cout << joint_limits_lower_[i];
            if (i < joint_limits_lower_.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        std::cout << "Limits upper: [";
        for (size_t i = 0; i < joint_limits_upper_.size(); ++i) {
            std::cout << joint_limits_upper_[i];
            if (i < joint_limits_upper_.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        for (int idx : out_indices) {
            std::cout << "  Joint " << (idx + 1) << ": " << pos[idx]
                      << " not in [" << joint_limits_lower_[idx]
                      << ", " << joint_limits_upper_[idx] << "]" << std::endl;
        }
        std::cout << "Command rejected to protect the robot" << std::endl;
        std::cout << std::string(60, '=') << "\n" << std::endl;
        return false;
    }

    return true;
}

// ==================== Status Accessors ====================

std::vector<double> Panthera::getCurrentPos()
{
    std::vector<double> positions(motor_count_);
    for (int i = 0; i < motor_count_; ++i) {
        auto state = Motors[i]->get_current_motor_state();
        positions[i] = state->position;
    }
    return positions;
}

std::vector<double> Panthera::getCurrentVel()
{
    std::vector<double> velocities(motor_count_);
    for (int i = 0; i < motor_count_; ++i) {
        auto state = Motors[i]->get_current_motor_state();
        velocities[i] = state->velocity;
    }
    return velocities;
}

std::vector<double> Panthera::getCurrentTorque()
{
    std::vector<double> torques(motor_count_);
    for (int i = 0; i < motor_count_; ++i) {
        auto state = Motors[i]->get_current_motor_state();
        torques[i] = state->torque;
    }
    return torques;
}

double Panthera::getCurrentPosGripper()
{
    auto state = Motors[gripper_id_ - 1]->get_current_motor_state();
    return state->position;
}

double Panthera::getCurrentVelGripper()
{
    auto state = Motors[gripper_id_ - 1]->get_current_motor_state();
    return state->velocity;
}

double Panthera::getCurrentTorqueGripper()
{
    auto state = Motors[gripper_id_ - 1]->get_current_motor_state();
    return state->torque;
}

// ==================== Control Interface ====================

bool Panthera::posVelMaxTorque(const std::vector<double>& pos,
                                const std::vector<double>& vel,
                                const std::vector<double>& max_torque,
                                bool is_wait,
                                double tolerance,
                                double timeout)
{
    // Check parameter lengths
    if (pos.size() != motor_count_ || vel.size() != motor_count_ ||
        max_torque.size() != motor_count_) {
        std::cerr << "Error: joint parameter length must be " << motor_count_ << std::endl;
        return false;
    }

    // Check joint limits
    if (!checkJointLimits(pos)) {
        return false;
    }

    // Control joints (excluding gripper motor)
    for (int i = 0; i < motor_count_; ++i) {
        Motors[i]->pos_vel_MAXtqe(pos[i], vel[i], max_torque[i]);
    }
    motor_send_cmd();

    if (is_wait) {
        return waitForPosition(pos, tolerance, timeout);
    }

    return true;
}

bool Panthera::posVelTorqueKpKd(const std::vector<double>& pos,
                                 const std::vector<double>& vel,
                                 const std::vector<double>& torque,
                                 const std::vector<double>& kp,
                                 const std::vector<double>& kd)
{
    // Check parameter lengths
    if (pos.size() != motor_count_ || vel.size() != motor_count_ ||
        torque.size() != motor_count_ || kp.size() != motor_count_ ||
        kd.size() != motor_count_) {
        std::cerr << "Error: joint parameter length must be " << motor_count_ << std::endl;
        return false;
    }

    // Check joint limits
    if (!checkJointLimits(pos)) {
        return false;
    }

    // Control joints (excluding gripper motor)
    for (int i = 0; i < motor_count_; ++i) {
        Motors[i]->pos_vel_tqe_kp_kd(pos[i], vel[i], torque[i], kp[i], kd[i]);
    }
    motor_send_cmd();

    return true;
}

// ==================== Gripper Control Interface ====================

bool Panthera::gripperControl(double pos, double vel, double max_torque)
{
    Motors[gripper_id_ - 1]->pos_vel_MAXtqe(pos, vel, max_torque);
    motor_send_cmd();
    return true;
}

bool Panthera::gripperControlMIT(double pos, double vel, double torque,
                                  double kp, double kd)
{
    Motors[gripper_id_ - 1]->pos_vel_tqe_kp_kd(pos, vel, torque, kp, kd);
    motor_send_cmd();
    return true;
}

void Panthera::gripperOpen(double vel, double max_torque)
{
    gripperControl(0.8, vel, max_torque);
}

void Panthera::gripperClose(double pos, double vel, double max_torque)
{
    gripperControl(pos, vel, max_torque);
}

// ==================== Position Check Interface ====================

bool Panthera::checkPositionReached(const std::vector<double>& target_positions,
                                     double tolerance,
                                     std::vector<double>& position_errors)
{
    bool all_reached = true;
    position_errors.clear();
    position_errors.resize(motor_count_);

    send_get_motor_state_cmd();
    motor_send_cmd();

    // Check the first N joints (excluding gripper)
    for (int i = 0; i < motor_count_; ++i) {
        auto state = Motors[i]->get_current_motor_state();
        double error = std::abs(state->position - target_positions[i]);
        position_errors[i] = error;
        if (error > tolerance) {
            all_reached = false;
        }
    }

    return all_reached;
}

bool Panthera::waitForPosition(const std::vector<double>& target_positions,
                                double tolerance,
                                double timeout)
{
    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - start_time).count() / 1000.0;

        if (elapsed >= timeout) {
            return false;
        }

        std::vector<double> errors;
        if (checkPositionReached(target_positions, tolerance, errors)) {
            return true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

// ==================== Utility Methods ====================

void Panthera::getJointLimits(std::vector<double>& lower, std::vector<double>& upper) const
{
    lower = joint_limits_lower_;
    upper = joint_limits_upper_;
}

} // namespace panthera
