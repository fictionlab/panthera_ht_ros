#ifndef PANTHERA_HPP
#define PANTHERA_HPP

#include "../hardware/robot.hpp"
#include <vector>
#include <string>
#include <array>
#include <yaml-cpp/yaml.h>
#include <memory>

namespace panthera
{

/**
 * @brief High-level control class for the Panthera robotic arm
 *
 * Inherits from `hightorque_robot::robot` and provides high-level control
 * interfaces for the arm. Features include configuration loading, joint
 * state retrieval, position control, and gripper control. (Does not include
 * kinematics or dynamics functionality.)
 */
class Panthera : public hightorque_robot::robot
{
public:
    /**
     * @brief Constructor
     * @param config_path Path to the configuration file (YAML)
     */
    explicit Panthera(const std::string& config_path);

    /**
     * @brief Destructor
     */
    ~Panthera();

    // ==================== Status Accessors ====================

    /**
     * @brief Get current joint positions
     * @return Vector of joint positions (radians)
     */
    std::vector<double> getCurrentPos();

    /**
     * @brief Get current joint velocities
     * @return Vector of joint velocities (rad/s)
     */
    std::vector<double> getCurrentVel();

    /**
     * @brief Get current joint torques
     * @return Vector of joint torques (Nm)
     */
    std::vector<double> getCurrentTorque();

    /**
     * @brief Get current gripper position
     * @return Gripper position (radians)
     */
    double getCurrentPosGripper();

    /**
     * @brief Get current gripper velocity
     * @return Gripper velocity (rad/s)
     */
    double getCurrentVelGripper();

    /**
     * @brief Get current gripper torque
     * @return Gripper torque (Nm)
     */
    double getCurrentTorqueGripper();

    // ==================== Control Interface ====================

    /**
     * @brief Position-velocity with max torque control mode
     * @param pos Target positions (radians)
     * @param vel Target velocities (rad/s)
     * @param max_torque Max torque per joint (Nm)
     * @param is_wait Whether to wait for motion completion
     * @param tolerance Position tolerance (radians)
     * @param timeout Timeout in seconds
     * @return True on success
     */
    bool posVelMaxTorque(const std::vector<double>& pos,
                         const std::vector<double>& vel,
                         const std::vector<double>& max_torque,
                         bool is_wait = false,
                         double tolerance = 0.1,
                         double timeout = 15.0);

    /**
     * @brief Five-parameter MIT control mode
     * @param pos Target positions (radians)
     * @param vel Target velocities (rad/s)
     * @param torque Feedforward torque (Nm)
     * @param kp Proportional gains
     * @param kd Derivative gains
     * @return True on success
     */
    bool posVelTorqueKpKd(const std::vector<double>& pos,
                          const std::vector<double>& vel,
                          const std::vector<double>& torque,
                          const std::vector<double>& kp,
                          const std::vector<double>& kd);

    // ==================== Gripper Control Interface ====================

    /**
     * @brief Gripper control (position-velocity with max torque)
     * @param pos Target position (radians)
     * @param vel Target velocity (rad/s)
     * @param max_torque Max torque (Nm)
     * @return True on success
     */
    bool gripperControl(double pos, double vel, double max_torque);

    /**
     * @brief Gripper control (five-parameter MIT mode)
     * @param pos Target position (radians)
     * @param vel Target velocity (rad/s)
     * @param torque Feedforward torque (Nm)
     * @param kp Proportional gain
     * @param kd Derivative gain
     * @return True on success
     */
    bool gripperControlMIT(double pos, double vel, double torque,
                           double kp, double kd);

    /**
     * @brief Open gripper
     * @param vel Velocity (rad/s), default 0.5
     * @param max_torque Max torque (Nm), default 0.5
     */
    void gripperOpen(double vel = 0.5, double max_torque = 0.5);

    /**
     * @brief Close gripper
     * @param pos Target position (radians), default 0.0
     * @param vel Velocity (rad/s), default 0.5
     * @param max_torque Max torque (Nm), default 0.5
     */
    void gripperClose(double pos = 0.0, double vel = 0.5, double max_torque = 0.5);

    // ==================== Position Check Interface ====================

    /**
     * @brief Check whether joint positions reached targets
     * @param target_positions Target positions
     * @param tolerance Tolerance (radians)
     * @param position_errors Output per-joint position errors
     * @return True if all reached
     */
    bool checkPositionReached(const std::vector<double>& target_positions,
                              double tolerance,
                              std::vector<double>& position_errors);

    /**
     * @brief Wait for positions to be reached
     * @param target_positions Target positions
     * @param tolerance Tolerance (radians)
     * @param timeout Timeout in seconds
     * @return True if reached
     */
    bool waitForPosition(const std::vector<double>& target_positions,
                         double tolerance = 0.01,
                         double timeout = 15.0);

    // ==================== Utility Methods ====================

    /**
     * @brief Get number of motors (excluding gripper)
     * @return Number of motors
     */
    int getMotorCount() const { return motor_count_; }

    /**
     * @brief Get joint limits
     * @param lower Output lower limits
     * @param upper Output upper limits
     */
    void getJointLimits(std::vector<double>& lower, std::vector<double>& upper) const;

private:
    /**
     * @brief Initialize the robot
     * @param config_path Path to configuration file
     */
    void initialize(const std::string& config_path);

    /**
     * @brief Load configuration file
     * @param config_path Path to configuration file
     */
    void loadConfig(const std::string& config_path);

    /**
     * @brief Check whether positions are within joint limits
     * @param pos Target positions
     * @return True if within limits
     */
    bool checkJointLimits(const std::vector<double>& pos);

    // ==================== Member variables ====================

    YAML::Node config_;                          // Configuration content
    std::string config_dir_;                     // Configuration directory
    int motor_count_;                            // Number of motors (excluding gripper)
    int gripper_id_;                             // Gripper motor ID
    std::vector<double> joint_limits_lower_;     // Joint lower limits
    std::vector<double> joint_limits_upper_;     // Joint upper limits
    std::vector<std::string> joint_names_;       // Joint names
};

} // namespace panthera

#endif // PANTHERA_HPP
