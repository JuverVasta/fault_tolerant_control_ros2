// ftc_motor_model.hpp
//
// Gazebo Classic model plugin that simulates the dynamics of a quadrotor
// with INDIVIDUAL motor control. It is the ROS 2 native replacement for the
// RotorS simulator used by the original fault_tolerant_control project.
//
// What it does:
//   * Subscribes to a ROS 2 topic carrying desired rotor angular velocities
//     (quad_msgs/Actuators).
//   * For each of the 4 rotors, computes thrust = k_thrust * omega^2 and a
//     reaction torque = k_torque * omega^2, and applies them to the
//     corresponding link of the quadrotor model.
//   * Publishes ground-truth odometry of the base link as nav_msgs/Odometry.
//
// This is a standard quadrotor force model (see e.g. Mahony et al.,
// "Multirotor Aerial Vehicles", IEEE RAM 2012). It is intentionally simple
// and transparent so it can be described fully in a thesis.

#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quad_msgs/msg/actuators.hpp>

namespace ftc_gazebo_plugin {

class FtcMotorModel : public gazebo::ModelPlugin {
public:
  FtcMotorModel();
  ~FtcMotorModel() override;

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  void OnUpdate();
  void ActuatorCallback(const quad_msgs::msg::Actuators::SharedPtr msg);

  // Gazebo handles
  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr base_link_;
  std::array<gazebo::physics::LinkPtr, 4> rotor_links_;
  gazebo::event::ConnectionPtr update_connection_;

  // ROS 2
  std::shared_ptr<rclcpp::Node> ros_node_;
  rclcpp::Subscription<quad_msgs::msg::Actuators>::SharedPtr actuator_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;

  // Parameters
  std::string robot_namespace_;
  std::array<std::string, 4> rotor_link_names_;
  std::array<int, 4> turning_direction_;   // +1 CCW, -1 CW
  double motor_constant_;     // thrust  = motor_constant_  * omega^2
  double moment_constant_;    // torque  = moment_constant_ * thrust  (per rotor)
  double max_rot_velocity_;   // rad/s clamp
  double odometry_rate_;      // Hz

  // State
  std::mutex cmd_mutex_;
  std::array<double, 4> ref_motor_speed_{{0, 0, 0, 0}};
  gazebo::common::Time last_odometry_time_;
};

}  // namespace ftc_gazebo_plugin
