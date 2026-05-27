// Copyright (C) 2021 Sihao Sun, RPG, University of Zurich, Switzerland
// Copyright (C) 2021 Davide Scaramuzza, RPG, University of Zurich, Switzerland
//
// ROS 2 (Humble) port of the fault-tolerant NDI controller.
// The control algorithm is unchanged from the original work; only the
// ROS middleware API has been migrated from ROS 1 to ROS 2.
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quad_msgs/msg/control_command.hpp>
#include <quad_msgs/msg/quad_state_estimate.hpp>

#include <Eigen/Eigen>

#include "ftc_ctrl/filter.h"

#define PI 3.14159265

namespace ftc {

struct QuadState {
  rclcpp::Time timestamp;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d bodyrates;
};

class NDICtrl : public rclcpp::Node {
 public:
  NDICtrl();
  ~NDICtrl() override;

 private:
  // ---- ROS 2 interfaces ----
  rclcpp::Subscription<quad_msgs::msg::QuadStateEstimate>::SharedPtr state_est_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr reference_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_rotors_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_rotors_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_rate_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<quad_msgs::msg::ControlCommand>::SharedPtr motor_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr kill_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std_msgs::msg::Bool armed_out_;
  quad_msgs::msg::ControlCommand motor_command_msg_;
  sensor_msgs::msg::Joy joy_msg_;

  QuadState state_;

  // ---- Parameters and state (unchanged from the original) ----
  int kMinMotCom_DShot_;
  int kMaxMotCom_DShot_;
  int f_id_ = -1;
  int failure_id_;

  double ctrl_rate_;
  double time_traj_update_;
  static constexpr double g_ = 9.81;
  Eigen::Vector3d g_vect_ = Eigen::Vector3d(0.0, 0.0, -g_);
  double nx_b_ = 0;
  double ny_b_ = 0;
  double n_primary_axis_;
  double max_vert_acc_;
  double max_pos_err_int_;
  double max_nb_err_int_;
  double max_horz_acc_;
  double max_horz_acc_fail_;

  Eigen::Vector3d position_at_update_;
  Eigen::Vector3d n_b_;
  Eigen::Vector3d nb_err_int_;
  Eigen::Vector3d pos_err_int_;
  Eigen::Vector3d a_des_;
  Eigen::Matrix4d G_inv_;
  Eigen::Vector4d thrust_;
  Eigen::Vector3d pos_design_;
  Eigen::Vector3d pos_design_current_position_;
  Eigen::Matrix3d Inertia_;

  double yaw_rate_err_int_;
  double m_, Ix_, Iy_, Iz_;
  double coeff1_, coeff2_, coeff3_;
  double time_now_;
  double time_fail_;
  double heading_target_;

  std::vector<double> Kp_pos_vec_, Kd_pos_vec_, Ki_pos_vec_, K_att_;
  std::vector<double> Kp_pos_vec_fail_, Kd_pos_vec_fail_, Ki_pos_vec_fail_, K_att_fail_;
  std::vector<double> K_yaw_;

  bool sigmoid_traj_;
  bool position_design_updated_ = false;
  bool use_notch_filter_;
  bool estimation_received_ = false;
  bool referenceUpdated_ = false;
  bool use_vio_ = false;
  bool joy_called_ = false;
  bool manual_mode_ = false;
  bool kill_mode_ = false;
  bool failure_mode_ = false;

  Eigen::Vector3d pos_filtered_notch_;
  Eigen::Vector3d vel_filtered_notch_;

  filter::Filter filter_pos_;
  filter::Filter filter_vel_;

  std::vector<double> lever_arm_x_, lever_arm_y_, drag_coeff_;
  std::vector<double> cg_bias_;

  // ---- Methods (identical algorithm to the original) ----
  bool loadParams();
  void startRotorsCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void stopRotorsCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void controlUpdateCallback();
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void stateUpdateCallback(const quad_msgs::msg::QuadStateEstimate::SharedPtr msg);
  void headingCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void referenceUpdateCallback(const geometry_msgs::msg::Point::SharedPtr msg);
  void calculateControlEffectiveness();
  void sendMotorCommand();
  void controller_outerloop();
  void controller_innerloop();
  double getDesignedYawRate();
  void failIdCallback();
  void integrator3(const Eigen::Vector3d& input, Eigen::Vector3d& output,
                   const double dt, const double max);
  void integrator(const double& input, double& output, const double dt,
                  const double max);
  void sigmoidTraj(const double pos_end, const double pos_begin, double& pos_des,
                   double& vel_des, double& acc_des, double time);
};

}  // namespace ftc
