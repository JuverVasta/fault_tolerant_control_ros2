// state_interface_node.cpp
//
// Glue node between the Gazebo simulation and the fault-tolerant controller.
// It performs exactly the role of the original `rotors_interface` package:
//
//   * Subscribes to /<ns>/ground_truth/odometry (nav_msgs/Odometry from the
//     Gazebo plugin) and republishes it as /<ns>/state_est
//     (quad_msgs/QuadStateEstimate) for the controller.
//
//   * Subscribes to /<ns>/control_command (quad_msgs/ControlCommand from the
//     controller) and converts the per-rotor thrust into rotor angular
//     velocities, publishing /<ns>/command/motor_speed (quad_msgs/Actuators)
//     for the Gazebo plugin.
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quad_msgs/msg/quad_state_estimate.hpp>
#include <quad_msgs/msg/control_command.hpp>
#include <quad_msgs/msg/actuators.hpp>

#include <Eigen/Eigen>

class StateInterface : public rclcpp::Node {
 public:
  StateInterface() : Node("state_interface") {
    // motor_constant_ converts thrust [N] -> omega^2.
    // It MUST match <motorConstant> in the Gazebo plugin / URDF.
    declare_parameter("motor_constant", 8.54858e-06);
    motor_constant_ = get_parameter("motor_constant").as_double();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "ground_truth/odometry", 10,
        std::bind(&StateInterface::odomCallback, this, std::placeholders::_1));

    cmd_sub_ = create_subscription<quad_msgs::msg::ControlCommand>(
        "control_command", 10,
        std::bind(&StateInterface::cmdCallback, this, std::placeholders::_1));

    state_pub_ = create_publisher<quad_msgs::msg::QuadStateEstimate>(
        "state_est", 10);
    motor_pub_ = create_publisher<quad_msgs::msg::Actuators>(
        "command/motor_speed", 10);

    RCLCPP_INFO(get_logger(), "[%s] state_interface ready.", get_namespace());
  }

 private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    quad_msgs::msg::QuadStateEstimate out;
    out.header = msg->header;

    out.position.x = msg->pose.pose.position.x;
    out.position.y = msg->pose.pose.position.y;
    out.position.z = msg->pose.pose.position.z;

    out.orientation = msg->pose.pose.orientation;

    // The Gazebo plugin publishes body-frame linear velocity; the controller
    // expects world-frame velocity. Rotate it.
    Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                         msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z);
    Eigen::Vector3d v_body(msg->twist.twist.linear.x,
                           msg->twist.twist.linear.y,
                           msg->twist.twist.linear.z);
    Eigen::Vector3d v_world = q * v_body;

    out.velocity.x = v_world.x();
    out.velocity.y = v_world.y();
    out.velocity.z = v_world.z();

    out.bodyrates.x = msg->twist.twist.angular.x;
    out.bodyrates.y = msg->twist.twist.angular.y;
    out.bodyrates.z = msg->twist.twist.angular.z;

    state_pub_->publish(out);
  }

  void cmdCallback(const quad_msgs::msg::ControlCommand::SharedPtr msg) {
    quad_msgs::msg::Actuators out;
    out.angular_velocities.resize(4);

    for (int i = 0; i < 4; ++i) {
      double thrust = msg->rotor_thrust[i];   // [N], per rotor
      if (thrust < 0.0) thrust = 0.0;
      // thrust = motor_constant_ * omega^2  ->  omega = sqrt(thrust / k)
      out.angular_velocities[i] = std::sqrt(thrust / motor_constant_);
    }
    motor_pub_->publish(out);
  }

  double motor_constant_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<quad_msgs::msg::ControlCommand>::SharedPtr cmd_sub_;
  rclcpp::Publisher<quad_msgs::msg::QuadStateEstimate>::SharedPtr state_pub_;
  rclcpp::Publisher<quad_msgs::msg::Actuators>::SharedPtr motor_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateInterface>());
  rclcpp::shutdown();
  return 0;
}
