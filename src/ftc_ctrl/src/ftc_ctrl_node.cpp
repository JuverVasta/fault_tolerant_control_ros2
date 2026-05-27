// ftc_ctrl_node.cpp — entry point for the fault-tolerant controller node.
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rclcpp/rclcpp.hpp>
#include "ftc_ctrl/ftc_ctrl.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ftc::NDICtrl>());
  rclcpp::shutdown();
  return 0;
}
