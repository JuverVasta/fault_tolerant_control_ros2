// ftc_motor_model.cpp — hardened version.
// The whole Load() is wrapped in try/catch so any failure prints a clear
// reason instead of a silent "Exception occured in Load".

#include "ftc_gazebo_plugin/ftc_motor_model.hpp"

#include <gazebo_ros/node.hpp>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

namespace ftc_gazebo_plugin {

namespace {

std::string getString(sdf::ElementPtr sdf, const std::string& key,
                       const std::string& def) {
  if (sdf && sdf->HasElement(key)) {
    try { return sdf->Get<std::string>(key); } catch (...) {}
  }
  return def;
}

double getDouble(sdf::ElementPtr sdf, const std::string& key, double def) {
  if (sdf && sdf->HasElement(key)) {
    try { return sdf->Get<double>(key); } catch (...) {}
  }
  return def;
}

}  // namespace

FtcMotorModel::FtcMotorModel() = default;

FtcMotorModel::~FtcMotorModel() {
  if (executor_) executor_->cancel();
  if (spin_thread_.joinable()) spin_thread_.join();
}

void FtcMotorModel::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
  try {
    model_ = model;

    robot_namespace_ = getString(sdf, "robotNamespace", "hummingbird");
    std::string base_link_name = getString(sdf, "baseLinkName", "base_link");

    rotor_link_names_[0] = getString(sdf, "rotor0LinkName", "rotor_0");
    rotor_link_names_[1] = getString(sdf, "rotor1LinkName", "rotor_1");
    rotor_link_names_[2] = getString(sdf, "rotor2LinkName", "rotor_2");
    rotor_link_names_[3] = getString(sdf, "rotor3LinkName", "rotor_3");

    turning_direction_[0] = static_cast<int>(getDouble(sdf, "rotor0Direction", 1.0));
    turning_direction_[1] = static_cast<int>(getDouble(sdf, "rotor1Direction", -1.0));
    turning_direction_[2] = static_cast<int>(getDouble(sdf, "rotor2Direction", 1.0));
    turning_direction_[3] = static_cast<int>(getDouble(sdf, "rotor3Direction", -1.0));

    motor_constant_ = getDouble(sdf, "motorConstant", 8.54858e-06);
    moment_constant_ = getDouble(sdf, "momentConstant", 0.016);
    max_rot_velocity_ = getDouble(sdf, "maxRotVelocity", 1500.0);
    odometry_rate_ = getDouble(sdf, "odometryRate", 200.0);

    base_link_ = model_->GetLink(base_link_name);
    if (!base_link_) {
      gzerr << "[ftc_motor_model] base link '" << base_link_name
            << "' not found. Available links:\n";
      for (const auto& l : model_->GetLinks())
        gzerr << "    - " << l->GetName() << "\n";
      gzerr << "[ftc_motor_model] Plugin disabled.\n";
      return;
    }
    for (int i = 0; i < 4; ++i) {
      rotor_links_[i] = model_->GetLink(rotor_link_names_[i]);
      if (!rotor_links_[i]) {
        gzerr << "[ftc_motor_model] rotor link '" << rotor_link_names_[i]
              << "' not found. Available links:\n";
        for (const auto& l : model_->GetLinks())
          gzerr << "    - " << l->GetName() << "\n";
        gzerr << "[ftc_motor_model] Plugin disabled.\n";
        return;
      }
    }

    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    ros_node_ = std::make_shared<rclcpp::Node>("ftc_motor_model");

    const std::string ns = "/" + robot_namespace_;
    actuator_sub_ = ros_node_->create_subscription<quad_msgs::msg::Actuators>(
        ns + "/command/motor_speed", rclcpp::QoS(10),
        std::bind(&FtcMotorModel::ActuatorCallback, this, std::placeholders::_1));

    odometry_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(
        ns + "/ground_truth/odometry", rclcpp::QoS(10));

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(ros_node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    last_odometry_time_ = model_->GetWorld()->SimTime();

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&FtcMotorModel::OnUpdate, this));

    gzmsg << "[ftc_motor_model] loaded successfully for namespace '"
          << robot_namespace_ << "'.\n";

  } catch (const std::exception& e) {
    gzerr << "[ftc_motor_model] Load() failed: " << e.what() << "\n";
  } catch (...) {
    gzerr << "[ftc_motor_model] Load() failed with unknown exception.\n";
  }
}

void FtcMotorModel::ActuatorCallback(
    const quad_msgs::msg::Actuators::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  const size_t n = std::min<size_t>(4, msg->angular_velocities.size());
  for (size_t i = 0; i < n; ++i) {
    double w = msg->angular_velocities[i];
    if (w < 0.0) w = 0.0;
    if (w > max_rot_velocity_) w = max_rot_velocity_;
    ref_motor_speed_[i] = w;
  }
}

void FtcMotorModel::OnUpdate() {
  std::array<double, 4> omega;
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    omega = ref_motor_speed_;
  }

// ---- Apply all forces and torques directly to the base link ----
  // Each rotor produces lift along the body +Z axis, applied at the rotor's
  // mounting point. Because the offset is relative to the centre of mass,
  // Gazebo automatically produces the correct roll/pitch moments. The yaw
  // reaction torque of each rotor is summed and applied separately.
  // Applying everything to base_link (instead of the rotor links) avoids
  // numerical artefacts from force transmission across fixed joints.
  double total_reaction_torque = 0.0;
  for (int i = 0; i < 4; ++i) {
    const double w2 = omega[i] * omega[i];
    const double thrust = motor_constant_ * w2;            // [N]
    const double drag_torque = moment_constant_ * thrust;  // [Nm]

    // Rotor mounting offset in the base-link frame.
    const ignition::math::Vector3d offset(
        rotor_links_[i]->RelativePose().Pos());

    base_link_->AddLinkForce(
        ignition::math::Vector3d(0.0, 0.0, thrust), offset);

    total_reaction_torque += -turning_direction_[i] * drag_torque;
  }

  base_link_->AddRelativeTorque(
      ignition::math::Vector3d(0.0, 0.0, total_reaction_torque));

  const gazebo::common::Time now = model_->GetWorld()->SimTime();
  const double dt = (now - last_odometry_time_).Double();
  if (dt < 1.0 / odometry_rate_) return;
  last_odometry_time_ = now;

  const ignition::math::Pose3d pose = base_link_->WorldPose();
  const ignition::math::Vector3d lin_vel = base_link_->RelativeLinearVel();
  const ignition::math::Vector3d ang_vel = base_link_->RelativeAngularVel();

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = ros_node_->now();
  odom.header.frame_id = "world";
  odom.child_frame_id = robot_namespace_ + "/base_link";

  odom.pose.pose.position.x = pose.Pos().X();
  odom.pose.pose.position.y = pose.Pos().Y();
  odom.pose.pose.position.z = pose.Pos().Z();
  odom.pose.pose.orientation.w = pose.Rot().W();
  odom.pose.pose.orientation.x = pose.Rot().X();
  odom.pose.pose.orientation.y = pose.Rot().Y();
  odom.pose.pose.orientation.z = pose.Rot().Z();

  odom.twist.twist.linear.x = lin_vel.X();
  odom.twist.twist.linear.y = lin_vel.Y();
  odom.twist.twist.linear.z = lin_vel.Z();
  odom.twist.twist.angular.x = ang_vel.X();
  odom.twist.twist.angular.y = ang_vel.Y();
  odom.twist.twist.angular.z = ang_vel.Z();

  odometry_pub_->publish(odom);
  
  // --- DEBUG: print physics state once per second ---
  static int dbg_counter = 0;
  if (++dbg_counter % 200 == 0) {
    gzmsg << "[ftc DEBUG] omega=[" << omega[0] << "," << omega[1] << ","
          << omega[2] << "," << omega[3] << "]  reaction_torque="
          << total_reaction_torque << "  bodyrate_z=" << ang_vel.Z()
          << "  roll=" << pose.Rot().Roll()
          << "  pitch=" << pose.Rot().Pitch() << "\n";
  }
  
}

GZ_REGISTER_MODEL_PLUGIN(FtcMotorModel)

}  // namespace ftc_gazebo_plugin
