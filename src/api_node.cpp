#include "laser_uav_px4_api/api_node.hpp"

namespace laser_uav_px4_api
{
/* ApiNode() //{ */
ApiNode::ApiNode(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("api_node", "", options) {
  RCLCPP_INFO(get_logger(), "Creating");

  declare_parameter("rate.pub_offboard_control_mode", rclcpp::ParameterValue(100.0));
  declare_parameter("rate.pub_api_diagnostic", rclcpp::ParameterValue(10.0));
  declare_parameter("constants.takeoff.height", rclcpp::ParameterValue(1.5));

  ned_enu_quaternion_first_rotation_ = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                                          Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  ned_enu_quaternion_second_rotation_ = Eigen::Quaterniond(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                                           Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  ned_enu_reflection_xy_              = Eigen::PermutationMatrix<3>(Eigen::Vector3i(1, 0, 2));
  ned_enu_reflection_z_ = Eigen::DiagonalMatrix<double, 3>(1, 1, -1);

  current_reference_.position.x = 0;
  current_reference_.position.y = 0;
  current_reference_.position.z = 0;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  current_reference_.orientation.x = q.getX();
  current_reference_.orientation.y = q.getY();
  current_reference_.orientation.z = q.getZ();
  current_reference_.orientation.w = q.getW();

  have_goal_.data = false;
}
//}

/* ~ApiNode() //{ */
ApiNode::~ApiNode() {
}
//}

/* on_configure() //{ */
CallbackReturn ApiNode::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Configuring");

  getParameters();
  configPubSub();
  configTimers();
  configServices();

  return CallbackReturn::SUCCESS;
}
//}

/* on_activate() //{ */
CallbackReturn ApiNode::on_activate([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Activating");

  pub_vehicle_command_px4_->on_activate();
  pub_position_setpoint_px4_->on_activate();
  pub_offboard_control_mode_px4_->on_activate();
  pub_nav_odometry_->on_activate();
  pub_have_goal_->on_activate();

  is_active_ = true;

  return CallbackReturn::SUCCESS;
}
//}

/* on_deactivate() //{ */
CallbackReturn ApiNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Deactivating");

  pub_vehicle_command_px4_->on_deactivate();
  pub_position_setpoint_px4_->on_deactivate();
  pub_offboard_control_mode_px4_->on_deactivate();
  pub_nav_odometry_->on_deactivate();
  pub_have_goal_->on_deactivate();

  is_active_ = false;

  return CallbackReturn::SUCCESS;
}
//}

/* on_clenaup() //{ */
CallbackReturn ApiNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Cleaning up");

  sub_odometry_px4_.reset();

  pub_vehicle_command_px4_.reset();
  pub_position_setpoint_px4_.reset();
  pub_offboard_control_mode_px4_.reset();
  pub_nav_odometry_.reset();
  pub_have_goal_.reset();

  tmr_pub_offboard_control_mode_px4_.reset();
  tmr_pub_api_diagnostic_.reset();

  return CallbackReturn::SUCCESS;
}
//}

/* on_shutdown() //{ */
CallbackReturn ApiNode::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Shutting down");

  return CallbackReturn::SUCCESS;
}
//}

/* getParameters() //{ */
void ApiNode::getParameters() {
  get_parameter("rate.pub_offboard_control_mode", _rate_pub_offboard_control_mode_px4_);
  get_parameter("rate.pub_api_diagnostic", _rate_pub_api_diagnostic_);
  get_parameter("constants.takeoff.height", _takeoff_height_);
}
//}

/* configPubSub() //{ */
void ApiNode::configPubSub() {
  RCLCPP_INFO(get_logger(), "initPubSub");

  // Pubs and Subs for Px4 topics
  sub_odometry_px4_ = create_subscription<px4_msgs::msg::VehicleOdometry>("vehicle_odometry_px4_in", rclcpp::SensorDataQoS(),
                                                                          std::bind(&ApiNode::subOdometryPx4, this, std::placeholders::_1));

  pub_position_setpoint_px4_     = create_publisher<px4_msgs::msg::TrajectorySetpoint>("position_setpoint_px4_out", 10);
  pub_vehicle_command_px4_       = create_publisher<px4_msgs::msg::VehicleCommand>("vehicle_command_px4_out", 10);
  pub_offboard_control_mode_px4_ = create_publisher<px4_msgs::msg::OffboardControlMode>("offboard_control_mode_px4_out", 10);

  // Pubs and Subs for System topics
  pub_have_goal_    = create_publisher<std_msgs::msg::Bool>("have_goal", 10);
  pub_nav_odometry_ = create_publisher<nav_msgs::msg::Odometry>("odometry", 10);

  sub_goto_          = create_subscription<geometry_msgs::msg::Pose>("goto", 1, std::bind(&ApiNode::subGoto, this, std::placeholders::_1));
  sub_goto_relative_ = create_subscription<geometry_msgs::msg::Pose>("goto_relative", 1, std::bind(&ApiNode::subGotoRelative, this, std::placeholders::_1));
}
//}

/* configTimers() //{ */
void ApiNode::configTimers() {
  RCLCPP_INFO(get_logger(), "initTimers");

  tmr_pub_offboard_control_mode_px4_ = create_wall_timer(std::chrono::duration<double>(1.0 / _rate_pub_offboard_control_mode_px4_),
                                                         std::bind(&ApiNode::tmrPubOffboardControlModePx4, this), nullptr);
  tmr_pub_api_diagnostic_ =
      create_wall_timer(std::chrono::duration<double>(1.0 / _rate_pub_api_diagnostic_), std::bind(&ApiNode::tmrPubApiDiagnostic, this), nullptr);
}
//}

/* configServices() //{ */
void ApiNode::configServices() {
  RCLCPP_INFO(get_logger(), "initServices");

  srv_takeoff_ = create_service<std_srvs::srv::Trigger>("takeoff", std::bind(&ApiNode::srvTakeoff, this, std::placeholders::_1, std::placeholders::_2));
  srv_land_    = create_service<std_srvs::srv::Trigger>("land", std::bind(&ApiNode::srvLand, this, std::placeholders::_1, std::placeholders::_2));
  srv_arm_     = create_service<std_srvs::srv::Trigger>("arm", std::bind(&ApiNode::srvArm, this, std::placeholders::_1, std::placeholders::_2));
  srv_disarm_  = create_service<std_srvs::srv::Trigger>("disarm", std::bind(&ApiNode::srvDisarm, this, std::placeholders::_1, std::placeholders::_2));
}
//}

/* subOdometryPx4() //{ */
void ApiNode::subOdometryPx4(const px4_msgs::msg::VehicleOdometry &msg) {
  if (!is_active_) {
    return;
  }

  nav_msgs::msg::Odometry current_nav_odometry{};
  current_nav_odometry.header.frame_id = "odom";
  current_nav_odometry.header.stamp    = rclcpp::Clock().now();
  current_nav_odometry.child_frame_id  = "fcu";

  Eigen::Vector3d ned_to_enu_tf(msg.position[0], msg.position[1], msg.position[2]);
  ned_to_enu_tf = enuToNed(ned_to_enu_tf);

  current_nav_odometry.pose.pose.position.x = ned_to_enu_tf(0);
  current_nav_odometry.pose.pose.position.y = ned_to_enu_tf(1);
  current_nav_odometry.pose.pose.position.z = ned_to_enu_tf(2);

  Eigen::Quaterniond ned_to_enu_orientation_tf(msg.q[0], msg.q[1], msg.q[2], msg.q[3]);
  ned_to_enu_orientation_tf = enuToNedOrientation(ned_to_enu_orientation_tf);

  current_nav_odometry.pose.pose.orientation.x = ned_to_enu_orientation_tf.x();
  current_nav_odometry.pose.pose.orientation.y = ned_to_enu_orientation_tf.y();
  current_nav_odometry.pose.pose.orientation.z = ned_to_enu_orientation_tf.z();
  current_nav_odometry.pose.pose.orientation.w = ned_to_enu_orientation_tf.w();

  current_nav_odometry.pose.covariance = {msg.position_variance[0],    0, 0, 0, 0, 0, 0, msg.position_variance[1],    0, 0, 0, 0, 0, 0,
                                          msg.position_variance[2],    0, 0, 0, 0, 0, 0, msg.orientation_variance[0], 0, 0, 0, 0, 0, 0,
                                          msg.orientation_variance[1], 0, 0, 0, 0, 0, 0, msg.orientation_variance[2]};

  ned_to_enu_tf(0) = msg.velocity[0];
  ned_to_enu_tf(1) = msg.velocity[1];
  ned_to_enu_tf(2) = msg.velocity[2];
  ned_to_enu_tf    = enuToNed(ned_to_enu_tf);

  current_nav_odometry.twist.twist.linear.x = ned_to_enu_tf(0);
  current_nav_odometry.twist.twist.linear.y = ned_to_enu_tf(1);
  current_nav_odometry.twist.twist.linear.z = ned_to_enu_tf(2);

  ned_to_enu_tf(0) = msg.angular_velocity[0];
  ned_to_enu_tf(1) = msg.angular_velocity[1];
  ned_to_enu_tf(2) = msg.angular_velocity[2];
  ned_to_enu_tf    = enuToNed(ned_to_enu_tf);

  current_nav_odometry.twist.twist.angular.x = ned_to_enu_tf(0);
  current_nav_odometry.twist.twist.angular.y = ned_to_enu_tf(1);
  current_nav_odometry.twist.twist.angular.z = ned_to_enu_tf(2);

  current_nav_odometry.twist.covariance = {msg.velocity_variance[0], 0, 0, 0, 0, 0, 0, msg.velocity_variance[1], 0, 0, 0, 0, 0, 0,
                                           msg.velocity_variance[2], 0, 0, 0, 0, 0, 0, msg.velocity_variance[0], 0, 0, 0, 0, 0, 0,
                                           msg.velocity_variance[1], 0, 0, 0, 0, 0, 0, msg.velocity_variance[2]};

  pub_nav_odometry_->publish(current_nav_odometry);

  if (current_nav_odometry.header.stamp.nanosec - last_nav_odometry_.header.stamp.nanosec >= 0.01) {
    if (first_iteraction_) {
      derivative_position_.position.x = 0;
      derivative_position_.position.y = 0;
      derivative_position_.position.z = 0;
      derivative_position_filtered_   = derivative_position_;
      first_iteraction_               = false;
    } else {
      derivative_position_.position.x = abs((current_nav_odometry.pose.pose.position.x - last_nav_odometry_.pose.pose.position.x) / (0.01));
      derivative_position_.position.y = abs((current_nav_odometry.pose.pose.position.y - last_nav_odometry_.pose.pose.position.y) / (0.01));
      derivative_position_.position.z = abs((current_nav_odometry.pose.pose.position.z - last_nav_odometry_.pose.pose.position.z) / (0.01));

      last_nav_odometry_ = current_nav_odometry;
      received_odometry_ = true;
    }

    double check_x = abs(0.005 * derivative_position_.position.x + (1 - 0.005) * derivative_position_filtered_.position.x);
    double check_y = abs(0.005 * derivative_position_.position.y + (1 - 0.005) * derivative_position_filtered_.position.y);
    double check_z = abs(0.005 * derivative_position_.position.z + (1 - 0.005) * derivative_position_filtered_.position.z);

    if (requested_takeoff_ || requested_land_ || requested_goto_) {
      have_goal_.data = ((check_x > 0.0005) || (check_y > 0.0005) || (check_z > 0.0005) || (abs(current_nav_odometry_.twist.twist.angular.z) > 0.0005)) &&
                        (requested_takeoff_ || requested_land_ || requested_goto_);

      if (last_have_goal_ && !have_goal_.data) {
        requested_takeoff_ = false;
        requested_land_    = false;
        requested_goto_    = false;
      }
      last_have_goal_ = have_goal_.data;
    }
  }
}
//}

/* tmrPubOffboardControlModePx4() //{ */
void ApiNode::tmrPubOffboardControlModePx4() {
  if (!is_active_) {
    return;
  }

  px4_msgs::msg::OffboardControlMode msg{};

  msg.position          = true;
  msg.thrust_and_torque = false;
  msg.velocity          = false;
  msg.acceleration      = false;
  msg.attitude          = false;
  msg.body_rate         = false;
  msg.direct_actuator   = false;
  msg.timestamp         = get_clock()->now().nanoseconds() / 1000;
  pub_offboard_control_mode_px4_->publish(msg);
}
//}

/* pubVehicleCommandPx4() //{ */
void ApiNode::pubVehicleCommandPx4(int command, float param1, float param2, float param3, float param4, float param5, float param6, float param7) {
  if (!is_active_) {
    return;
  }

  px4_msgs::msg::VehicleCommand msg{};
  msg.param1           = param1;
  msg.param2           = param2;
  msg.param3           = param3;
  msg.param4           = param4;
  msg.param5           = param5;
  msg.param6           = param6;
  msg.param7           = param7;
  msg.command          = command;
  msg.target_system    = 1;
  msg.target_component = 1;
  msg.source_system    = 1;
  msg.source_component = 1;
  msg.from_external    = true;
  msg.timestamp        = get_clock()->now().nanoseconds() / 1000;
  pub_vehicle_command_px4_->publish(msg);
}
//}

/* tmrPubApiDiagnostic() //{ */
void ApiNode::tmrPubApiDiagnostic() {
  if (!is_active_) {
    return;
  }

  pub_have_goal_->publish(have_goal_);
}
//}

/* subGoto() //{ */
void ApiNode::subGoto(const geometry_msgs::msg::Pose &msg) {
  if (!is_active_) {
    return;
  }

  if (!is_flying_) {
    return;
  }

  if (requested_takeoff_ || requested_land_) {
    return;
  }

  requested_goto_ = true;

  px4_msgs::msg::TrajectorySetpoint position_setpoint{};

  // Position values are assigned in this order due to the ENU to NED frame conversion
  Eigen::Vector3d enu_to_ned_tf(msg.position.x, msg.position.y, msg.position.z);
  enu_to_ned_tf = enuToNed(enu_to_ned_tf);

  position_setpoint.position[0] = enu_to_ned_tf(0);
  position_setpoint.position[1] = enu_to_ned_tf(1);
  position_setpoint.position[2] = enu_to_ned_tf(2);

  Eigen::Quaterniond enu_to_ned_orientation_tf(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  enu_to_ned_orientation_tf = enuToNedOrientation(enu_to_ned_orientation_tf);

  tf2::Quaternion q(enu_to_ned_orientation_tf.x(), enu_to_ned_orientation_tf.y(), enu_to_ned_orientation_tf.z(), enu_to_ned_orientation_tf.w());

  tf2::Matrix3x3 m(q);
  double         roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  position_setpoint.yaw = yaw;

  position_setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  pub_position_setpoint_px4_->publish(position_setpoint);

  current_reference_ = msg;
}
//}

/* subGotoRelative() //{ */
void ApiNode::subGotoRelative(const geometry_msgs::msg::Pose &msg) {
  if (!is_active_) {
    return;
  }

  if (!is_flying_) {
    return;
  }

  if (requested_takeoff_ || requested_land_) {
    return;
  }

  requested_goto_ = true;

  current_reference_.position.x += msg.position.x;
  current_reference_.position.y += msg.position.y;
  current_reference_.position.z += msg.position.z;

  current_reference_.orientation.x += msg.orientation.x;
  current_reference_.orientation.y += msg.orientation.y;
  current_reference_.orientation.z += msg.orientation.z;
  current_reference_.orientation.w += msg.orientation.w;

  px4_msgs::msg::TrajectorySetpoint position_setpoint{};

  // Position values are assigned in this order due to the ENU to NED frame conversion
  Eigen::Vector3d enu_to_ned_tf(current_reference_.position.x, current_reference_.position.y, current_reference_.position.z);
  enu_to_ned_tf = enuToNed(enu_to_ned_tf);

  position_setpoint.position[0] = enu_to_ned_tf(0);
  position_setpoint.position[1] = enu_to_ned_tf(1);
  position_setpoint.position[2] = enu_to_ned_tf(2);

  Eigen::Quaterniond enu_to_ned_orientation_tf(current_reference_.orientation.w, current_reference_.orientation.x, current_reference_.orientation.y,
                                               current_reference_.orientation.z);
  enu_to_ned_orientation_tf = enuToNedOrientation(enu_to_ned_orientation_tf);

  tf2::Quaternion q(enu_to_ned_orientation_tf.x(), enu_to_ned_orientation_tf.y(), enu_to_ned_orientation_tf.z(), enu_to_ned_orientation_tf.w());

  tf2::Matrix3x3 m(q);
  double         roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  position_setpoint.yaw = yaw;

  position_setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  pub_position_setpoint_px4_->publish(position_setpoint);
}
//}

/* srvTakeoff() //{ */
void ApiNode::srvTakeoff([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Response>      response) {
  if (!is_active_) {
    return;
  }

  if (!received_odometry_) {
    response->success = false;
    response->message = "arm requested failed, api dont received odometry";
    return;
  }

  if (is_flying_) {
    response->success = false;
    response->message = "takeoff requested failed, uav is already in flight";
    return;
  }

  requested_takeoff_ = true;

  px4_msgs::msg::TrajectorySetpoint position_setpoint{};

  // Position values are assigned in this order due to the ENU to NED frame conversion
  current_reference_            = last_nav_odometry_.pose.pose;
  current_reference_.position.z = _takeoff_height_;

  Eigen::Vector3d enu_to_ned_tf(current_reference_.position.x, current_reference_.position.y, current_reference_.position.z);
  enu_to_ned_tf = enuToNed(enu_to_ned_tf);

  position_setpoint.position[0] = enu_to_ned_tf(0);
  position_setpoint.position[1] = enu_to_ned_tf(1);
  position_setpoint.position[2] = enu_to_ned_tf(2);

  Eigen::Quaterniond enu_to_ned_orientation_tf(current_reference_.orientation.w, current_reference_.orientation.x, current_reference_.orientation.y,
                                               current_reference_.orientation.z);
  enu_to_ned_orientation_tf = enuToNedOrientation(enu_to_ned_orientation_tf);

  tf2::Quaternion q(enu_to_ned_orientation_tf.x(), enu_to_ned_orientation_tf.y(), enu_to_ned_orientation_tf.z(), enu_to_ned_orientation_tf.w());

  tf2::Matrix3x3 m(q);
  double         roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  position_setpoint.yaw = yaw;

  position_setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  pub_position_setpoint_px4_->publish(position_setpoint);

  is_flying_        = true;
  response->success = true;
  response->message = "takeoff requested success";
}
//}

/* srvLand() //{ */
void ApiNode::srvLand([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                      [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Response>      response) {
  if (!is_active_) {
    return;
  }

  if (!is_flying_) {
    response->success = false;
    response->message = "land requested failed, uav is already landed";
    return;
  }

  requested_land_ = true;

  current_reference_.position.z = 0;
  pubVehicleCommandPx4(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);

  is_flying_        = false;
  response->success = true;
  response->message = "land requested success";
}
//}

/* srvArm() //{ */
void ApiNode::srvArm([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                     [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Response>      response) {
  if (!is_active_) {
    return;
  }

  if (is_flying_) {
    response->success = false;
    response->message = "arm requested failed, uav is already in flight";
    return;
  }

  pubVehicleCommandPx4(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
  pubVehicleCommandPx4(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

  response->success = true;
  response->message = "arm requested success";
}
//}

/* srvDisarm() //{ */
void ApiNode::srvDisarm([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Response>      response) {
  if (!is_active_) {
    return;
  }

  if (is_flying_) {
    response->success = false;
    response->message = "disarm requested failed, uav is already in flight";
    return;
  }

  pubVehicleCommandPx4(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

  response->success = true;
  response->message = "disarm requested success";
}
//}

/* enuToNed() //{ */
Eigen::Vector3d ApiNode::enuToNed(Eigen::Vector3d p) {
  return ned_enu_reflection_xy_ * (ned_enu_reflection_z_ * p);
}
//}

/* enuToNedOrientation() //{ */
Eigen::Quaterniond ApiNode::enuToNedOrientation(Eigen::Quaterniond q) {
  return (ned_enu_quaternion_first_rotation_ * q) * ned_enu_quaternion_second_rotation_;
}
//}
}  // namespace laser_uav_px4_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(laser_uav_px4_api::ApiNode)
