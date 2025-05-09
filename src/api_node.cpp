#include "laser_uav_px4_api/api_node.hpp"

namespace laser_uav_px4_api
{
/* ApiNode() //{ */
ApiNode::ApiNode(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("api_node", "", options) {
  RCLCPP_INFO(get_logger(), "Creating");

  declare_parameter("rate.pub_offboard_control_mode", rclcpp::ParameterValue(100.0));
  declare_parameter("rate.pub_api_diagnostic", rclcpp::ParameterValue(10.0));
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
  pub_attitude_rates_setpoint_px4_->on_activate();
  pub_offboard_control_mode_px4_->on_activate();
  pub_nav_odometry_->on_activate();

  is_active_ = true;

  return CallbackReturn::SUCCESS;
}
//}

/* on_deactivate() //{ */
CallbackReturn ApiNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Deactivating");

  pub_attitude_rates_setpoint_px4_->on_deactivate();
  pub_offboard_control_mode_px4_->on_deactivate();
  pub_nav_odometry_->on_deactivate();

  is_active_ = false;

  return CallbackReturn::SUCCESS;
}
//}

/* on_clenaup() //{ */
CallbackReturn ApiNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Cleaning up");

  sub_odometry_px4_.reset();
  sub_attitude_rates_and_thrust_reference_.reset();

  pub_attitude_rates_setpoint_px4_.reset();
  pub_offboard_control_mode_px4_.reset();
  pub_nav_odometry_.reset();

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
}
//}

/* configPubSub() //{ */
void ApiNode::configPubSub() {
  RCLCPP_INFO(get_logger(), "initPubSub");

  // Pubs and Subs for Px4 topics
  sub_odometry_px4_ = create_subscription<px4_msgs::msg::VehicleOdometry>("vehicle_odometry_px4_in", rclcpp::SensorDataQoS(),
                                                                          std::bind(&ApiNode::subOdometryPx4, this, std::placeholders::_1));

  pub_attitude_rates_setpoint_px4_ = create_publisher<px4_msgs::msg::VehicleRatesSetpoint>("attitude_rates_setpoint_px4_out", 10);
  pub_vehicle_command_px4_         = create_publisher<px4_msgs::msg::VehicleCommand>("vehicle_command_px4_out", 10);
  pub_offboard_control_mode_px4_   = create_publisher<px4_msgs::msg::OffboardControlMode>("offboard_control_mode_px4_out", 10);

  // Pubs and Subs for System topics
  pub_nav_odometry_ = create_publisher<nav_msgs::msg::Odometry>("odometry", 10);

  sub_attitude_rates_and_thrust_reference_ = create_subscription<laser_msgs::msg::AttitudeRatesAndThrust>(
      "attitude_rates_thrust_in", 1, std::bind(&ApiNode::subAttitudeRatesAndThrustReference, this, std::placeholders::_1));
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

  srv_arm_    = create_service<std_srvs::srv::Trigger>("arm", std::bind(&ApiNode::srvArm, this, std::placeholders::_1, std::placeholders::_2));
  srv_disarm_ = create_service<std_srvs::srv::Trigger>("disarm", std::bind(&ApiNode::srvDisarm, this, std::placeholders::_1, std::placeholders::_2));
}
//}

/* subOdometryPx4() //{ */
void ApiNode::subOdometryPx4(const px4_msgs::msg::VehicleOdometry &msg) {
  if (!is_active_) {
    return;
  }

  nav_msgs::msg::Odometry current_nav_odometry{};
  current_nav_odometry.header.frame_id = "fmu";
  current_nav_odometry.header.stamp    = rclcpp::Clock().now();
  current_nav_odometry.child_frame_id  = "center_body";

  current_nav_odometry.pose.pose.position.x = msg.position[0];
  current_nav_odometry.pose.pose.position.y = msg.position[1];
  current_nav_odometry.pose.pose.position.z = -msg.position[2];

  current_nav_odometry.pose.pose.orientation.x = msg.q[0];
  current_nav_odometry.pose.pose.orientation.y = msg.q[1];
  current_nav_odometry.pose.pose.orientation.z = msg.q[2];
  current_nav_odometry.pose.pose.orientation.w = msg.q[3];

  current_nav_odometry.pose.covariance = {msg.position_variance[0],    0, 0, 0, 0, 0, 0, msg.position_variance[1],    0, 0, 0, 0, 0, 0,
                                          msg.position_variance[2],    0, 0, 0, 0, 0, 0, msg.orientation_variance[0], 0, 0, 0, 0, 0, 0,
                                          msg.orientation_variance[1], 0, 0, 0, 0, 0, 0, msg.orientation_variance[2]};

  current_nav_odometry.twist.twist.linear.x = msg.velocity[0];
  current_nav_odometry.twist.twist.linear.y = msg.velocity[1];
  current_nav_odometry.twist.twist.linear.z = msg.velocity[2];

  current_nav_odometry.twist.twist.angular.x = msg.angular_velocity[0];
  current_nav_odometry.twist.twist.angular.y = msg.angular_velocity[1];
  current_nav_odometry.twist.twist.angular.z = msg.angular_velocity[2];

  current_nav_odometry.twist.covariance = {msg.velocity_variance[0], 0, 0, 0, 0, 0, 0, msg.velocity_variance[1], 0, 0, 0, 0, 0, 0,
                                           msg.velocity_variance[2], 0, 0, 0, 0, 0, 0, msg.velocity_variance[0], 0, 0, 0, 0, 0, 0,
                                           msg.velocity_variance[1], 0, 0, 0, 0, 0, 0, msg.velocity_variance[2]};

  pub_nav_odometry_->publish(current_nav_odometry);
}
//}

/* tmrPubOffboardControlModePx4() //{ */
void ApiNode::tmrPubOffboardControlModePx4() {
  if (!is_active_) {
    return;
  }

  px4_msgs::msg::OffboardControlMode msg{};

  msg.position          = false;
  msg.velocity          = false;
  msg.acceleration      = false;
  msg.attitude          = false;
  msg.body_rate         = true;
  msg.thrust_and_torque = false;
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
}
//}

/* subAttitudeRatesAndThrustReference() //{ */
void ApiNode::subAttitudeRatesAndThrustReference(const laser_msgs::msg::AttitudeRatesAndThrust &msg) {
  if (!is_active_) {
    return;
  }

  px4_msgs::msg::VehicleRatesSetpoint attitude_rates_setpoint{};

  // Frame Transformation FLU to FRD
  Eigen::Matrix4d rot_x = Eigen::Matrix4d::Identity();
  rot_x(1, 1)           = std::cos(M_PI);
  rot_x(1, 2)           = -std::sin(M_PI);
  rot_x(2, 1)           = std::sin(M_PI);
  rot_x(2, 2)           = std::cos(M_PI);

  Eigen::Vector4d aux;
  aux << msg.roll_rate, msg.pitch_rate, msg.yaw_rate, 1;

  aux = rot_x * aux;

  attitude_rates_setpoint.roll  = aux(0);
  attitude_rates_setpoint.pitch = aux(1);
  attitude_rates_setpoint.yaw   = aux(2);

  attitude_rates_setpoint.thrust_body[0] = 0;
  attitude_rates_setpoint.thrust_body[1] = 0;
  attitude_rates_setpoint.thrust_body[2] = -msg.total_thrust_normalized;

  attitude_rates_setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000;

  pub_attitude_rates_setpoint_px4_->publish(attitude_rates_setpoint);
}
//}

/* srvArm() //{ */
void ApiNode::srvArm([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                     [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Response>      response) {
  if (!is_active_) {
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

  pubVehicleCommandPx4(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

  response->success = true;
  response->message = "disarm requested success";
}
//}
}  // namespace laser_uav_px4_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(laser_uav_px4_api::ApiNode)
