#include "laser_uav_px4_api/api_node.hpp"

namespace laser_uav_px4_api
{
ApiNode::ApiNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("api_px4_node", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating");

  declare_parameter("setpoint_control_mode", rclcpp::ParameterValue(std::string("Ps")));

  declare_parameter("rate.send_offboard_control_mode", rclcpp::ParameterValue(100.0));
}

ApiNode::~ApiNode()
{
}

CallbackReturn ApiNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  getParameters();
  configPubSub();
  configTimers();
  configServices();

  return CallbackReturn::SUCCESS;
}

CallbackReturn ApiNode::on_activate([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating");

  pub_vehicle_command_px4_->on_activate();

  if (setpoint_control_mode_px4_ == "TTs") {
    pub_torque_setpoint_px4_->on_activate();
    pub_thrust_setpoint_px4_->on_activate();
  } else if (setpoint_control_mode_px4_ == "Ps") {
    pub_position_setpoint_px4_->on_activate();
  }

  pub_offboard_control_mode_px4_->on_activate();
  pub_nav_odometry_->on_activate();

  is_active_ = true;

  return CallbackReturn::SUCCESS;
}

CallbackReturn ApiNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  pub_vehicle_command_px4_->on_deactivate();

  if (setpoint_control_mode_px4_ == "TTs") {
    pub_torque_setpoint_px4_->on_deactivate();
    pub_thrust_setpoint_px4_->on_deactivate();
  } else if (setpoint_control_mode_px4_ == "Ps") {
    pub_position_setpoint_px4_->on_deactivate();
  }

  pub_torque_setpoint_px4_->on_deactivate();
  pub_thrust_setpoint_px4_->on_deactivate();
  pub_offboard_control_mode_px4_->on_deactivate();
  pub_nav_odometry_->on_deactivate();

  is_active_ = false;

  return CallbackReturn::SUCCESS;
}

CallbackReturn ApiNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  /* sub_vehicle_control_mode_px4_.reset(); */
  sub_vehicle_odometry_px4_.reset();

  pub_vehicle_command_px4_.reset();

  if (setpoint_control_mode_px4_ == "TTs") {
    pub_torque_setpoint_px4_.reset();
    pub_thrust_setpoint_px4_.reset();
  } else if (setpoint_control_mode_px4_ == "Ps") {
    pub_position_setpoint_px4_.reset();
  }

  pub_offboard_control_mode_px4_.reset();
  pub_nav_odometry_.reset();

  timer_send_offboard_control_mode_px4_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn ApiNode::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting down");

  return CallbackReturn::SUCCESS;
}

void ApiNode::getParameters()
{
  get_parameter("setpoint_control_mode", setpoint_control_mode_px4_);
  get_parameter("rate.send_offboard_control_mode", rate_send_offboard_control_mode_px4_);
}

void ApiNode::configPubSub()
{
  RCLCPP_INFO(get_logger(), "initPubSub");

  // Pubs and Subs for Px4 topics
  sub_vehicle_control_mode_px4_ = create_subscription<VehicleControlModePx4>(
    "vehicle_control_mode_px4_in", rclcpp::SensorDataQoS(),
    std::bind(&ApiNode::vehicleControlModePx4Received, this, std::placeholders::_1));

  sub_vehicle_odometry_px4_ =
    create_subscription<VehicleOdometryPx4>(
    "vehicle_odometry_px4_in", rclcpp::SensorDataQoS(),
    std::bind(&ApiNode::vehicleOdometryPx4Received, this, std::placeholders::_1));

  if (setpoint_control_mode_px4_ == "TTs") {
    pub_torque_setpoint_px4_ = create_publisher<TorqueSetpointPx4>("torque_setpoint_px4_out", 10);

    pub_thrust_setpoint_px4_ = create_publisher<ThrustSetpointPx4>("thrust_setpoint_px4_out", 10);

    /* sub_torque_and_thrust_setpoint_ = create_subscription<>("fmu_torque_thrust_setpoint_in", 1, std::bind(&ApiNode::torqueAndThrustSetpointReceived, this, _1)); */
  } else if (setpoint_control_mode_px4_ == "Ps") {
    pub_position_setpoint_px4_ = create_publisher<TrajectorySetpointPx4>(
      "trajectory_setpoint_px4_out",
      10);

    sub_position_setpoint_ = create_subscription<GeometryPoint>(
      "position_setpoint_in", 1, std::bind(
        &ApiNode::positionSetpointReceived, this,
        std::placeholders::_1));
  }

  pub_vehicle_command_px4_ = create_publisher<VehicleCommandPx4>("vehicle_command_px4_out", 10);

  pub_offboard_control_mode_px4_ = create_publisher<OffboardControlModePx4>(
    "offboard_control_mode_px4_out", 10);

  // Pubs and Subs for System topics
  pub_nav_odometry_ = create_publisher<NavOdometry>("fmu_nav_odometry_out", 10);
}

void ApiNode::configTimers()
{
  RCLCPP_INFO(get_logger(), "initTimers");

  timer_send_offboard_control_mode_px4_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / 100 /*rate_send_offboard_control_mode_px4_*/),
    std::bind(&ApiNode::sendOffboardControlModePx4Callback, this), nullptr);
}

void ApiNode::configServices()
{
  RCLCPP_INFO(get_logger(), "initServices");

  srv_arm_ =
    create_service<Trigger>(
    "arm",
    std::bind(&ApiNode::arm, this, std::placeholders::_1, std::placeholders::_2));

  srv_disarm_ =
    create_service<Trigger>(
    "disarm",
    std::bind(&ApiNode::disarm, this, std::placeholders::_1, std::placeholders::_2));
}

void ApiNode::vehicleControlModePx4Received(const VehicleControlModePx4 & msg)
{
  if (!is_active_) {
    return;
  }

  if (!msg.flag_control_offboard_enabled) {
  }
}

void ApiNode::vehicleOdometryPx4Received(const VehicleOdometryPx4 & msg)
{
  if (!is_active_) {
    return;
  }

  NavOdometry nav_odometry_msg{};
  nav_odometry_msg.header.frame_id = "fmu";
  nav_odometry_msg.header.stamp = rclcpp::Time();
  nav_odometry_msg.child_frame_id = "center_body";

  nav_odometry_msg.pose.pose.position.x = msg.position[0];
  nav_odometry_msg.pose.pose.position.y = msg.position[1];
  nav_odometry_msg.pose.pose.position.z = msg.position[2];

  nav_odometry_msg.pose.pose.orientation.x = msg.q[0];
  nav_odometry_msg.pose.pose.orientation.y = msg.q[1];
  nav_odometry_msg.pose.pose.orientation.z = msg.q[2];
  nav_odometry_msg.pose.pose.orientation.w = msg.q[3];

  nav_odometry_msg.pose.covariance =
  {msg.position_variance[0], 0, 0, 0, 0, 0, 0, msg.position_variance[1], 0, 0, 0, 0, 0, 0,
    msg.position_variance[2], 0, 0, 0, 0, 0, 0, msg.orientation_variance[0], 0, 0, 0, 0, 0, 0,
    msg.orientation_variance[1], 0, 0, 0, 0, 0, 0, msg.orientation_variance[2]};

  nav_odometry_msg.twist.twist.linear.x = msg.velocity[0];
  nav_odometry_msg.twist.twist.linear.y = msg.velocity[1];
  nav_odometry_msg.twist.twist.linear.z = msg.velocity[2];

  nav_odometry_msg.twist.twist.angular.x = msg.angular_velocity[0];
  nav_odometry_msg.twist.twist.angular.y = msg.angular_velocity[1];
  nav_odometry_msg.twist.twist.angular.z = msg.angular_velocity[2];

  nav_odometry_msg.twist.covariance =
  {msg.velocity_variance[0], 0, 0, 0, 0, 0, 0, msg.velocity_variance[1], 0, 0, 0, 0, 0, 0,
    msg.velocity_variance[2], 0, 0, 0, 0, 0, 0, msg.velocity_variance[0], 0, 0, 0, 0, 0, 0,
    msg.velocity_variance[1], 0, 0, 0, 0, 0, 0, msg.velocity_variance[2]};

  pub_nav_odometry_->publish(nav_odometry_msg);
}

/* void ApiNode::torqueAndThrustSetpointReceived(const TorqueAndThrustSetpoint& msg) { */
/* } */

void ApiNode::positionSetpointReceived(const GeometryPoint & msg)
{
  if (!is_active_) {
    return;
  }

  TrajectorySetpointPx4 trajectory_px4_msg{};
  trajectory_px4_msg.position[0] = msg.x;
  trajectory_px4_msg.position[1] = -msg.y;
  trajectory_px4_msg.position[2] = -msg.z;

  trajectory_px4_msg.velocity[0] = 5.0;
  trajectory_px4_msg.velocity[1] = 5.0;
  trajectory_px4_msg.velocity[2] = 3.0;

  trajectory_px4_msg.acceleration[0] = 2.0;
  trajectory_px4_msg.acceleration[1] = 2.0;
  trajectory_px4_msg.acceleration[2] = 1.0;

  trajectory_px4_msg.yaw = 0;
  trajectory_px4_msg.yawspeed = 0;
  
  trajectory_px4_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  pub_position_setpoint_px4_->publish(trajectory_px4_msg);
}

void ApiNode::sendOffboardControlModePx4Callback()
{
  if (!is_active_) {
    return;
  }

  OffboardControlModePx4 msg{};

  if (setpoint_control_mode_px4_ == "TTs") {
    msg.position = false;
    msg.thrust_and_torque = true;
  } else if (setpoint_control_mode_px4_ == "Ps") {
    msg.position = true;
    msg.thrust_and_torque = false;
  }

  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.direct_actuator = false;
  msg.timestamp = get_clock()->now().nanoseconds() / 1000;
  pub_offboard_control_mode_px4_->publish(msg);
}

void ApiNode::pubVehicleCommandPx4(int command, float param1, float param2)
{
  if (!is_active_) {
    return;
  }

  VehicleCommandPx4 msg{};
  msg.param1 = param1;
  msg.param2 = param2;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = get_clock()->now().nanoseconds() / 1000;
  pub_vehicle_command_px4_->publish(msg);
}

void ApiNode::arm(
  [[maybe_unused]] const std::shared_ptr<Trigger::Request> request,
  [[maybe_unused]] std::shared_ptr<Trigger::Response> response)
{
  if (!is_active_) {
    return;
  }

  pubVehicleCommandPx4(VehicleCommandPx4::VEHICLE_CMD_DO_SET_MODE, 1, 6);

  pubVehicleCommandPx4(VehicleCommandPx4::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

  response->success = true;
  response->message = "arming";
}

void ApiNode::disarm(
  [[maybe_unused]] const std::shared_ptr<Trigger::Request> request,
  [[maybe_unused]] std::shared_ptr<Trigger::Response> response)
{
  if (!is_active_) {
    return;
  }

  pubVehicleCommandPx4(VehicleCommandPx4::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

  response->success = true;
  response->message = "disarming";
}
}  // namespace laser_uav_px4_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(laser_uav_px4_api::ApiNode)
