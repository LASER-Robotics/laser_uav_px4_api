#ifndef LASER_UAV_API_PX4__API_NODE_HPP
#define LASER_UAV_API_PX4__API_NODE_HPP

#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using VehicleCommandPx4 = px4_msgs::msg::VehicleCommand;
using OffboardControlModePx4 = px4_msgs::msg::OffboardControlMode;
using VehicleControlModePx4 = px4_msgs::msg::VehicleControlMode;
using TorqueSetpointPx4 = px4_msgs::msg::VehicleTorqueSetpoint;
using ThrustSetpointPx4 = px4_msgs::msg::VehicleThrustSetpoint;
using TrajectorySetpointPx4 = px4_msgs::msg::TrajectorySetpoint;
using VehicleOdometryPx4 = px4_msgs::msg::VehicleOdometry;

using Trigger = std_srvs::srv::Trigger;
using NavOdometry = nav_msgs::msg::Odometry;
using GeometryPoint = geometry_msgs::msg::Point;

namespace laser_uav_px4_api
{
class ApiNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ApiNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~ApiNode() override;

private:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &);

  CallbackReturn on_activate(const rclcpp_lifecycle::State & state);

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state);

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);

  rclcpp::CallbackGroup::SharedPtr callback_group_;

  void getParameters();
  void configPubSub();
  void configTimers();
  void configServices();

  rclcpp::Subscription<VehicleControlModePx4>::ConstSharedPtr sub_vehicle_control_mode_px4_;
  void                                                        vehicleControlModePx4Received(
    const VehicleControlModePx4 & msg);

  rclcpp_lifecycle::LifecyclePublisher<NavOdometry>::SharedPtr pub_nav_odometry_;
  rclcpp::Subscription<VehicleOdometryPx4>::ConstSharedPtr sub_vehicle_odometry_px4_;
  void                                                     vehicleOdometryPx4Received(
    const VehicleOdometryPx4 & msg);

  /* rclcpp::Subscription<TorqueAndThrustSetpoint>::ConstSharedPtr sub_torque_and_thrust_setpoint_; */
  /* void                                                          torqueAndThrustSetpointReceived(const TorqueAndThrustSetpoint& msg); */

  rclcpp::Subscription<GeometryPoint>::ConstSharedPtr sub_position_setpoint_;
  void                                                          positionSetpointReceived(
    const GeometryPoint & msg);

  rclcpp_lifecycle::LifecyclePublisher<VehicleCommandPx4>::SharedPtr pub_vehicle_command_px4_;
  void                                                               pubVehicleCommandPx4(
    int command, float param1 = 0.0, float param2 = 0.0);

  rclcpp_lifecycle::LifecyclePublisher<TorqueSetpointPx4>::SharedPtr pub_torque_setpoint_px4_;
  rclcpp_lifecycle::LifecyclePublisher<ThrustSetpointPx4>::SharedPtr pub_thrust_setpoint_px4_;
  void                                                               pubTorqueAndThrustSetpointPx4();

  rclcpp_lifecycle::LifecyclePublisher<TrajectorySetpointPx4>::SharedPtr pub_position_setpoint_px4_;

  double rate_send_offboard_control_mode_px4_;
  rclcpp_lifecycle::LifecyclePublisher<OffboardControlModePx4>::SharedPtr
    pub_offboard_control_mode_px4_;
  rclcpp::TimerBase::SharedPtr timer_send_offboard_control_mode_px4_;
  void
  sendOffboardControlModePx4Callback();

  rclcpp::Service<Trigger>::SharedPtr srv_arm_;
  void                                arm(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);

  rclcpp::Service<Trigger>::SharedPtr srv_disarm_;
  void                                disarm(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);

  std::string setpoint_control_mode_px4_;

  bool is_active_{false};
};
}  // namespace laser_uav_api_px4

#endif
