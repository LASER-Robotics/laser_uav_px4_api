#ifndef LASER_UAV_API_PX4__API_NODE_HPP
#define LASER_UAV_API_PX4__API_NODE_HPP

#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/takeoff_status.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace laser_uav_px4_api
{
class ApiNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit ApiNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  ~ApiNode() override;

private:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &);

  CallbackReturn on_activate(const rclcpp_lifecycle::State &state);

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);

  rclcpp::CallbackGroup::SharedPtr callback_group_;

  void getParameters();
  void configPubSub();
  void configTimers();
  void configServices();

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::ConstSharedPtr     sub_odometry_px4_;
  void                                                                     subOdometryPx4(const px4_msgs::msg::VehicleOdometry &msg);
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub_nav_odometry_;

  rclcpp::Subscription<px4_msgs::msg::TakeoffStatus>::SharedPtr sub_takeoff_status_;
  void                                                          subTakeoffStatus(const px4_msgs::msg::TakeoffStatus &msg);

  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_vehicle_command_px4_;
  void pubVehicleCommandPx4(int command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0, float param5 = 0.0, float param6 = 0.0,
                            float param7 = 0.0);

  double                                                                              _rate_pub_offboard_control_mode_px4_;
  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offboard_control_mode_px4_;
  rclcpp::TimerBase::SharedPtr                                                        tmr_pub_offboard_control_mode_px4_;
  void                                                                                tmrPubOffboardControlModePx4();

  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_position_setpoint_px4_;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_goto_;
  void                                                      subGoto(const geometry_msgs::msg::Pose &msg);

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_goto_relative_;
  void                                                      subGotoRelative(const geometry_msgs::msg::Pose &msg);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_takeoff_;
  void srvTakeoff(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_land_;
  void srvLand(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_arm_;
  void srvArm(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_disarm_;
  void srvDisarm(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  double                       _rate_pub_api_diagnostic_;
  rclcpp::TimerBase::SharedPtr tmr_pub_api_diagnostic_;
  void                         tmrPubApiDiagnostic();

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr pub_have_goal_;
  std_msgs::msg::Bool                                                  have_goal_;

  double _takeoff_height_;

  geometry_msgs::msg::Pose current_reference_;
  geometry_msgs::msg::Pose derivative_position_;
  geometry_msgs::msg::Pose derivative_position_filtered_;
  nav_msgs::msg::Odometry  current_nav_odometry_;
  nav_msgs::msg::Odometry  last_nav_odometry_;

  bool first_iteraction_{true};
  bool is_flying_{false};
  bool last_have_goal_{false};
  bool requested_goto_{false};
  bool requested_takeoff_{false};
  bool requested_land_{false};
  bool is_active_{false};
};
}  // namespace laser_uav_px4_api

#endif
