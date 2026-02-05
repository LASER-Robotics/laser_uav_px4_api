#ifndef LASER_UAV_HW_API__AP_API_NODE_HPP
#define LASER_UAV_HW_API__AP_API_NODE_HPP

#include <Eigen/Dense>
#include <cstdlib>
#include <regex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <laser_msgs/msg/attitude_rates_and_thrust.hpp>
#include <laser_msgs/msg/api_px4_diagnostics.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <ardupilot_msgs/msg/attitude_target.hpp>
#include <ardupilot_msgs/msg/status.hpp>
#include <ardupilot_msgs/srv/arm_motors.hpp>
#include <ardupilot_msgs/srv/mode_switch.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace laser_uav_hw_api
{
class ApApiNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit ApApiNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  ~ApApiNode() override;

private:
  Eigen::Vector3d    enuToNed(Eigen::Vector3d p);
  Eigen::Vector3d    frdToFlu(Eigen::Vector3d p);
  Eigen::Quaterniond enuToNedOrientation(Eigen::Quaterniond q);

  CallbackReturn on_configure(const rclcpp_lifecycle::State &);

  CallbackReturn on_activate(const rclcpp_lifecycle::State &state);

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);

  rclcpp::CallbackGroup::SharedPtr callback_group_;

  void getParameters();
  void configPubSub();
  void configTimers();
  void configServices();

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::ConstSharedPtr    sub_estimated_pose_ap_;
  void                                                                     subEstimatedPoseAp(const geometry_msgs::msg::PoseStamped &msg);
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::ConstSharedPtr   sub_estimated_twist_ap_;
  void                                                                     subEstimatedTwistAp(const geometry_msgs::msg::TwistStamped &msg);
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub_nav_odometry_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr            sub_imu_ap_;
  void                                                                   subImuAp(const sensor_msgs::msg::Imu &msg);
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;

  rclcpp::Subscription<ardupilot_msgs::msg::Status>::ConstSharedPtr sub_vehicle_status_ap_;
  void                                                              subVehicleStatusAp(const ardupilot_msgs::msg::Status &msg);

  /* rclcpp::Subscription<px4_msgs::msg::EscStatus>::ConstSharedPtr               sub_esc_status_px4_; */
  /* void                                                                         subEscStatusPx4(const px4_msgs::msg::EscStatus &msg); */
  /* rclcpp_lifecycle::LifecyclePublisher<laser_msgs::msg::MotorSpeed>::SharedPtr pub_motor_speed_estimation_; */

  /* rclcpp::Subscription<laser_msgs::msg::MotorSpeed>::ConstSharedPtr              sub_motor_speed_reference_; */
  /* void                                                                           subMotorSpeedReference(const laser_msgs::msg::MotorSpeed &msg); */
  /* rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::ActuatorMotors>::SharedPtr pub_motor_speed_reference_px4_; */
  /* rclcpp::TimerBase::SharedPtr                                                   tmr_pub_motor_speed_reference_px4_; */
  /* void                                                                           tmrPubMotorSpeedReferencePx4(); */

  /* rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_vehicle_command_px4_; */
  /* void pubVehicleCommandPx4(int command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0, float param5 = 0.0, float param6 =
   * 0.0, */
  /*                           float param7 = 0.0); */

  rclcpp::Client<ardupilot_msgs::srv::ArmMotors>::SharedPtr  clt_arm_motors_ap_;
  rclcpp::Client<ardupilot_msgs::srv::ModeSwitch>::SharedPtr clt_mode_switch_ap_;

  /* double                                                                              _rate_pub_offboard_control_mode_px4_; */
  /* rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offboard_control_mode_px4_; */
  /* rclcpp::TimerBase::SharedPtr                                                        tmr_pub_offboard_control_mode_px4_; */
  /* void                                                                                tmrPubOffboardControlModePx4(); */

  rclcpp_lifecycle::LifecyclePublisher<ardupilot_msgs::msg::AttitudeTarget>::SharedPtr pub_attitude_rates_reference_ap_;
  rclcpp::Subscription<laser_msgs::msg::AttitudeRatesAndThrust>::SharedPtr             sub_attitude_rates_and_thrust_reference_;
  void subAttitudeRatesAndThrustReference(const laser_msgs::msg::AttitudeRatesAndThrust &msg);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_arm_;
  void srvArm(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_disarm_;
  void srvDisarm(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  double                                                                           _rate_pub_api_diagnostics_;
  rclcpp_lifecycle::LifecyclePublisher<laser_msgs::msg::ApiPx4Diagnostics>::SharedPtr pub_api_diagnostics_;
  rclcpp::TimerBase::SharedPtr                                                     tmr_pub_api_diagnostics_;
  void                                                                             tmrPubApiDiagnostics();

  Eigen::Quaterniond               ned_enu_quaternion_rotation_;
  Eigen::Quaterniond               frd_flu_rotation_;
  Eigen::Affine3d                  frd_flu_affine_;
  Eigen::PermutationMatrix<3>      ned_enu_reflection_xy_;
  Eigen::DiagonalMatrix<double, 3> ned_enu_reflection_z_;
  geometry_msgs::msg::TwistStamped twist_stamped_ap_;
  sensor_msgs::msg::Imu            imu_;
  laser_msgs::msg::ApiPx4Diagnostics  api_diagnostics_;
  /* px4_msgs::msg::ActuatorMotors      actuator_motors_reference_; */

  /* int target_system_; */

  /* std::string _control_input_mode_; */

  bool real_uav_{false};
  bool offboard_is_enabled_{false};
  /* bool fw_preflight_checks_pass_{false}; */
  bool is_active_{false};
};
}  // namespace laser_uav_hw_api

#endif
