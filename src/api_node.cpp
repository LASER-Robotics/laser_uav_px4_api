#include "laser_uav_px4_api/api_node.hpp"

namespace laser_uav_px4_api {

LaserUavPx4Api::LaserUavPx4Api(const rclcpp::NodeOptions& options) : Node("laser_uav_px4_api", options) {
    if (!initialize()) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize LaserUavPx4Api");
    }
}

LaserUavPx4Api::~LaserUavPx4Api() {
    is_active_ = false;
    is_initialized_ = false;
}

  // substitute for both the ApiNode() and the on_configure() functions of hubmle version
bool LaserUavPx4Api::initialize() {

    RCLCPP_INFO(this->get_logger(), "Initializing LaserUavPx4Api...");

    clock_ = this->get_clock();

    // parameter declarations
    this->declare_parameter("control_input_mode", rclcpp::ParameterValue(""));
    this->declare_parameter("rate.pub_offboard_control_mode", rclcpp::ParameterValue(100.0));
    this->declare_parameter("rate.pub_api_diagnostics", rclcpp::ParameterValue(10.0));


    // callback groups creation
    // callback_group_high_freq_ = node_->create_callback_group(
    //                           rclcpp::CallbackGroupType::MutuallyExclusive);

    // callback_group_services_ = node_->create_callback_group(
    //                           rclcpp::CallbackGroupType::MutuallyExclusive);

    // callback_group_low_freq_ = node_->create_callback_group(
    //                           rclcpp::CallbackGroupType::MutuallyExclusive);

    // coordinate transformations
    ned_enu_quaternion_rotation_ = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                                      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
    frd_flu_rotation_            = Eigen::Quaterniond(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                                      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
    frd_flu_affine_              = Eigen::Affine3d(frd_flu_rotation_);

    ned_enu_reflection_xy_ = Eigen::PermutationMatrix<3>(Eigen::Vector3i(1, 0, 2));
    ned_enu_reflection_z_  = Eigen::DiagonalMatrix<double, 3>(1, 1, -1);


    // target system id
    const char *real_uav = std::getenv("REAL_UAV");
    target_system_       = 1;
    if (real_uav != nullptr) {
      std::string real_uav_str = std::string(real_uav);
      if (real_uav_str == "false") {
        real_uav_            = false;
        const char *uav_name = std::getenv("UAV_NAME");
        if (uav_name != nullptr) {
          std::smatch match;
          std::regex  re("(\\d+)$");

          std::string uav_name_str = std::string(uav_name);
          if (std::regex_search(uav_name_str, match, re)) {
            std::string numero_str = match[1];
            target_system_         = std::stoi(std::string(match[1]));
          }
        }
      } else {
        real_uav_ = true;
      }
    }

    getParameters();
    configPubSub();
    configTimers();
    configServices();

    
    is_initialized_ = true;
    
    activate();
    
    return true;
    
  }
  
  void LaserUavPx4Api::getParameters() {
  try
  {
    this->get_parameter("control_input_mode", _control_input_mode_);
    this->get_parameter("rate.pub_offboard_control_mode", _rate_pub_offboard_control_mode_px4_);
    this->get_parameter("rate.pub_api_diagnostics", _rate_pub_api_diagnostics_);
  }
  catch (const rclcpp::exceptions::ParameterNotDeclaredException& e) 
  {
      RCLCPP_ERROR(this->get_logger(), "Parameter not declared: %s", e.what());
      throw;
  }
  
  
}

void LaserUavPx4Api::configPubSub() {
  RCLCPP_INFO(this->get_logger(), "initPubSub");


  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Pubs and Subs for Px4 topics
  sub_odometry_px4_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("vehicle_odometry_px4_in", rclcpp::SensorDataQoS(),
                                                                          std::bind(&LaserUavPx4Api::subOdometryPx4, this, std::placeholders::_1));

  sub_sensor_gyro_px4_ = this->create_subscription<px4_msgs::msg::SensorGyro>("sensor_gyro_px4_in", rclcpp::SensorDataQoS(),
                                                                                std::bind(&LaserUavPx4Api::subSensorGyroPx4, this, std::placeholders::_1));
  sub_sensor_accel_px4_ = this->create_subscription<px4_msgs::msg::SensorAccel>("sensor_accel_px4_in", rclcpp::SensorDataQoS(),
                                                                                std::bind(&LaserUavPx4Api::subSensorAccelPx4, this, std::placeholders::_1));

  sub_vehicle_status_px4_ = this->create_subscription<px4_msgs::msg::VehicleStatus>("vehicle_status_px4_in", rclcpp::SensorDataQoS(),
                                                                              std::bind(&LaserUavPx4Api::subVehicleStatusPx4, this, std::placeholders::_1));
  sub_control_mode_px4_   = this->create_subscription<px4_msgs::msg::VehicleControlMode>("vehicle_control_mode_px4_in", rclcpp::SensorDataQoS(),
                                                                                 std::bind(&LaserUavPx4Api::subControlModePx4, this, std::placeholders::_1));
  sub_esc_status_px4_     = this->create_subscription<px4_msgs::msg::EscStatus>("esc_status_px4_in", rclcpp::SensorDataQoS(),
                                                                      std::bind(&LaserUavPx4Api::subEscStatusPx4, this, std::placeholders::_1));

  pub_vehicle_command_px4_       = this->create_publisher<px4_msgs::msg::VehicleCommand>("vehicle_command_px4_out", 10);
  pub_offboard_control_mode_px4_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("offboard_control_mode_px4_out", 10);

  // Pubs and Subs for System topics
  pub_api_diagnostics_ = this->create_publisher<laser_msgs::msg::ApiPx4Diagnostics>("api_diagnostics", 10);

  pub_nav_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);

  pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);

  pub_motor_speed_estimation_ = this->create_publisher<laser_msgs::msg::MotorSpeed>("motor_speed_estimation_out", 10);

  if (_control_input_mode_ == "individual_thrust") {
    pub_motor_speed_reference_px4_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>("motor_speed_reference_px4_out", 10);
    sub_motor_speed_reference_     = this->create_subscription<laser_msgs::msg::MotorSpeed>("motor_speed_reference_in", 1,
                                                                                  std::bind(&LaserUavPx4Api::subMotorSpeedReference, this, std::placeholders::_1));
  } else if (_control_input_mode_ == "angular_rates_and_thrust") {
    pub_attitude_rates_reference_px4_        = this->create_publisher<px4_msgs::msg::VehicleRatesSetpoint>("attitude_rates_reference_px4_out", 10);
    sub_attitude_rates_and_thrust_reference_ = this->create_subscription<laser_msgs::msg::AttitudeRatesAndThrust>(
        "attitude_rates_thrust_in", 1, std::bind(&LaserUavPx4Api::subAttitudeRatesAndThrustReference, this, std::placeholders::_1));
  }
}

void LaserUavPx4Api::configTimers() {
  RCLCPP_INFO(this->get_logger(), "initTimers");

  tmr_pub_offboard_control_mode_px4_ = this->create_wall_timer(
                                      std::chrono::duration<double>(1.0 / _rate_pub_offboard_control_mode_px4_),
                                      std::bind(&LaserUavPx4Api::tmrPubOffboardControlModePx4, this),
                                      nullptr); //callback_group_high_freq_

  tmr_pub_api_diagnostics_ = this->create_wall_timer(
                            std::chrono::duration<double>(1.0 / _rate_pub_api_diagnostics_),
                            std::bind(&LaserUavPx4Api::tmrPubApiDiagnostics, this),
                            nullptr); //callback_group_low_freq_

  if (_control_input_mode_ == "individual_thrust") {
    tmr_pub_motor_speed_reference_px4_ = this->create_wall_timer(
                                       std::chrono::duration<double>(1.0 / 500),
                                       std::bind(&LaserUavPx4Api::tmrPubMotorSpeedReferencePx4, this),
                                       nullptr); //callback_group_high_freq_
  }
}

void LaserUavPx4Api::configServices() {
  RCLCPP_INFO(this->get_logger(), "initServices");

  rclcpp::SubscriptionOptions options;
  //options.callback_group = callback_group_services_;

  srv_arm_    = this->create_service<std_srvs::srv::Trigger>(
              "arm",
              std::bind(&LaserUavPx4Api::srvArm, this, std::placeholders::_1, std::placeholders::_2)
              );  //callback_group_services_
  srv_disarm_ = this->create_service<std_srvs::srv::Trigger>(
              "disarm",
              std::bind(&LaserUavPx4Api::srvDisarm, this, std::placeholders::_1, std::placeholders::_2)
              );  //callback_group_services_
}

bool LaserUavPx4Api::activate() {

  RCLCPP_INFO(this->get_logger(), "Activating");

  if (_control_input_mode_ == "individual_thrust") {
    RCLCPP_INFO(this->get_logger(), "Control Input Mode: Individual Thrust");
  } else if (_control_input_mode_ == "angular_rates_and_thrust") {
    RCLCPP_INFO(this->get_logger(), "Control Input Mode: Angular Rates and Thrust");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Invalid Control Input Mode: %s", _control_input_mode_.c_str());
    return false;
  }

  is_active_ = true;

  return true;
}

void LaserUavPx4Api::subControlModePx4(const px4_msgs::msg::VehicleControlMode &msg) {
  if (!is_active_) {
    return;
  }

  api_diagnostics_.armed         = msg.flag_armed;
  offboard_is_enabled_           = msg.flag_control_offboard_enabled;
  api_diagnostics_.offboard_mode = offboard_is_enabled_;
}

void LaserUavPx4Api::subEscStatusPx4(const px4_msgs::msg::EscStatus &msg) {
  if (!is_active_) {
    return;
  }

  laser_msgs::msg::MotorSpeed motor_speed_estimation;

  for (auto i = 0; i < (int)msg.esc_count; i++) {
    motor_speed_estimation.data.push_back(msg.esc[i].esc_rpm * 0.1047);
  }
  motor_speed_estimation.unit_of_measurement = "rad/s";

  pub_motor_speed_estimation_->publish(motor_speed_estimation);
}

void LaserUavPx4Api::subSensorGyroPx4(const px4_msgs::msg::SensorGyro &msg) {
  if (!is_active_) {
    return;
  }

  Eigen::Vector3d frd_to_flu;
  frd_to_flu << msg.x, msg.y, msg.z;
  frd_to_flu = frdToFlu(frd_to_flu);

  imu_.angular_velocity.x = frd_to_flu(0);
  imu_.angular_velocity.y = frd_to_flu(1);
  imu_.angular_velocity.z = frd_to_flu(2);

  imu_.header.stamp    = this->get_clock()->now();
  imu_.header.frame_id = "fcu";
  pub_imu_->publish(imu_);
}

void LaserUavPx4Api::subSensorAccelPx4(const px4_msgs::msg::SensorAccel &msg) {
  if (!is_active_) {
    return;
  }

  Eigen::Vector3d frd_to_flu;
  frd_to_flu << msg.x, msg.y, msg.z;
  frd_to_flu = frdToFlu(frd_to_flu);

  imu_.linear_acceleration.x = frd_to_flu(0);
  imu_.linear_acceleration.y = frd_to_flu(1);
  imu_.linear_acceleration.z = frd_to_flu(2);
}

void LaserUavPx4Api::subVehicleStatusPx4(const px4_msgs::msg::VehicleStatus &msg) {
  if (!is_active_) {
    return;
  }

  if (!fw_preflight_checks_pass_) {
    fw_preflight_checks_pass_ = msg.pre_flight_checks_pass;

    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Current situation of the preflight checks on Autopilot: %s",
                         fw_preflight_checks_pass_ ? "True" : "False");
  } else {
    RCLCPP_INFO_ONCE(this->get_logger(), "Current situation of the preflight checks on Autopilot: %s", fw_preflight_checks_pass_ ? "True" : "False");
  }
}

void LaserUavPx4Api::subOdometryPx4(const px4_msgs::msg::VehicleOdometry &msg) {
  if (!is_active_) {
    return;
  }

  if (!fw_preflight_checks_pass_) {
    return;
  }

  nav_msgs::msg::Odometry current_nav_odometry{};
  current_nav_odometry.header.frame_id = "odom";
  current_nav_odometry.header.stamp    = this->get_clock()->now();
  current_nav_odometry.child_frame_id  = "fcu";

  Eigen::Vector3d ned_to_enu_tf(msg.position[0], msg.position[1], msg.position[2]);
  ned_to_enu_tf = enuToNed(ned_to_enu_tf);

  current_nav_odometry.pose.pose.position.x = ned_to_enu_tf(0);
  current_nav_odometry.pose.pose.position.y = ned_to_enu_tf(1);
  current_nav_odometry.pose.pose.position.z = ned_to_enu_tf(2);

  Eigen::Quaterniond ned_to_enu_orientation_tf(msg.q[0], msg.q[1], msg.q[2], msg.q[3]);
  ned_to_enu_orientation_tf = enuToNedOrientation(ned_to_enu_orientation_tf);
  ned_to_enu_orientation_tf = ned_to_enu_orientation_tf.normalized();
  ned_to_enu_orientation_tf.coeffs() *= -1;

  // --- Multiply by -1 for adjust rotation
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
  ned_to_enu_tf    = ned_to_enu_orientation_tf.conjugate().normalized().toRotationMatrix() * ned_to_enu_tf;

  current_nav_odometry.twist.twist.linear.x = ned_to_enu_tf(0);
  current_nav_odometry.twist.twist.linear.y = ned_to_enu_tf(1);
  current_nav_odometry.twist.twist.linear.z = ned_to_enu_tf(2);

  Eigen::Vector3d frd_to_flu;
  frd_to_flu << msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2];
  frd_to_flu = frdToFlu(frd_to_flu);

  current_nav_odometry.twist.twist.angular.x = frd_to_flu(0);
  current_nav_odometry.twist.twist.angular.y = frd_to_flu(1);
  current_nav_odometry.twist.twist.angular.z = frd_to_flu(2);

  current_nav_odometry.twist.covariance = {msg.velocity_variance[0], 0, 0, 0, 0, 0, 0, msg.velocity_variance[1], 0, 0, 0, 0, 0, 0,
                                           msg.velocity_variance[2], 0, 0, 0, 0, 0, 0, msg.velocity_variance[0], 0, 0, 0, 0, 0, 0,
                                           msg.velocity_variance[1], 0, 0, 0, 0, 0, 0, msg.velocity_variance[2]};

  pub_nav_odometry_->publish(current_nav_odometry);
}

void LaserUavPx4Api::tmrPubOffboardControlModePx4() {
  if (!is_active_) {
    return;
  }

  px4_msgs::msg::OffboardControlMode msg{};

  msg.position          = false;
  msg.velocity          = false;
  msg.acceleration      = false;
  msg.attitude          = false;
  msg.thrust_and_torque = false;
  if (_control_input_mode_ == "individual_thrust") {
    msg.body_rate       = false;
    msg.direct_actuator = true;
  } else if (_control_input_mode_ == "angular_rates_and_thrust") {
    msg.body_rate       = true;
    msg.direct_actuator = false;
  }
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;


  pub_offboard_control_mode_px4_->publish(msg);
}

void LaserUavPx4Api::pubVehicleCommandPx4(int command, float param1, float param2, float param3, float param4, float param5, float param6, float param7) {
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
  msg.target_system    = target_system_;
  msg.target_component = 1;
  msg.source_system    = 1;
  msg.source_component = 1;
  msg.from_external    = true;
  msg.timestamp        = this->get_clock()->now().nanoseconds() / 1000;
  pub_vehicle_command_px4_->publish(msg);
}

void LaserUavPx4Api::tmrPubApiDiagnostics() {
  if (!is_active_) {
    return;
  }

  pub_api_diagnostics_->publish(api_diagnostics_);
}

void LaserUavPx4Api::subAttitudeRatesAndThrustReference(const laser_msgs::msg::AttitudeRatesAndThrust &msg) {
  if (!is_active_) {
    return;
  }

  if (!offboard_is_enabled_) {
    return;
  }

  if (!fw_preflight_checks_pass_) {
    RCLCPP_ERROR(this->get_logger(), "Preflight Checks dont's Pass in Firmware!");
    return;
  }

  px4_msgs::msg::VehicleRatesSetpoint attitude_rates_reference{};

  Eigen::Vector3d flu_to_frd;
  flu_to_frd << msg.roll_rate, msg.pitch_rate, msg.yaw_rate;
  flu_to_frd = frdToFlu(flu_to_frd);

  attitude_rates_reference.roll  = flu_to_frd(0);
  attitude_rates_reference.pitch = flu_to_frd(1);
  attitude_rates_reference.yaw   = flu_to_frd(2);

  attitude_rates_reference.thrust_body[0] = 0;
  attitude_rates_reference.thrust_body[1] = 0;
  attitude_rates_reference.thrust_body[2] = -msg.total_thrust_normalized;

  attitude_rates_reference.timestamp = this->get_clock()->now().nanoseconds() / 1000;

  pub_attitude_rates_reference_px4_->publish(attitude_rates_reference);
}

void LaserUavPx4Api::subMotorSpeedReference(const laser_msgs::msg::MotorSpeed &msg) {
  if (!is_active_) {
    return;
  }

  if (!offboard_is_enabled_) {
    return;
  }

  if (!fw_preflight_checks_pass_) {
    RCLCPP_ERROR(this->get_logger(), "Preflight Checks dont's Pass in Firmware!");
    return;
  }

  for (auto i = 0; i < (int)msg.data.size(); i++) {
    actuator_motors_reference_.control[i] = msg.data[i];
  }

  actuator_motors_reference_.timestamp        = this->get_clock()->now().nanoseconds() / 1000;
  actuator_motors_reference_.timestamp_sample = 0;
}

void LaserUavPx4Api::tmrPubMotorSpeedReferencePx4() {
  if (!is_active_) {
    return;
  }

  if (!offboard_is_enabled_) {
    return;
  }

  if (!fw_preflight_checks_pass_) {
    return;
  }

  if (_control_input_mode_ != "individual_thrust") {
    return;
  }

  pub_motor_speed_reference_px4_->publish(actuator_motors_reference_);
}

void LaserUavPx4Api::srvArm([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                     [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Response>      response) {
  if (!is_active_) {
    return;
  }

  if (real_uav_) {
    response->success = false;
    response->message = "arm requested failed, in real drone use the RC to arm";
    return;
  }

  if (!fw_preflight_checks_pass_) {
    response->success = false;
    response->message = "arm requested failed, preflight checks don't pass in firmware, try again in a few seconds";
    return;
  }

  pubVehicleCommandPx4(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
  pubVehicleCommandPx4(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

  response->success = true;
  response->message = "arm requested success";
}

void LaserUavPx4Api::srvDisarm([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Response>      response) {
  if (!is_active_) {
    return;
  }

  if (real_uav_) {
    response->success = false;
    response->message = "disarm requested failed, in real drone use the RC to disarm";
    return;
  }

  pubVehicleCommandPx4(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

  response->success = true;
  response->message = "disarm requested success";
}

Eigen::Vector3d LaserUavPx4Api::enuToNed(Eigen::Vector3d p) {
  return ned_enu_reflection_xy_ * (ned_enu_reflection_z_ * p);
}

Eigen::Vector3d LaserUavPx4Api::frdToFlu(Eigen::Vector3d p) {
  return frd_flu_affine_ * p;
}

Eigen::Quaterniond LaserUavPx4Api::enuToNedOrientation(Eigen::Quaterniond q) {
  return (ned_enu_quaternion_rotation_ * q) * frd_flu_rotation_;
}

}// namespace laser_uav_px4_api

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(laser_uav_px4_api::LaserUavPx4Api)