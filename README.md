# laser_uav_px4_api

This API serves as the communication bridge between the autopilot hardware and the high-level control system on the companion computer, managing time synchronization and reference frame conversions.

## Communication interfaces

### Topics

``` yaml
topic: /uav1/control_manager/motor_speed_reference
type: laser_msgs::msg::MotorSpeed
I/O: input

topic: /uav1/control_manager/attitude_rates_thrust
type:laser_msgs::msg::AttitudeRatesAndThrust
I/O: input

topic: /uav1/px4_api/imu
type: sensor_msgs::msg::Imu
I/O: output

topic: /uav1/px4_api/motor_speed_estimated
type: laser_msgs::msg::MotorSpeed
I/O: output

topic: /uav1/px4_api/odometry
type: nav_msgs::msg::Odometry
I/O: output
```

**/uav1/control_manager/motor_speed_reference**: This topic is used to send normalized individual thrust commands when the controller is in the corresponding mode.

**/uav1/control_manager/attitude_rates_thrust**: This topic is used to send angular velocity references and a total normalized thrust value when in the corresponding mode.

Note: The topics above are namespaced with /control_manager/ because this component is the intended source of control inputs for the autopilot pipeline.

**/uav1/px4_api/imu** is a topic that provides information from the autopilot's internal IMU.

**/uav1/px4_api/motor_speed_estimated** is a topic that provides the estimated current motor speed.

**/uav1/px4_api/odometry** is a topic that provides the estimated state of the UAV, including its position, orientation, and velocity.

### Services

``` yaml
service: /uav1/arm
type: std_srvs::srv::Trigger

service: /uav1/disarm
type: std_srvs::srv::Trigger
```

**/uav1/arm** provides the request to arm the uav engines [ONLY SIMULATION].

**/uav1/disarm** provides the request to disarm the uav engines [ONLY SIMULATION].
