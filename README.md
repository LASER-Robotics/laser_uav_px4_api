# laser_uav_px4_api

This API provides a level of abstraction so that mission strategies using UAV are easier to implement without having to worry about synchronization bureaucracy or event arrival.

## Communication interfaces

### Topics

``` yaml
topic: /uav1/goto
type: geometry_msgs:msg:Pose
I/O: input

topic: /uav1/goto_relative
type: geometry_msgs:msg:Pose
I/O: input

topic: /uav1/have_goal
type: std_msgs::msg::Bool
I/O: output

topic: /uav1/odometry
type: nav_msgs::msg::Odometry
I/O: output
```

**/uav1/goto** is a topic that makes it possible to send a setpoint to the uav. The setpoint coordinates for this topic are absolute in relation to the world frame.

**/uav1/goto_relative** is a topic that makes it possible to send a setpoint to the uav. The setpoint coordinates for this topic are relative to the uav frame.

**/uav1/have_goal** is a topic that notifies whether the uav has a goal or not, this facilitates the implementation of state machines in mission algorithms, being the state change flag.

**/uav1/odometry** is a topic that provides the information of estimating states of the uav its position, orientation and speed.

### Services

``` yaml
service: /uav1/takeoff
type: std_srvs::srv::Trigger

service: /uav1/land
type: std_srvs::srv::Trigger

service: /uav1/arm
type: std_srvs::srv::Trigger

service: /uav1/disarm
type: std_srvs::srv::Trigger
```

**/uav1/takeoff** provides takeoff request for uav.

**/uav1/land** provides landing request for uav.

**/uav1/arm** provides the request to arm the uav engines.

**/uav1/disarm** provides the request to disarm the uav engines.
