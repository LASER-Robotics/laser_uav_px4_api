# LASER UAV PX4 API

This API serves as the communication bridge between the autopilot hardware and the high/low-level control system on the companion computer, managing time synchronization and reference frame conversions.

## Communication Interfaces

This package provides its functionality through a set of ROS 2 topics and services.

### Topics

#### Subscribed Topics
- **/uav1/control_manager/motor_speed_reference:** Used to send normalized individual thrust commands when the controller is in the corresponding mode.
- **/uav1/control_manager/attitude_rates_thrust:** Used to send angular velocity setpoints and a total normalized thrust value when the controller is in the corresponding mode.

#### Published Topics
- **/uav1/px4_api/imu:** Publishes filtered data from the autopilot's internal IMU (Inertial Measurement Unit).
- **/uav1/px4_api/motor_speed_estimated:** Publishes the current estimated speed of each motor.
- **/uav1/px4_api/odometry:** Publishes the estimated state of the UAV, including its position, orientation, and velocities.

### Services
- **/uav1/px4_api/arm:** Service to arm the UAV's motors. **[SIMULATION ONLY]**
- **/uav1/px4_api/disarm:** Service to disarm the UAV's motors. **[SIMULATION ONLY]**
