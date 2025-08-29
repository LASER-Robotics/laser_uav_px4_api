# LASER UAV PX4 API

This API serves as the communication bridge between the autopilot hardware and the high/low-level control system on the companion computer, managing time synchronization and reference frame conversions.

## Communication Interfaces

This package provides its functionality through a set of ROS 2 topics and services.

### Topics

The API subscribes to topics for control commands and publishes topics with sensor and state information from the autopilot.

#### Input Topics (Subscribed by API)

> **Note:** The topics below are namespaced with `/control_manager/` because this component is the intended source of control inputs for the autopilot pipeline.

---
### `/uav1/control_manager/motor_speed_reference`
-   **Type:** `laser_msgs::msg::MotorSpeed`
-   **Direction:** Input
-   **Description:** Used to send normalized individual thrust commands when the controller is operating in this specific mode.

---
### `/uav1/control_manager/attitude_rates_thrust`
-   **Type:** `laser_msgs::msg::AttitudeRatesAndThrust`
-   **Direction:** Input
-   **Description:** Used to send angular velocity setpoints and a total normalized thrust value when the controller is in the corresponding mode.

---

#### Output Topics (Published by API)

---
### `/uav1/px4_api/imu`
-   **Type:** `sensor_msgs::msg::Imu`
-   **Direction:** Output
-   **Description:** Publishes filtered data from the autopilot's internal IMU (Inertial Measurement Unit).

---
### `/uav1/px4_api/motor_speed_estimated`
-   **Type:** `laser_msgs::msg::MotorSpeed`
-   **Direction:** Output
-   **Description:** Publishes the current estimated speed of each motor.

---
### `/uav1/px4_api/odometry`
-   **Type:** `nav_msgs::msg::Odometry`
-   **Direction:** Output
-   **Description:** Publishes the estimated state of the UAV, including its position, orientation, and velocities.

---

### Services

The API provides services for basic vehicle commands.

---
### `/uav1/arm`
-   **Type:** `std_srvs::srv::Trigger`
-   **Description:** Service to arm the UAV's motors. **[SIMULATION ONLY]**

---
### `/uav1/disarm`
-   **Type:** `std_srvs::srv::Trigger`
-   **Description:** Service to disarm the UAV's motors. **[SIMULATION ONLY]**
---
