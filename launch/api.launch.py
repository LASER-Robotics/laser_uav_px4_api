#!/usr/bin/env python3

import launch
import os

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ld = launch.LaunchDescription()

    uav_name = os.getenv('UAV_NAME', 'uav1')
    real_uav = os.getenv('REAL_UAV', 'false')
    use_sim_time = (real_uav == 'false')

    ld.add_action(DeclareLaunchArgument(
        'control_input_mode',
        default_value='angular_rates_and_thrust',
        description='Control input mode: individual_thrust or angular_rates_and_thrust'
    ))

    ld.add_action(DeclareLaunchArgument(
        'api_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('laser_uav_px4_api'),
            'params', 'api.yaml'
        ]),
        description='Full path to the API parameters file.'
    ))

    control_input_mode = LaunchConfiguration('control_input_mode')
    api_file = LaunchConfiguration('api_file')

    container = ComposableNodeContainer(
        namespace=uav_name,
        name=uav_name + '_px4_api_container',
        package='rclcpp_components',
        executable='component_container_mt', 
        output='screen',
        
        composable_node_descriptions=[
            ComposableNode(
                package='laser_uav_px4_api',
                plugin='laser_uav_px4_api::LaserUavPx4Api',
                namespace=uav_name,
                name='px4_api',
                
                parameters=[
                    api_file,
                    {
                        'use_sim_time': use_sim_time,
                    }
                ],
                
                remappings=[
                    ('vehicle_command_px4_out', '/' + uav_name + '/fmu/in/vehicle_command'),
                    ('attitude_rates_reference_px4_out', '/' + uav_name + '/fmu/in/vehicle_rates_setpoint'),
                    ('motor_speed_reference_px4_out', '/' + uav_name + '/fmu/in/actuator_motors'),
                    ('offboard_control_mode_px4_out', '/' + uav_name + '/fmu/in/offboard_control_mode'),
                    
                    ('vehicle_odometry_px4_in', '/' + uav_name + '/fmu/out/vehicle_odometry'),
                    ('sensor_gyro_px4_in', '/' + uav_name + '/fmu/out/sensor_gyro'),
                    ('sensor_accel_px4_in', '/' + uav_name + '/fmu/out/sensor_accel'),
                    ('vehicle_status_px4_in', '/' + uav_name + '/fmu/out/vehicle_status'),
                    ('vehicle_control_mode_px4_in', '/' + uav_name + '/fmu/out/vehicle_control_mode'),
                    ('esc_status_px4_in', '/' + uav_name + '/fmu/out/esc_status'),
                    
                    ('motor_speed_reference_in', '/' + uav_name + '/control_manager/motor_speed_reference'),
                    ('attitude_rates_thrust_in', '/' + uav_name + '/control_manager/attitude_rates_thrust'),
                    
                    ('odometry', '/' + uav_name + '/px4_api/odometry'),
                    ('imu', '/' + uav_name + '/px4_api/imu'),
                    ('api_diagnostics', '/' + uav_name + '/px4_api/diagnostics'),
                    ('motor_speed_estimation_out', '/' + uav_name + '/px4_api/motor_speed_estimated'),
                    
                    ('arm', '/' + uav_name + '/px4_api/arm'),
                    ('disarm', '/' + uav_name + '/px4_api/disarm'),
                ]
            )
        ]
    )

    ld.add_action(container)

    return ld