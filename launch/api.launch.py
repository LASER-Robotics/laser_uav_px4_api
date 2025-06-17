from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch.actions import RegisterEventHandler, EmitEvent

from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare

from launch.events import matches_action
from launch.event_handlers.on_process_start import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

import lifecycle_msgs.msg

import os

def generate_launch_description():
    uav_name = os.environ['uav_name']

    if uav_name == "":
        print("The uav name dont set up in yours enviroment variables")
        return

    #Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'api_file',
            default_value=PathJoinSubstitution([FindPackageShare('laser_uav_px4_api'),
                                                'params', 'api.yaml']),
            description='Full path to the file with the all parameters.'
        )
    )

#Initialize arguments
    api_file = LaunchConfiguration('api_file')

    api_lifecycle_node = LifecycleNode(
        package='laser_uav_px4_api',
        executable='api',
        name='px4_api',
        namespace=uav_name,
        output='screen',
        parameters=[api_file],
        remappings=[
            ('/' + uav_name + '/vehicle_command_px4_out', '/' + uav_name + '/fmu/in/vehicle_command'),
            ('/' + uav_name + '/attitude_rates_setpoint_px4_out', '/' + uav_name + '/fmu/in/vehicle_rates_setpoint'),
            ('/' + uav_name + '/offboard_control_mode_px4_out', '/' + uav_name + '/fmu/in/offboard_control_mode'),
            ('/' + uav_name + '/vehicle_odometry_px4_in', '/' + uav_name + '/fmu/out/vehicle_odometry'),
            ('/' + uav_name + '/vehicle_control_mode_px4_in', '/' + uav_name + '/fmu/out/vehicle_control_mode'),
            ('/' + uav_name + '/attitude_rates_thrust_in', '/' + uav_name + '/px4_api/attitude_rates_thrust'),
            ('/' + uav_name + '/odometry', '/' + uav_name + '/estimation_manager/estimation'),
            ('/' + uav_name + '/arm', '/' + uav_name + '/px4_api/arm'),
            ('/' + uav_name + '/disarm', '/' + uav_name + '/px4_api/disarm'),
            ('/' + uav_name + '/api_diagnostics', '/' + uav_name + '/px4_api/diagnostics'),
        ]
    )

    event_handlers = []

    event_handlers.append(
#Right after the node starts, make it take the 'configure' transition.
        RegisterEventHandler(
            OnProcessStart(
                target_action=api_lifecycle_node,
                on_start=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(api_lifecycle_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )),
                ],
            )
        ),
    )

    event_handlers.append(
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=api_lifecycle_node,
                start_state='configuring',
                goal_state='inactive',
                entities=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(api_lifecycle_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            )
        ),
    )

    ld = LaunchDescription()

#Declare the arguments
    for argument in declared_arguments:
        ld.add_action(argument)

#Add client node
    ld.add_action(api_lifecycle_node)

#Add event handlers
    for event_handler in event_handlers:
        ld.add_action(event_handler)

    return ld
