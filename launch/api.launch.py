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


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='uav1',
            description='Top-level namespace.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([FindPackageShare('laser_uav_px4_api'),
                                                'params', 'default.yaml']),
            description='Full path to the file with the all parameters.'
        )
    )

    # Initialize arguments
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')

    # Replace '<robot_namespace>' tag with the real robot namespace
    # params_file = ReplaceString(
    #     source_file=params_file,
    #     replacements={'<robot_namespace>': namespace}
    # )

    # configured_params = RewrittenYaml(
    #     source_file=params_file,
    #     root_key=namespace,
    #     param_rewrites={},
    #     convert_types=True
    # )

    api_lifecycle_node = LifecycleNode(
        package='laser_uav_px4_api',
        executable='api',
        name='px4_api',
        namespace=namespace,
        output='screen',
        # parameters=[configured_params]
        remappings=[
            ('/uav1/vehicle_command_px4_out', '/fmu/in/vehicle_command'),
            ('/uav1/torque_setpoint_px4_out', '/fmu/in/vehicle_torque_setpoint'),
            ('/uav1/thrust_setpoint_px4_out', '/fmu/in/vehicle_thrust_setpoint'),
            ('/uav1/offboard_control_mode_px4_out', '/fmu/in/offboard_control_mode'),
            ('/uav1/trajectory_setpoint_px4_out', '/fmu/in/trajectory_setpoint'),
            ('/uav1/vehicle_odometry_px4_in', '/fmu/out/vehicle_odometry'),
            ('/uav1/vehicle_control_mode_px4_in', '/fmu/out/vehicle_control_mode'),
        ]
    )

    event_handlers = []

    event_handlers.append(
        # Right after the node starts, make it take the 'configure' transition.
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

    # Declare the arguments
    for argument in declared_arguments:
        ld.add_action(argument)

    # Add client node
    ld.add_action(api_lifecycle_node)

    # Add event handlers
    for event_handler in event_handlers:
        ld.add_action(event_handler)

    return ld
