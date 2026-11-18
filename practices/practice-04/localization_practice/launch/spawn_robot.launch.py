import os
import math
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch_ros.event_handlers import OnStateTransition
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
import launch
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import (
    WaitForControllerConnection,
)


def check_map(context):
    if LaunchConfiguration("use_localization").perform(context) == "true":
        map_yaml = LaunchConfiguration("map_yaml").perform(context)
        if not os.path.isfile(map_yaml):
            raise RuntimeError(
                f"Map YAML not found: {map_yaml}. "
                "Provide map_yaml:=/absolute/path/to/map.yaml or use "
                "use_localization:=false to inspect the scene."
            )
    return []


def generate_launch_description():
    webots_turtlebot_package_dir = get_package_share_directory(
        "webots_ros2_turtlebot"
    )
    package_dir = get_package_share_directory("localization_practice")

    scene = LaunchConfiguration("scene")
    localization_mode = LaunchConfiguration("localization_mode")
    use_localization = LaunchConfiguration("use_localization")

    error_x = LaunchConfiguration("error_x")
    error_y = LaunchConfiguration("error_y")
    error_yaw = LaunchConfiguration("error_yaw")
    initial_pose_x = PythonExpression(
        ["float(", LaunchConfiguration("start_x"), ") + float(", error_x, ")"]
    )
    initial_pose_y = PythonExpression(
        ["float(", LaunchConfiguration("start_y"), ") + float(", error_y, ")"]
    )
    initial_pose_yaw = PythonExpression(
        ["float(", LaunchConfiguration("start_yaw"), ") + float(", error_yaw, ")"]
    )

    world = PythonExpression(["'", scene, ".wbt'"])
    mode = LaunchConfiguration("mode")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, "worlds", world]),
        mode=mode,
        ros2_supervisor=False,
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(package_dir, "rviz", "config.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": '<robot name=""><link name=""/></robot>'}
        ],
    )

    footprint_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"],
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "yaml_filename": LaunchConfiguration("map_yaml"),
            }
        ],
        condition=IfCondition(use_localization),
    )
    amcl = LifecycleNode(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        namespace="",
        output="screen",
        parameters=[
            os.path.join(package_dir, "config", "amcl.yaml"),
            {
                "use_sim_time": use_sim_time,
                "min_particles": ParameterValue(
                    LaunchConfiguration("min_particles"), value_type=int
                ),
                "max_particles": ParameterValue(
                    LaunchConfiguration("max_particles"), value_type=int
                ),
                "set_initial_pose": ParameterValue(
                    PythonExpression(
                        ["'", localization_mode, "' == 'initial'"]
                    ),
                    value_type=bool,
                ),
                "initial_pose.x": ParameterValue(
                    initial_pose_x, value_type=float
                ),
                "initial_pose.y": ParameterValue(
                    initial_pose_y, value_type=float
                ),
                "initial_pose.yaw": ParameterValue(
                    initial_pose_yaw, value_type=float
                ),
            },
        ],
        condition=IfCondition(use_localization),
    )

    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["map_server", "amcl"],
            }
        ],
        condition=IfCondition(use_localization),
    )

    global_initializer = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=amcl,
            goal_state="active",
            entities=[
                TimerAction(
                    period=2.0,
                    actions=[
                        ExecuteProcess(
                            cmd=[
                                "ros2",
                                "service",
                                "call",
                                "/reinitialize_global_localization",
                                "std_srvs/srv/Empty",
                                "{}",
                            ],
                            output="screen",
                        )
                    ],
                )
            ],
        ),
        condition=IfCondition(
            PythonExpression(
                ["'", use_localization, "' == 'true' and '",
                 localization_mode, "' == 'global'"]
            )
        ),
    )

    supervisor_driver = WebotsController(
        robot_name="ground_truth_supervisor",
        parameters=[
            {
                "robot_description": os.path.join(
                    package_dir, "urdf", "ground_truth_supervisor.urdf"
                ),
                "use_sim_time": use_sim_time,
            }
        ],
        respawn=True,
    )

    # ROS control spawners
    controller_manager_timeout = ["--controller-manager-timeout", "50"]

    diffdrive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "diffdrive_controller",
            "--controller-ros-args=-p diffdrive_controller:wheel_separation_multiplier:=1.1",
        ]
        + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster"] + controller_manager_timeout,
    )
    circle_motion = Node(
        package="localization_practice",
        executable="circle_motion",
        parameters=[{
            "use_sim_time": use_sim_time,
            "wait_for_amcl": ParameterValue(use_localization, value_type=bool),
        }],
        condition=IfCondition(PythonExpression(["'", scene, "' == 'digits'"])),
    )
    start_motion = RegisterEventHandler(
        OnProcessExit(
            target_action=diffdrive_controller_spawner,
            on_exit=lambda event, context: [circle_motion] if event.returncode == 0 else [],
        )
    )
    collector = Node(
        package="localization_practice",
        executable="collect_localization",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "duration": ParameterValue(LaunchConfiguration("duration"), value_type=float),
            "position_threshold": ParameterValue(
                LaunchConfiguration("position_threshold"), value_type=float
            ),
            "yaw_threshold": ParameterValue(
                LaunchConfiguration("yaw_threshold"), value_type=float
            ),
            "confirmation_duration": ParameterValue(
                LaunchConfiguration("confirmation_duration"), value_type=float
            ),
            "output_prefix": ParameterValue(
                LaunchConfiguration("output_prefix"), value_type=str
            ),
        }],
        condition=IfCondition(PythonExpression(
            ["'", scene, "' == 'digits' and '", use_localization, "' == 'true'"]
        )),
    )
    ros_control_spawners = [
        diffdrive_controller_spawner,
        joint_state_broadcaster_spawner,
    ]

    robot_description_path = os.path.join(
        webots_turtlebot_package_dir, "resource", "turtlebot_webots.urdf"
    )
    ros2_control_params = os.path.join(
        webots_turtlebot_package_dir, "resource", "ros2control.yml"
    )
    turtlebot_driver = WebotsController(
        robot_name="TurtleBot3Burger",
        parameters=[
            {
                "robot_description": robot_description_path,
                "use_sim_time": use_sim_time,
                "set_robot_state_publisher": True,
            },
            ros2_control_params,
        ],
        remappings=[
            ("/diffdrive_controller/cmd_vel", "/cmd_vel"),
            ("/diffdrive_controller/odom", "/odom"),
        ],
        respawn=True,
    )

    # Wait for the simulation to be ready to start control nodes
    waiting_nodes = WaitForControllerConnection(
        target_driver=turtlebot_driver, nodes_to_start=ros_control_spawners
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scene",
                default_value="digits",
                choices=["digits", "three_boxes", "one_box"],
                description="Scene and map",
            ),
            DeclareLaunchArgument(
                "map_yaml",
                default_value=PathJoinSubstitution(
                    [package_dir, "maps", PythonExpression(["'", scene, ".yaml'"])]
                ),
                description="Absolute path to the occupancy map YAML",
            ),
            DeclareLaunchArgument(
                "use_localization",
                default_value="true",
                choices=["true", "false"],
                description="Start map server and AMCL; false previews the scene",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                choices=["true", "false"],
                description="Use the simulation clock",
            ),
            DeclareLaunchArgument(
                "start_x",
                default_value=PythonExpression(
                    ["2.0 if '", scene, "' == 'digits' else 1.0"]
                ),
                description="True initial x from the world, metres",
            ),
            DeclareLaunchArgument(
                "start_y",
                default_value="0.0",
                description="True initial y from the world, metres",
            ),
            DeclareLaunchArgument(
                "start_yaw",
                default_value=str(math.pi / 2),
                description="True initial yaw from the world, radians",
            ),
            DeclareLaunchArgument(
                "min_particles",
                default_value="500",
                description="AMCL minimum particle count",
            ),
            DeclareLaunchArgument(
                "max_particles",
                default_value="15000",
                description="AMCL maximum particle count",
            ),
            DeclareLaunchArgument("duration", default_value="200.0"),
            DeclareLaunchArgument("position_threshold", default_value="0.15"),
            DeclareLaunchArgument("yaw_threshold", default_value="0.12"),
            DeclareLaunchArgument("confirmation_duration", default_value="10.0"),
            DeclareLaunchArgument("output_prefix", default_value="results/experiment"),
            DeclareLaunchArgument(
                "localization_mode",
                default_value="global",
                choices=["global", "initial"],
                description="AMCL initialization",
            ),
            DeclareLaunchArgument(
                "error_x",
                default_value="0.0",
                description="Initial x error, metres",
            ),
            DeclareLaunchArgument(
                "error_y",
                default_value="0.0",
                description="Initial y error, metres",
            ),
            DeclareLaunchArgument(
                "error_yaw",
                default_value="0.0",
                description="Initial yaw error, radians",
            ),
            DeclareLaunchArgument(
                "mode",
                default_value="realtime",
                description="Webots startup mode",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz with the localization configuration",
            ),
            OpaqueFunction(function=check_map),
            webots,
            rviz,
            robot_state_publisher,
            footprint_publisher,
            map_server,
            global_initializer,
            amcl,
            lifecycle,
            collector,
            supervisor_driver,
            turtlebot_driver,
            start_motion,
            waiting_nodes,
            # This action will kill all nodes once the Webots simulation has exited
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[
                        launch.actions.EmitEvent(event=launch.events.Shutdown())
                    ],
                )
            ),
        ]
    )
