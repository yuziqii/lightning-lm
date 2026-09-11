from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    nav2_share = get_package_share_directory("nav2_bringup")

    map_file = LaunchConfiguration("map")
    lightning_config = LaunchConfiguration("lightning_config")
    nav2_params = LaunchConfiguration("nav2_params")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_file, "use_sim_time": use_sim_time}],
    )

    map_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": ["map_server"],
            }
        ],
    )

    lightning_localization = Node(
        package="lightning",
        executable="run_loc_online",
        name="lightning_localization",
        output="screen",
        arguments=["--config", lightning_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_share, "launch", "navigation_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "params_file": nav2_params,
            "use_composition": "False",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map", description="Absolute path to the Nav2 map YAML file."),
            DeclareLaunchArgument(
                "lightning_config", description="Absolute path to the Lightning-LM localization YAML file."
            ),
            DeclareLaunchArgument(
                "nav2_params",
                default_value=PathJoinSubstitution([nav2_share, "params", "nav2_params.yaml"]),
                description="Absolute path to the Nav2 parameter YAML file.",
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use the ROS simulation clock."
            ),
            DeclareLaunchArgument("autostart", default_value="true", description="Autostart Nav2 lifecycle nodes."),
            map_server,
            map_lifecycle_manager,
            lightning_localization,
            navigation,
        ]
    )