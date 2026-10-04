"""iris (2D lidar) ve ev.sdf. ardupilot_gz'deki iris_maze.launch.py'den uyarlandi."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

DUNYA_DOSYASI = Path(__file__).resolve().parent / "ev.sdf"
DUNYA_ADI = "ev"   # sdf icindeki <world name> ile ayni olmali


def generate_launch_description():
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_project_bringup, "launch", "robots", "iris_lidar.launch.py"])]
        ),
        launch_arguments={"world_name": DUNYA_ADI}.items(),
        condition=IfCondition(LaunchConfiguration("spawn_robot")),
    )

    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'),
        launch_arguments={"gz_args": f"-v4 -s -r {DUNYA_DOSYASI}"}.items(),
        condition=IfCondition(LaunchConfiguration("use_gz_sim_server")),
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'),
        launch_arguments={"gz_args": "-v4 -g"}.items(),
        condition=IfCondition(LaunchConfiguration("use_gz_sim_gui")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="", description="Robot namespace."),
            DeclareLaunchArgument("use_gz_sim_server", default_value="true", description="Run the Gazebo server."),
            DeclareLaunchArgument("use_gz_sim_gui", default_value="true", description="Run the Gazebo GUI."),
            DeclareLaunchArgument("spawn_robot", default_value="true", description="Spawn the robot and start SITL+ROS."),
            DeclareLaunchArgument("rviz", default_value="false", description="Kullanilmiyor, slam-baslat rviz:=false veriyor."),
            gz_sim_server,
            gz_sim_gui,
            robot,
        ]
    )
