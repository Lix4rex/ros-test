import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os
import xacro 

packageName="ros2_control_car"

xacroRelativePath="model/model.xacro"
rvizRelativePath="config/config.rviz"

ros2controlRelativePath="config/robot_controller.yaml"


def generate_launch_description():
        pkgPath=launch_ros.substitutions.FindPackageShare(package=packageName).find(packageName)

        xacroModelPath=os.path.join(pkgPath, xacroRelativePath)
        rvizConfigPath=os.path.join(pkgPath, rvizRelativePath)
        ros2controlPath=os.path.join(pkgPath, ros2controlRelativePath)


        print(xacroModelPath)

        robot_desc = xacro.process_file(xacroModelPath).toxml()
        robot_description={"robot_description": robot_desc}


        declared_arguments = []
        declared_arguments.append(
                launch.actions.DeclareLaunchArgument(name="gui", default_value="true",
                        description="Start the RViz2 GUI")
        )

        gui = LaunchConfiguration("gui")

        gazebo = launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                        [launch_ros.substitutions.FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
                ),
                launch_arguments=[("gz_args", " -r -v 3 empty.sdf")],
                condition=launch.conditions.IfCondition(gui)
        )

        gazebo_headless = launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                        [launch_ros.substitutions.FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
                ),
                launch_arguments=[("gz_args"), ["--headless-rendering -s -r -v 3 empty.sdf"]],
                condition=launch.conditions.UnlessCondition(gui)
        )

        gazebo_bridge = launch_ros.actions.Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]"],
                output="screen"
        )

        gz_spawn_entity = launch_ros.actions.Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                        "-topic",
                        "/robot_description",
                        "-name",
                        "robot_system_position",
                        "-allow_renaming",
                        "true",
                        "-x", "0",
                        "-y", "0",
                        "-z", "0.2"
                ]
        )

        lidar_node = launch_ros.actions.Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                        "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
                ],
                output="screen"
        )

        lidar_tf_node = launch_ros.actions.Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=[
                        '0', '0', '0',        # translation
                        '0', '0', '0',        # rotation
                        'base_link',
                        'robot_system_position/base_link/gpu_lidar'
                ],
                output='screen'
        )




        robot_state_publisher_node=launch_ros.actions.Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[robot_description,
                        {
                                "ignore_timestamp": True,
                                "publish_frequency": 50.0
                        }
                ]
        )

        rviz_node = launch_ros.actions.Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rvizConfigPath]
        )

        control_node = launch_ros.actions.Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_description, ros2controlPath],
                output="both"
        )

        joint_state_broadcaster_spawner = launch_ros.actions.Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
        )

        robot_controller_spawner = launch_ros.actions.Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_position_controller", "omni_wheel_drive_controller", "--param-file", ros2controlPath]
        )


        nodeList=[gazebo,
                  gazebo_headless,
                  gazebo_bridge,
                  gz_spawn_entity,

                  lidar_node,
                  lidar_tf_node,

                  robot_state_publisher_node,
                  rviz_node,
                  control_node,
                  joint_state_broadcaster_spawner,
                  robot_controller_spawner
        ]

        return launch.LaunchDescription(declared_arguments + nodeList)