from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    pkg_share = os.path.join(
        os.getenv('COLCON_PREFIX_PATH', '/opt/ros/humble'), 
        'share', 
        'ur5e_collision_avoidance'
    )

    urdf_file = os.path.join(pkg_share, 'urdf', 'dual_ur5e.urdf.xacro')

    return LaunchDescription([

        # Joint State Publisher GUI (for visualization)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_file}]
        ),

        # RViz2 visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'dual_ur5e.rviz')]
        ),
    ])