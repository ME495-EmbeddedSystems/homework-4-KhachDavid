from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    # Create the launch description and add actions
    return LaunchDescription([
        # Include the launch description for the navigation stack
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('nubot_nav'), 'launch', 'manual_explore.launch.xml']
                )
            )
        ),
        Node(
            package='nubot_nav',
            executable='explore',
            name='explore',
            output='screen'
        )
    ])
