from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

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
